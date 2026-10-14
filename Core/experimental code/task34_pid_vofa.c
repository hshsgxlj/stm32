/**
 * @file    task34_pid_vofa.c
 * @brief   实验3任务4：B电机 10 ms 增量式速度闭环与 VOFA+ 双通道输出
 * @note    target/actual 使用 RPM；I2 临时输出 PID PWM 便于调试。
 */

#include "task34_pid_vofa.h"

#include "main.h"
#include "motor.h"
#include "task33_encoder_periodic.h"

#include <stdio.h>

/* Effective scale calibrated from the observed wheel speed (~12 s/rev at displayed ~200 RPM). */
#define TASK34_COUNTS_PER_OUTPUT_REV   62400L
#define TASK34_FILTER_SAMPLES          10U
#define TASK34_CONTROL_PERIOD_S        0.01f
#define TASK34_REPORT_PERIOD_MS        100U
#define TASK34_STARTUP_STEP_DELAY_MS   500U
#define TASK34_DEFAULT_TARGET_RPM      120
#define TASK34_TARGET_MAX_RPM          150
#define TASK34_PWM_LIMIT               500.0f
#define TASK34_PWM_STEP_LIMIT          4.0f
#define TASK34_FEEDFORWARD_PWM_PER_RPM 3.18f

/*
 * 增量式 PID 参数单位：
 * Kp: PWM/RPM, Ki: PWM/(RPM*s), Kd: PWM*s/RPM。
 * 使用温和初值；车轮悬空后可根据 VOFA+ 曲线小步调整。
 */
#define TASK34_PID_KP                  0.10f
#define TASK34_PID_KI                  0.6f
#define TASK34_PID_KD                  0.00005f

extern UART_HandleTypeDef huart1;

static volatile int16_t task34_target_rpm;
static volatile int32_t task34_actual_rpm;
static volatile int16_t task34_output;
static float task34_output_accumulator;
static volatile uint8_t task34_active;

static int16_t task34_delta_history[TASK34_FILTER_SAMPLES];
static int32_t task34_delta_sum;
static uint8_t task34_filter_count;
static uint8_t task34_filter_index;
static float task34_previous_error;
static float task34_previous_previous_error;
static uint32_t task34_start_tick;
static uint32_t task34_last_report_tick;
static uint8_t task34_step_pending;

static int16_t Task34_LimitTarget(int16_t target_rpm)
{
    if (target_rpm < 0) {
        return 0;
    }
    if (target_rpm > TASK34_TARGET_MAX_RPM) {
        return TASK34_TARGET_MAX_RPM;
    }
    return target_rpm;
}

static void Task34_ResetFilter(void)
{
    uint8_t i;

    task34_delta_sum = 0;
    task34_filter_count = 0U;
    task34_filter_index = 0U;
    for (i = 0U; i < TASK34_FILTER_SAMPLES; i++) {
        task34_delta_history[i] = 0;
    }
}

static void Task34_UpdateController(void)
{
    float error = (float)task34_target_rpm - (float)task34_actual_rpm;
    float delta_error = error - task34_previous_error;
    float second_difference = error -
                              (2.0f * task34_previous_error) +
                              task34_previous_previous_error;
    float delta_output = (TASK34_PID_KP * delta_error) +
                         (TASK34_PID_KI * error * TASK34_CONTROL_PERIOD_S) +
                         (TASK34_PID_KD * second_difference /
                          TASK34_CONTROL_PERIOD_S);
    float next_output;

    /* 限制每 10 ms 的 PWM 变化，降低电机惯性导致的过冲和来回拉扯。 */
    if (delta_output > TASK34_PWM_STEP_LIMIT) {
        delta_output = TASK34_PWM_STEP_LIMIT;
    } else if (delta_output < -TASK34_PWM_STEP_LIMIT) {
        delta_output = -TASK34_PWM_STEP_LIMIT;
    }
    next_output = task34_output_accumulator + delta_output;

    /* 增量式抗积分饱和：饱和且仍向饱和方向推动时，不累加该次增量。 */
    if (((next_output > TASK34_PWM_LIMIT) && (delta_output > 0.0f)) ||
        ((next_output < 0.0f) && (delta_output < 0.0f))) {
        next_output = task34_output_accumulator;
    }

    if (next_output > TASK34_PWM_LIMIT) {
        next_output = TASK34_PWM_LIMIT;
    } else if (next_output < 0.0f) {
        next_output = 0.0f;
    }

    task34_previous_previous_error = task34_previous_error;
    task34_previous_error = error;
    task34_output_accumulator = next_output;
    task34_output = (int16_t)(next_output + 0.5f);
    MOTOR_B_SetSpeed(task34_output);
}

void Task34_PID_VOFA_Init(void)
{
    task34_active = 0U;
    task34_target_rpm = 0;
    task34_actual_rpm = 0;
    task34_output = 0;
    task34_output_accumulator = 0.0f;
    task34_previous_error = 0.0f;
    task34_previous_previous_error = 0.0f;
    Task34_ResetFilter();

    /* 复用任务33已验证的 TIM6 10 ms 编码器采样节拍，启动时电机保持停止。 */
    Task33_Encoder_PeriodicInitWithPwm(0);
    task34_start_tick = HAL_GetTick();
    task34_last_report_tick = task34_start_tick;
    task34_step_pending = 1U;
    task34_active = 1U;
}

void Task34_PID_SetTargetRPM(int16_t target_rpm)
{
    task34_target_rpm = Task34_LimitTarget(target_rpm);
    task34_step_pending = 0U;

    /*
     * 用当前电机实测的目标转速/PWM 比作起始前馈，
     * 让 PID 从接近工作点开始修正，避免积分从零慢慢爬升。
     */
    task34_output_accumulator = (float)task34_target_rpm *
                                TASK34_FEEDFORWARD_PWM_PER_RPM;
    if (task34_output_accumulator > TASK34_PWM_LIMIT) {
        task34_output_accumulator = TASK34_PWM_LIMIT;
    }
    task34_output = (int16_t)(task34_output_accumulator + 0.5f);
    /* Track the current error at the setpoint change to avoid a proportional kick. */
    task34_previous_error = (float)task34_target_rpm - (float)task34_actual_rpm;
    task34_previous_previous_error = task34_previous_error;
    if (task34_active != 0U) {
        MOTOR_B_SetSpeed(task34_output);
    }
}

void Task34_PID_EncoderSample10ms(int16_t encoder_delta)
{
    int32_t absolute_delta;
    int32_t denominator;

    if (task34_active == 0U) {
        return;
    }

    /* 记录中断中取得的编码器计数，5 点滑动平均抑制 10 ms 量化跳动。 */
    if (task34_filter_count >= TASK34_FILTER_SAMPLES) {
        task34_delta_sum -= task34_delta_history[task34_filter_index];
    } else {
        task34_filter_count++;
    }
    task34_delta_history[task34_filter_index] = encoder_delta;
    task34_delta_sum += encoder_delta;
    task34_filter_index++;
    if (task34_filter_index >= TASK34_FILTER_SAMPLES) {
        task34_filter_index = 0U;
    }

    /* Use the empirically calibrated effective counts/revolution; report speed magnitude. */
    absolute_delta = (task34_delta_sum < 0) ? -task34_delta_sum : task34_delta_sum;
    denominator = TASK34_COUNTS_PER_OUTPUT_REV * (int32_t)task34_filter_count;
    if (denominator > 0) {
        task34_actual_rpm = (absolute_delta * 6000L + (denominator / 2L)) /
                            denominator;
    } else {
        task34_actual_rpm = 0;
    }

    if (task34_target_rpm == 0) {
        task34_output = 0;
        task34_output_accumulator = 0.0f;
        task34_previous_error = 0.0f;
        task34_previous_previous_error = 0.0f;
        MOTOR_B_SetSpeed(0);
        return;
    }

    Task34_UpdateController();
}

void Task34_PID_VOFA_PeriodicTask(void)
{
    uint32_t now = HAL_GetTick();

    /* 自动做一次 0 -> 120 RPM 阶跃，便于直接验收闭环与曲线。 */
    if ((task34_step_pending != 0U) &&
        ((now - task34_start_tick) >= TASK34_STARTUP_STEP_DELAY_MS)) {
        Task34_PID_SetTargetRPM(TASK34_DEFAULT_TARGET_RPM);
    }

    if ((now - task34_last_report_tick) >= TASK34_REPORT_PERIOD_MS) {
        int16_t target_snapshot = task34_target_rpm;
        int32_t actual_snapshot = task34_actual_rpm;
        int16_t pwm_snapshot = task34_output;
        char frame[32];
        int length = snprintf(frame, sizeof(frame), "%d,%ld,%d\r\n",
                              (int)target_snapshot, (long)actual_snapshot,
                              (int)pwm_snapshot);

        task34_last_report_tick = now;
        if ((length > 0) && ((size_t)length < sizeof(frame))) {
            (void)HAL_UART_Transmit(&huart1, (uint8_t *)frame,
                                    (uint16_t)length, 20U);
        }
    }
}

int16_t Task34_PID_GetOutput(void)
{
    return task34_output;
}

int32_t Task34_PID_GetActualRPM(void)
{
    return task34_actual_rpm;
}
