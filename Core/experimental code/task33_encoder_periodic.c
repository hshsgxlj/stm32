/**
 * @file    task33_encoder_periodic.c
 * @brief   实验3任务3：TIM6 周期中断读取 C10B 电机 B 编码器并换算 RPM
 * @note    编码器由 TIM8 进行四倍频计数；TIM6 仅提供 10 ms 采样节拍。
 */

#include "task33_encoder_periodic.h"

#include "main.h"
#include "motor.h"
#include "task34_pid_vofa.h"

#include <stdio.h>

#define TASK33_REPORT_SAMPLE_COUNT    10U
#define TASK33_DEFAULT_MOTOR_PWM      200

/* Empirical effective scale from the observed wheel speed: about 62400 counts/output revolution. */
#define TASK33_DEFAULT_COUNTS_PER_OUTPUT_REV  62400U

TIM_HandleTypeDef htim6;

static volatile int32_t task33_window_delta;
static volatile uint8_t task33_sample_count;
static volatile int32_t task33_report_delta;
static volatile int32_t task33_speed_per_second;
static volatile int32_t task33_rpm;
static volatile uint8_t task33_report_ready;
static volatile uint32_t task33_counts_per_output_rev =
    TASK33_DEFAULT_COUNTS_PER_OUTPUT_REV;

static HAL_StatusTypeDef Task33_TIM6_Init(void)
{
    uint32_t timer_clock = HAL_RCC_GetPCLK1Freq();

    /* APB1 prescaler > 1 时，STM32F1 定时器时钟为 PCLK1 的 2 倍。 */
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) {
        timer_clock *= 2U;
    }
    if ((timer_clock < 10000U) || ((timer_clock % 10000U) != 0U)) {
        return HAL_ERROR;
    }

    htim6.Instance = TIM6;
    htim6.Init.Prescaler = (timer_clock / 10000U) - 1U;
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = 99U; /* 10 kHz counter tick / 100 = 10 ms */
    htim6.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&htim6) != HAL_OK) {
        return HAL_ERROR;
    }
    return HAL_TIM_Base_Start_IT(&htim6);
}

void Task33_Encoder_PeriodicInit(void)
{
    Task33_Encoder_PeriodicInitWithPwm(TASK33_DEFAULT_MOTOR_PWM);
}

void Task33_Encoder_PeriodicInitWithPwm(int16_t initial_pwm)
{
    task33_window_delta = 0;
    task33_sample_count = 0U;
    task33_report_delta = 0;
    task33_speed_per_second = 0;
    task33_rpm = 0;
    task33_report_ready = 0U;
    task33_counts_per_output_rev = TASK33_DEFAULT_COUNTS_PER_OUTPUT_REV;

    MOTOR_Init();
    MOTOR_SetEnabled(1U);
    MOTOR_ResetEncoders();
    MOTOR_B_SetSpeed(initial_pwm);

    if (Task33_TIM6_Init() != HAL_OK) {
        Error_Handler();
    }

}

void Task33_Encoder_SetMotorPwm(int16_t pwm)
{
    MOTOR_B_SetSpeed(pwm);
}

void Task33_Encoder_SetCountsPerOutputRevolution(uint32_t counts)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    task33_counts_per_output_rev = counts;
    __set_PRIMASK(primask);
}

void Task33_Encoder_PeriodicTask(void)
{
    int32_t delta;
    int32_t rpm;
    uint8_t report_ready;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    report_ready = task33_report_ready;
    if (report_ready != 0U) {
        delta = task33_report_delta;
        rpm = task33_rpm;
        task33_report_ready = 0U;
    }
    __set_PRIMASK(primask);

    if (report_ready == 0U) {
        return;
    }

    /* FireWater: 任意文本前缀:数值0,数值1\r\n。VOFA+ 显示为 I0=DeltaCNT、I1=RPM。 */
    printf("DeltaCNT:%ld,%ld\r\n", (long)delta, (long)rpm);
}

int32_t Task33_Encoder_GetDelta(void)
{
    return task33_report_delta;
}

int32_t Task33_Encoder_GetSpeedPerSecond(void)
{
    return task33_speed_per_second;
}

int32_t Task33_Encoder_GetRPM(void)
{
    return task33_rpm;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if ((htim == NULL) || (htim->Instance != TIM6)) {
        return;
    }

    /* 中断内只采集/计算，不做 printf，避免阻塞 10 ms 采样节拍。 */
    {
        int16_t encoder_delta = MOTOR_B_ReadEncoderDelta();

        Task34_PID_EncoderSample10ms(encoder_delta);
        task33_window_delta += encoder_delta;
    }
    task33_sample_count++;

    if (task33_sample_count >= TASK33_REPORT_SAMPLE_COUNT) {
        uint32_t counts_per_rev = task33_counts_per_output_rev;

        task33_report_delta = task33_window_delta;
        task33_speed_per_second = task33_window_delta * 10L;
        task33_rpm = (counts_per_rev != 0U)
                   ? ((task33_window_delta * 600L) / (int32_t)counts_per_rev)
                   : 0;
        task33_window_delta = 0;
        task33_sample_count = 0U;
        task33_report_ready = 1U;
    }
}
