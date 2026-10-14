#include "line_follow.h"

#include "motor.h"
#include "track.h"

#if LINE_FOLLOW_DEBUG
#include <stdio.h>
#endif

typedef struct {
    float kp;
    float ki;
    float kd;
} LineFollow_PIDTypeDef;

static LineFollow_PIDTypeDef line_follow_pid = {
    LINE_FOLLOW_KP,
    LINE_FOLLOW_KI,
    LINE_FOLLOW_KD
};

static TRACK_StateTypeDef line_follow_state;
static float line_follow_error;
static float line_follow_previous_error;
static float line_follow_integral;
static int16_t line_follow_base_speed = LINE_FOLLOW_BASE_SPEED;
static int16_t line_follow_left_speed;
static int16_t line_follow_right_speed;
static uint32_t line_follow_last_tick;
#if LINE_FOLLOW_DEBUG
static uint32_t line_follow_last_report_tick;
#endif
static uint8_t line_follow_initialized;

static int16_t LineFollow_LimitSpeed(int32_t speed)
{
    if (speed > LINE_FOLLOW_MAX_SPEED) {
        return LINE_FOLLOW_MAX_SPEED;
    }
    if (speed < -LINE_FOLLOW_MAX_SPEED) {
        return -LINE_FOLLOW_MAX_SPEED;
    }
    return (int16_t)speed;
}

static void LineFollow_ResetController(void)
{
    line_follow_error = 0.0f;
    line_follow_previous_error = 0.0f;
    line_follow_integral = 0.0f;
    line_follow_left_speed = 0;
    line_follow_right_speed = 0;
}

void LineFollow_Init(void)
{
    TRACK_Init();
    MOTOR_Init();
    MOTOR_SetEnabled(1U);
    MOTOR_SetSpeeds(0, 0);

    LineFollow_ResetController();
    line_follow_last_tick = HAL_GetTick() - LINE_FOLLOW_PERIOD_MS;
#if LINE_FOLLOW_DEBUG
    line_follow_last_report_tick = HAL_GetTick();
#endif
    line_follow_initialized = 1U;
}

void LineFollow_Stop(void)
{
    MOTOR_AllBrake();
    MOTOR_SetEnabled(0U);
    line_follow_left_speed = 0;
    line_follow_right_speed = 0;
    line_follow_initialized = 0U;
}

void LineFollow_SetBaseSpeed(int16_t speed)
{
    if (speed < 0) {
        speed = 0;
    }
    if (speed > LINE_FOLLOW_MAX_SPEED) {
        speed = LINE_FOLLOW_MAX_SPEED;
    }
    line_follow_base_speed = speed;
}

void LineFollow_SetPid(float kp, float ki, float kd)
{
    line_follow_pid.kp = kp;
    line_follow_pid.ki = ki;
    line_follow_pid.kd = kd;
    line_follow_integral = 0.0f;
    line_follow_previous_error = line_follow_error;
}

void LineFollow_PeriodicTask(void)
{
    uint32_t now;
    uint8_t black_count;
    int32_t weighted_sum;
    float correction;
    int32_t left_speed;
    int32_t right_speed;

    if (line_follow_initialized == 0U) {
        return;
    }

    now = HAL_GetTick();
    if ((now - line_follow_last_tick) < LINE_FOLLOW_PERIOD_MS) {
        return;
    }
    line_follow_last_tick = now;

    TRACK_Read(&line_follow_state);
    black_count = (uint8_t)((line_follow_state.p1 == 0U) +
                            (line_follow_state.p2 == 0U) +
                            (line_follow_state.p3 == 0U) +
                            (line_follow_state.p4 == 0U));

    /* All white means the line is lost: stop safely and retain last error. */
    if (black_count == 0U) {
        line_follow_integral = 0.0f;
        line_follow_left_speed = 0;
        line_follow_right_speed = 0;
        MOTOR_SetSpeeds(0, 0);
        return;
    }

    /* All black is treated as a wide/marked section of the fixed track. */
    if (black_count == 4U) {
        line_follow_integral = 0.0f;
        line_follow_left_speed = LINE_FOLLOW_ALL_BLACK_SPEED;
        line_follow_right_speed = LINE_FOLLOW_ALL_BLACK_SPEED;
        MOTOR_SetSpeeds(line_follow_left_speed, line_follow_right_speed);
        return;
    }

    /* P1..P4 are left-to-right, with black-line weights -3,-1,+1,+3. */
    weighted_sum = 0;
    if (line_follow_state.p1 == 0U) {
        weighted_sum -= 3;
    }
    if (line_follow_state.p2 == 0U) {
        weighted_sum -= 1;
    }
    if (line_follow_state.p3 == 0U) {
        weighted_sum += 1;
    }
    if (line_follow_state.p4 == 0U) {
        weighted_sum += 3;
    }

    line_follow_error = (float)weighted_sum / (float)black_count;
    line_follow_integral += line_follow_error * ((float)LINE_FOLLOW_PERIOD_MS / 1000.0f);
    if (line_follow_integral > LINE_FOLLOW_INTEGRAL_LIMIT) {
        line_follow_integral = LINE_FOLLOW_INTEGRAL_LIMIT;
    } else if (line_follow_integral < -LINE_FOLLOW_INTEGRAL_LIMIT) {
        line_follow_integral = -LINE_FOLLOW_INTEGRAL_LIMIT;
    }

    correction = (line_follow_pid.kp * line_follow_error) +
                 (line_follow_pid.ki * line_follow_integral) +
                 (line_follow_pid.kd * (line_follow_error - line_follow_previous_error));
    line_follow_previous_error = line_follow_error;

    left_speed = (int32_t)((float)line_follow_base_speed + correction);
    right_speed = (int32_t)((float)line_follow_base_speed - correction);
    line_follow_left_speed = LineFollow_LimitSpeed(left_speed);
    line_follow_right_speed = LineFollow_LimitSpeed(right_speed);
    MOTOR_SetSpeeds(line_follow_left_speed, line_follow_right_speed);

#if LINE_FOLLOW_DEBUG
    if ((now - line_follow_last_report_tick) >= 100U) {
        int32_t error_x100 = (int32_t)(line_follow_error * 100.0f);
        printf("P=%u%u%u%u,error_x100=%ld,left=%d,right=%d\r\n",
               line_follow_state.p1, line_follow_state.p2,
               line_follow_state.p3, line_follow_state.p4,
               (long)error_x100, (int)line_follow_left_speed,
               (int)line_follow_right_speed);
        line_follow_last_report_tick = now;
    }
#endif
}
