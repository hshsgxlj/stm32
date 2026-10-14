/**
 * @file    task32_motor_encoder.c
 * @brief   实验3任务2：板载电机 PWM 驱动与编码器四倍频测速
 * @details C10B 使用板载 AT8236 驱动；CN2/B 电机由 TIM3 驱动，TIM8 读取编码器。
 */

#include "task32_motor_encoder.h"

#include "motor.h"
#include "stm32f1xx_hal.h"

#include <stdio.h>

#define TASK32_PRINT_PERIOD_MS  100U
#define TASK32_MOTOR_PWM        200
#define TASK32_PWM50_TEST_PWM   500
#define TASK32_PWM50_TEST_MS    5000U
#define TASK32_COUNTS_PER_OUTPUT_REV  62400L

static const char *Task32_GetDirection(int16_t encoder_delta)
{
    if (encoder_delta > 0) {
        return "Forward";
    }
    if (encoder_delta < 0) {
        return "Reverse";
    }
    return "Stop/No pulse";
}

/**
 * @brief 驱动 CN2/B 电机，并每 100 ms 打印编码器增量和方向。
 * @note 速度命令范围为 -1000~1000；默认 +200 约为 20% PWM。
 *       测试时将车轮悬空，并确保板子电机电源已接通。
 */
void Task32_Motor_EncoderTest(void)
{
    uint32_t last_sample_tick;

    MOTOR_Init();
    MOTOR_SetEnabled(1U);
    MOTOR_ResetEncoders();
    MOTOR_B_SetSpeed(TASK32_MOTOR_PWM);

    printf("\r\n=== Task 3.2: C10B Motor + 4x Encoder ===\r\n");
    printf("Motor: CN2 / B, PWM command: %d/1000\r\n", TASK32_MOTOR_PWM);
    printf("Encoder: PC6/PC7 via TIM8 encoder mode 3\r\n");
    printf("Format: DeltaCNT and direction every 100 ms\r\n\r\n");

    last_sample_tick = HAL_GetTick();
    while (1) {
        uint32_t now = HAL_GetTick();
        uint32_t elapsed_ms = now - last_sample_tick;

        if (elapsed_ms >= TASK32_PRINT_PERIOD_MS) {
            int16_t delta = MOTOR_B_ReadEncoderDelta();
            int32_t counts_per_second = ((int32_t)delta * 1000L) /
                                        (int32_t)elapsed_ms;

            printf("[%lu ms] DeltaCNT=%d, Direction=%s, Speed=%ld counts/s\r\n",
                   (unsigned long)now,
                   (int)delta,
                   Task32_GetDirection(delta),
                   (long)counts_per_second);
            last_sample_tick = now;
        }
        HAL_Delay(1U);
    }
}

/**
 * @brief B电机以固定50% PWM运行5秒，周期打印编码器读数后自动刹停。
 * @note 仅作供电/驱动检查，不是闭环速度控制；测试时车轮必须架空。
 */
void Task32_Motor_Pwm50Test(void)
{
    uint32_t start_tick;
    uint32_t last_report_tick;

    MOTOR_Init();
    MOTOR_SetEnabled(1U);
    MOTOR_ResetEncoders();
    MOTOR_B_SetSpeed(TASK32_PWM50_TEST_PWM);

    printf("\r\n=== B motor fixed PWM test: 50%% for 5 seconds ===\r\n");
    printf("Command=500/1000; motor speed is not necessarily 50%% of max.\r\n");
    start_tick = HAL_GetTick();
    last_report_tick = start_tick;

    while ((HAL_GetTick() - start_tick) < TASK32_PWM50_TEST_MS) {
        uint32_t now = HAL_GetTick();

        if ((now - last_report_tick) >= TASK32_PRINT_PERIOD_MS) {
            int16_t delta = MOTOR_B_ReadEncoderDelta();
            int32_t magnitude = (delta < 0) ? -(int32_t)delta : (int32_t)delta;
            int32_t rpm = (magnitude * 600L) / TASK32_COUNTS_PER_OUTPUT_REV;

            printf("PWM=500,DeltaCNT=%d,RPM=%ld\r\n", (int)delta, (long)rpm);
            last_report_tick = now;
        }
        HAL_Delay(1U);
    }

    MOTOR_B_Brake();
    MOTOR_SetEnabled(0U);
    printf("50%% PWM test complete; motor stopped.\r\n");

    while (1) {
        HAL_Delay(1000U);
    }
}
