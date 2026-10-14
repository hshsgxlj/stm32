/**
 * @file    task31_tim2_pwm.c
 * @brief   任务3.1：CubeMX 配置 TIM2 CH1 输出 PWM（18 kHz）
 *
 * 使用说明：
 * 1. TIM2 由 main.c 的 MX_TIM2_Init() 配置：Period=3999，PSC=0
 *    定时器时钟 72 MHz（APB1 36 MHz ×2）→ 72M / 4000 = 18 kHz
 * 2. 本任务调用 HAL_TIM_PWM_Start() 启动输出，
 *    用 Task31_Tim2PWM_SetDuty() 实时改变占空比（0% → 100% → 0% 斜坡）
 * 3. 验收方式（二选一）：
 *    - 示波器：探针接 PA0（USER 排针上，GND 接板子 GND），
 *      看到 18 kHz 方波，占空比周期性变化
 *    - VOFA+：115200、协议选 FireWater，接收 duty 和 compare 两条曲线
 *
 * 注意：PA0 原是 USER 按键引脚，现在被 TIM2_CH1 占用，
 *       按键中断已停用；测试时不要按按键（会把波形拉偏）。
 */

#include "task31_tim2_pwm.h"

#include <stdio.h>

extern TIM_HandleTypeDef htim2;

/* CubeMX config: PSC=0, ARR=3999, TIM2 clock=72 MHz => 18 kHz. */
#define TASK31_TIM2_ARR      (__HAL_TIM_GET_AUTORELOAD(&htim2))
#define TASK31_DUTY_STEP     2U
#define TASK31_SAMPLE_MS     50U

void Task31_Tim2PWM_SetDuty(uint8_t duty_percent)
{
    uint32_t compare;

    if (duty_percent > 100U) {
        duty_percent = 100U;
    }

    compare = ((TASK31_TIM2_ARR + 1UL) * duty_percent) / 100UL;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, compare);
}

/**
 * @brief  任务3.1 完整测试：启动 TIM2 CH1 PWM 并斜坡改变占空比
 * @param  None
 * @retval None
 */
void Task31_Tim2PWM_Test(void)
{
    int16_t duty_percent = 0;
    int16_t step = (int16_t)TASK31_DUTY_STEP;
    uint32_t compare;
    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }

    printf("\r\n=== Task 3.1: TIM2 CH1 PWM (PA0, 18 kHz) ===\r\n");
    printf("Duty ramps 0%% -> 100%% -> 0%%\r\n");
    printf("Oscilloscope: probe PA0 (USER header pin)\r\n");
    printf("VOFA+: FireWater engine, 115200 baud\r\n\r\n");

    while (1) {
        Task31_Tim2PWM_SetDuty((uint8_t)duty_percent);
        compare = ((TASK31_TIM2_ARR + 1UL) * (uint16_t)duty_percent) / 100UL;

        /* FireWater 简单格式：每行两个逗号分隔的数值，以换行结束。 */
        printf("%u,%lu\r\n",
               (unsigned int)duty_percent, (unsigned long)compare);

        /* 斜坡：到顶/到底后反向 */
        if (duty_percent + step >= 100) {
            duty_percent = 100;
            step = -step;
        } else if (duty_percent + step <= 0) {
            duty_percent = 0;
            step = -step;
        } else {
            duty_percent += step;
        }

        HAL_Delay(TASK31_SAMPLE_MS);
    }
}
