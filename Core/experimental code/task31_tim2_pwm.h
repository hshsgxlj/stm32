/**
 * @file    task31_tim2_pwm.h
 * @brief   任务3.1：CubeMX 配置 TIM2 CH1 输出 PWM（18 kHz）
 * @note    输出引脚 PA0/TIM2_CH1；占空比可通过 Task31_Tim2PWM_SetDuty() 实时设置
 */

#ifndef __TASK31_TIM2_PWM_H
#define __TASK31_TIM2_PWM_H

#include "main.h"

void Task31_Tim2PWM_Test(void);
void Task31_Tim2PWM_SetDuty(uint8_t duty_percent);

#endif /* __TASK31_TIM2_PWM_H */
