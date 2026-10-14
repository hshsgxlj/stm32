#ifndef TASK31_PWM_ENCODER_H
#define TASK31_PWM_ENCODER_H

#include <stdint.h>
#include "stm32f1xx_hal.h"

/**
 * @brief  任务3.1/3.2：初始化 PWM 和编码器
 * @param  None
 * @retval None
 */
void Task31_PWM_Encoder_Init(void);

/**
 * @brief  设置电机速度（PWM占空比）
 * @param  speed: -1000 ~ +1000
 * @retval None
 */
void Task31_PWM_Encoder_SetSpeed(int16_t speed);

/**
 * @brief  读取编码器增量
 * @retval 编码器脉冲增量
 */
int16_t Task31_PWM_Encoder_ReadDelta(void);

/**
 * @brief  任务3.1/3.2：完整测试
 * @param  None
 * @retval None
 */
void Task31_PWM_Encoder_Test(void);

#endif /* TASK31_PWM_ENCODER_H */
