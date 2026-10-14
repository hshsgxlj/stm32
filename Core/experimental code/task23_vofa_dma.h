#ifndef TASK23_VOFA_DMA_H
#define TASK23_VOFA_DMA_H

#include <stdint.h>

/* 实验2任务3：USART1 DMA + IDLE + VOFA+ JustFloat 波形。
 * 目标值/实际值单位为编码器 counts/s，PWM 为 -1000~1000。 */
void Task23_VOFA_Init(void);
void Task23_VOFA_PeriodicTask(void);
void Task23_VOFA_SetTarget(float target);
void Task23_VOFA_SetPwm(int16_t pwm);

/* 由 USART1_IRQHandler 调用；只做短暂的 DMA 位置记录和数据搬运。 */
void Task23_VOFA_USART1_IdleCallback(void);

#endif /* TASK23_VOFA_DMA_H */
