#ifndef TASK34_PID_VOFA_H
#define TASK34_PID_VOFA_H

#include <stdint.h>

/* 实验3任务4：10 ms 增量式速度 PID + VOFA+ FireWater 双通道。 */
void Task34_PID_VOFA_Init(void);
void Task34_PID_VOFA_PeriodicTask(void);
void Task34_PID_SetTargetRPM(int16_t target_rpm);
int16_t Task34_PID_GetOutput(void);
int32_t Task34_PID_GetActualRPM(void);

/* 由任务33的 TIM6 中断每 10 ms 调用，不在中断中发送串口数据。 */
void Task34_PID_EncoderSample10ms(int16_t encoder_delta);

#endif /* TASK34_PID_VOFA_H */
