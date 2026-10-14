#ifndef TASK33_ENCODER_PERIODIC_H
#define TASK33_ENCODER_PERIODIC_H

#include <stdint.h>

/* 实验3任务3：周期读取编码器，周期单位为毫秒。 */
void Task33_Encoder_PeriodicInit(void);
void Task33_Encoder_PeriodicInitWithPwm(int16_t initial_pwm);
void Task33_Encoder_PeriodicTask(void);
void Task33_Encoder_SetMotorPwm(int16_t pwm);
void Task33_Encoder_SetCountsPerOutputRevolution(uint32_t counts);
int32_t Task33_Encoder_GetDelta(void);
int32_t Task33_Encoder_GetSpeedPerSecond(void);
int32_t Task33_Encoder_GetRPM(void);

#endif /* TASK33_ENCODER_PERIODIC_H */
