#ifndef LINE_FOLLOW_H
#define LINE_FOLLOW_H

#include <stdint.h>

/* C10B four-sensor line following defaults. Values are in MOTOR_SPEED_MAX units. */
#define LINE_FOLLOW_PERIOD_MS          10U
#define LINE_FOLLOW_BASE_SPEED         220
#define LINE_FOLLOW_MAX_SPEED          450
#define LINE_FOLLOW_ALL_BLACK_SPEED    120

#define LINE_FOLLOW_KP                 70.0f
#define LINE_FOLLOW_KI                 0.8f
#define LINE_FOLLOW_KD                 25.0f
#define LINE_FOLLOW_INTEGRAL_LIMIT     10.0f

/* Set to 1 to print sensor/error/PWM data over USART1 every 100 ms. */
#ifndef LINE_FOLLOW_DEBUG
#define LINE_FOLLOW_DEBUG              0
#endif

void LineFollow_Init(void);
void LineFollow_PeriodicTask(void);
void LineFollow_Stop(void);
void LineFollow_SetBaseSpeed(int16_t speed);
void LineFollow_SetPid(float kp, float ki, float kd);

#endif /* LINE_FOLLOW_H */
