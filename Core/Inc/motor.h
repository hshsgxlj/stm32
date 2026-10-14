#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f1xx_hal.h"

/*
 * C10B motor connections:
 * Motor A: PA6/TIM3_CH1, PA7/TIM3_CH2; encoder PB6/PB7 (TIM4).
 * Motor B: PB0/TIM3_CH3, PB1/TIM3_CH4; encoder PC6/PC7 (TIM8).
 * Speed range: -1000 (full reverse) to +1000 (full forward).
 */
#define MOTOR_SPEED_MAX 1000

/*
 * Direction signs: left/right motors are mounted mirrored, so with the
 * same electrical drive they spin in opposite physical directions.
 * Tune so that a positive speed moves BOTH wheels forwards:
 *  - wheels spin opposite -> negate ONE side;
 *  - wheels agree but the car drives backwards -> negate BOTH sides.
 */
#define MOTOR_A_DIRECTION_SIGN  1
#define MOTOR_B_DIRECTION_SIGN  -1

void MOTOR_Init(void);
void MOTOR_SetEnabled(uint8_t enabled);
uint8_t MOTOR_IsEnabled(void);

void MOTOR_A_SetSpeed(int16_t speed);
void MOTOR_B_SetSpeed(int16_t speed);
void MOTOR_SetSpeeds(int16_t motor_a, int16_t motor_b);

void MOTOR_A_Brake(void);
void MOTOR_B_Brake(void);
void MOTOR_AllBrake(void);
void MOTOR_A_Coast(void);
void MOTOR_B_Coast(void);
void MOTOR_AllCoast(void);

int16_t MOTOR_A_ReadEncoderDelta(void);
int16_t MOTOR_B_ReadEncoderDelta(void);
int32_t MOTOR_A_GetEncoderTotal(void);
int32_t MOTOR_B_GetEncoderTotal(void);
void MOTOR_ResetEncoders(void);

#endif /* __MOTOR_H */
