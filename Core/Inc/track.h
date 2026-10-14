#ifndef __TRACK_H
#define __TRACK_H

#include "stm32f1xx_hal.h"

/*
 * New C10B tracking connector wiring, in the photographed order:
 * G, 3V3, PB8, PC9, PC4, PC8.
 * The user confirmed that the last signal (PC8) is P1, so the physical
 * sensor order is P1=PC8, P2=PC4, P3=PC9, P4=PB8.
 * Black=0, white=1.
 */
#define TRACK_P1_PORT GPIOC
#define TRACK_P1_PIN  GPIO_PIN_8
#define TRACK_P2_PORT GPIOC
#define TRACK_P2_PIN  GPIO_PIN_4
#define TRACK_P3_PORT GPIOC
#define TRACK_P3_PIN  GPIO_PIN_9
#define TRACK_P4_PORT GPIOB
#define TRACK_P4_PIN  GPIO_PIN_8

typedef struct {
    uint8_t p1;
    uint8_t p2;
    uint8_t p3;
    uint8_t p4;
} TRACK_StateTypeDef;

void TRACK_Init(void);
void TRACK_Read(TRACK_StateTypeDef *state);

#endif /* __TRACK_H */
