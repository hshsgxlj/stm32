#ifndef BADAPPLE_H
#define BADAPPLE_H

#include "stm32f1xx_hal.h"

#define BADAPPLE_FRAME_WIDTH       128U
#define BADAPPLE_FRAME_HEIGHT      64U
#define BADAPPLE_FRAME_SIZE        1024U
#define BADAPPLE_UART_BAUD_RATE    230400U

/*
 * Call once in each main-loop iteration. Three quick USER-button presses
 * start playback. This is the only Bad Apple function used by the project.
 */
void BadApple_Start(volatile uint8_t *outputs_requested_on);

#endif /* BADAPPLE_H */
