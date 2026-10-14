#ifndef __OLED_H
#define __OLED_H

#include "stm32f1xx_hal.h"

// The OLED connector is fixed: PC14=SCL and PB5=SDA.
#ifndef OLED_USE_SPI2
#define OLED_USE_SPI2 0U
#endif

// C10B OLED socket: GND, 3V3, SCL/SCK, SDA/MOSI, RES, DC.
#define OLED_SCL_PORT GPIOC
#define OLED_SCL_PIN  GPIO_PIN_14
#define OLED_SDA_PORT GPIOB
#define OLED_SDA_PIN  GPIO_PIN_5
#define OLED_RST_PORT GPIOB
#define OLED_RST_PIN  GPIO_PIN_4
#define OLED_DC_PORT  GPIOB
#define OLED_DC_PIN   GPIO_PIN_3

void OLED_Init(void);
void OLED_Fill(uint8_t data);
void OLED_Clear(void);
void OLED_DisplayOn(void);
void OLED_DisplayOff(void);
void OLED_SendCommand(uint8_t command);
void OLED_ShowChar(uint8_t x, uint8_t page, char character);
void OLED_ShowString(uint8_t x, uint8_t page, const char *text);
void OLED_ShowInt16(uint8_t x, uint8_t page, int16_t value);
void OLED_ShowAngle(uint8_t x, uint8_t page, int16_t degrees_x10);

#endif /* __OLED_H */
