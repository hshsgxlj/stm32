/**
  ******************************************************************************
  * @file           : led.h
  * @brief          : LED控制头文件
  ******************************************************************************
  */

#ifndef __LED_H
#define __LED_H

#ifdef __cplusplus
extern "C" {
#endif

/* 包含头文件 */
#include "stm32f1xx_hal.h"

/* LED引脚定义 - 根据原理图配置 */
#define LED1_PIN                    GPIO_PIN_2
#define LED1_GPIO_PORT              GPIOC
#define LED1_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOC_CLK_ENABLE()

#define LED2_PIN                    GPIO_PIN_3
#define LED2_GPIO_PORT              GPIOC
#define LED2_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOC_CLK_ENABLE()

/* LED控制宏定义 */
/* 共阳极连接: 低电平点亮，高电平熄灭 */
#define LED1_ON()       HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1_PIN, GPIO_PIN_RESET)
#define LED1_OFF()      HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1_PIN, GPIO_PIN_SET)
#define LED1_TOGGLE()   HAL_GPIO_TogglePin(LED1_GPIO_PORT, LED1_PIN)

#define LED2_ON()       HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2_PIN, GPIO_PIN_RESET)
#define LED2_OFF()      HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2_PIN, GPIO_PIN_SET)
#define LED2_TOGGLE()   HAL_GPIO_TogglePin(LED2_GPIO_PORT, LED2_PIN)


/* 函数声明 */
void LED_Init(void);                                           // LED初始化
void LED_On(uint8_t led_num);                                  // 打开指定LED
void LED_Off(uint8_t led_num);                                 // 关闭指定LED
void LED_Toggle(uint8_t led_num);                              // 翻转指定LED
void LED_All_On(void);                                         // 打开所有LED
void LED_All_Off(void);                                        // 关闭所有LED
void LED_Blink(uint8_t led_num, uint8_t times, uint16_t delay_ms);     // LED闪烁
void LED_RunningLight(uint16_t delay_ms, uint8_t cycles);      // 流水灯
void LED_Breath(uint8_t led_num, uint8_t cycles);              // 呼吸灯
void LED_SOS(void);                                            // SOS信号

#ifdef __cplusplus
}
#endif

#endif /* __LED_H */
