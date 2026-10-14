/**
 * @file    task11_led_blink.h
 * @brief   实验1 任务1：CubeMX + VSCode 工具链，LED 闪烁
 */

#ifndef __TASK11_LED_BLINK_H
#define __TASK11_LED_BLINK_H

/**
 * @brief  任务1.1：LED 闪烁测试
 * @param  None
 * @retval None
 */
void Task11_LED_BlinkTest(void);

/**
 * @brief  任务1.2：按键控制 LED 翻转（GPIO输入+消抖）
 * @param  None
 * @retval None
 */
void Task12_Button_ToggleLED(void);

/**
 * @brief  扩展：PA2 引脚电平控制 LED1
 * @param  None
 * @retval None
 */
void Task1X_PA2_ControlLED(void);

#endif /* __TASK11_LED_BLINK_H */
