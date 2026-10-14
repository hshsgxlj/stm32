/**
 * @file    task11_led_blink.c
 * @brief   实验1 任务1：CubeMX + VSCode 工具链，LED 闪烁
 * @details 包含 LED 闪烁、按键控制等实验代码
 */

#include "task11_led_blink.h"
#include "led.h"
#include "stm32f1xx_hal.h"

/**
 * @brief  任务1.1：LED 闪烁测试
 * @param  None
 * @retval None
 * @note   LED1 闪烁 5 次，每次间隔 500ms
 */
void Task11_LED_BlinkTest(void)
{
  /* 示例: 单个LED闪烁 */
  LED_Blink(1, 50, 500);  // LED1闪烁5次，每次间隔500ms
}

/**
 * @brief  任务1.2：按键控制 LED 翻转（GPIO输入+消抖）
 * @param  None
 * @retval None
 * @note   按下 PA0 用户按键后，LED1 和 LED2 同时翻转
 */
void Task12_Button_ToggleLED(void)
{
  while (1) {
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) {
      HAL_Delay(20); // 消抖
      if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) {
        // 执行按键逻辑...
        LED_Toggle(1);
        LED_Toggle(2);
        while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) {
          HAL_Delay(10); // 等待按键释放
        }
      }
    }
  }
}

/**
 * @brief  扩展：PA2 引脚电平控制 LED1
 * @param  None
 * @retval None
 * @note   读取 PA2 电平：高电平点亮 LED1，低电平熄灭 LED1
 */
void Task1X_PA2_ControlLED(void)
{
  while (1) {
    // 1. 读取 PA2 引脚的电平状态
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_SET) {
      // 2. 如果读取到 1（高电平），点亮 LED1 (PC2 低电平点亮)
      LED1_ON();
    } else {
      // 3. 如果读取到 0（低电平），熄灭 LED1 (PC2 高电平熄灭)
      LED1_OFF();
    }

    // 稍微延时一下，防止循环跑太快占用CPU（可选）
    HAL_Delay(10);
  }
}
