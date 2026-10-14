/**
  ******************************************************************************
  * @file           : led.c
  * @brief          : LED控制功能实现
  ******************************************************************************
  */

#include "led.h"

/**
  * @brief  LED GPIO初始化
  * @param  None
  * @retval None
  */
void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* 使能GPIO时钟 */
    LED1_GPIO_CLK_ENABLE();
    LED2_GPIO_CLK_ENABLE();

    /* 初始化时关闭所有LED */
    LED1_OFF();
    LED2_OFF();
}

/**
  * @brief  打开指定LED
  * @param  led_num: LED编号 (1, 2)
  * @retval None
  */
void LED_On(uint8_t led_num)
{
    switch(led_num)
    {
        case 1:
            LED1_ON();
            break;
        case 2:
            LED2_ON();
            break;
        default:
            break;
    }
}

/**
  * @brief  关闭指定LED
  * @param  led_num: LED编号 (1, 2)
  * @retval None
  */
void LED_Off(uint8_t led_num)
{
    switch(led_num)
    {
        case 1:
            LED1_OFF();
            break;
        case 2:
            LED2_OFF();
            break;
        default:
            break;
    }
}

/**
  * @brief  翻转指定LED状态
  * @param  led_num: LED编号 (1, 2)
  * @retval None
  */
void LED_Toggle(uint8_t led_num)
{
    switch(led_num)
    {
        case 1:
            LED1_TOGGLE();
            break;
        case 2:
            LED2_TOGGLE();
            break;
        default:
            break;
    }
}

/**
  * @brief  打开所有LED
  * @param  None
  * @retval None
  */
void LED_All_On(void)
{
    LED1_ON();
    LED2_ON();
}

/**
  * @brief  关闭所有LED
  * @param  None
  * @retval None
  */
void LED_All_Off(void)
{
    LED1_OFF();
    LED2_OFF();
}

/**
  * @brief  LED闪烁
  * @param  led_num: LED编号 (1, 2, 0表示所有LED)
  * @param  times: 闪烁次数
  * @param  delay_ms: 每次闪烁间隔时间(毫秒)
  * @retval None
  */
void LED_Blink(uint8_t led_num, uint8_t times, uint16_t delay_ms)
{
    for(uint8_t i = 0; i < times; i++)
    {
        if(led_num == 0)
        {
            LED_All_On();
        }
        else
        {
            LED_On(led_num);
        }
        HAL_Delay(delay_ms);

        if(led_num == 0)
        {
            LED_All_Off();
        }
        else
        {
            LED_Off(led_num);
        }
        HAL_Delay(delay_ms);
    }
}

/**
  * @brief  LED流水灯效果
  * @param  delay_ms: 每个LED点亮间隔时间(毫秒)
  * @param  cycles: 循环次数
  * @retval None
  */
void LED_RunningLight(uint16_t delay_ms, uint8_t cycles)
{
    for(uint8_t i = 0; i < cycles; i++)
    {
        LED1_ON();
        HAL_Delay(delay_ms);
        LED1_OFF();

        LED2_ON();
        HAL_Delay(delay_ms);
        LED2_OFF();
    }
}

/**
  * @brief  LED呼吸灯效果（需要PWM支持，这里用延时模拟）
  * @param  led_num: LED编号 (1, 2)
  * @param  cycles: 呼吸周期次数
  * @retval None
  */
void LED_Breath(uint8_t led_num, uint8_t cycles)
{
    for(uint8_t cycle = 0; cycle < cycles; cycle++)
    {
        // 逐渐变亮
        for(uint16_t brightness = 0; brightness < 100; brightness++)
        {
            LED_On(led_num);
            for(volatile uint32_t i = 0; i < brightness * 10; i++);
            LED_Off(led_num);
            for(volatile uint32_t i = 0; i < (100 - brightness) * 10; i++);
        }

        // 逐渐变暗
        for(uint16_t brightness = 100; brightness > 0; brightness--)
        {
            LED_On(led_num);
            for(volatile uint32_t i = 0; i < brightness * 10; i++);
            LED_Off(led_num);
            for(volatile uint32_t i = 0; i < (100 - brightness) * 10; i++);
        }
    }
}

/**
  * @brief  LED SOS信号
  * @param  None
  * @retval None
  */
void LED_SOS(void)
{
    // S: 三短
    for(uint8_t i = 0; i < 3; i++)
    {
        LED_All_On();
        HAL_Delay(200);
        LED_All_Off();
        HAL_Delay(200);
    }

    HAL_Delay(200);

    // O: 三长
    for(uint8_t i = 0; i < 3; i++)
    {
        LED_All_On();
        HAL_Delay(600);
        LED_All_Off();
        HAL_Delay(200);
    }

    HAL_Delay(200);

    // S: 三短
    for(uint8_t i = 0; i < 3; i++)
    {
        LED_All_On();
        HAL_Delay(200);
        LED_All_Off();
        HAL_Delay(200);
    }

    HAL_Delay(1000);
}
