/**
 * @file    task21_uart_echo.c
 * @brief   实验2 任务1：串口回环 Echo（基础，必做）
 * @details 接收串口助手发送的数据，并原样回传
 * @note    测试串口通信是否正常，波特率 115200
 */

#include "task21_uart_echo.h"
#include "stm32f1xx_hal.h"
extern UART_HandleTypeDef huart1;

/**
 * @brief  任务2.1：串口回环 Echo 测试
 * @param  None
 * @retval None
 * @note   阻塞接收 1 字节后立即回传，符合任务 2.1 的 Echo 要求
 */
void Task21_UART_EchoTest(void)
{
  uint8_t rx_data;

  while (1) {
    if (HAL_UART_Receive(&huart1, &rx_data, 1U, HAL_MAX_DELAY) == HAL_OK) {
      (void)HAL_UART_Transmit(&huart1, &rx_data, 1U, HAL_MAX_DELAY);
    }
  }
}
