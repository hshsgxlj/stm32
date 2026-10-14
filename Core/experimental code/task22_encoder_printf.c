/**
 * @file    task22_encoder_printf.c
 * @brief   实验2 任务2：printf 重定向打印编码器值（必做）
 * @details 实现 fputc 重定向到 USART1，周期读取编码器速度并打印
 * @note    每 100 ms 打印一行编码器速度与目标值
 */

#include "task22_encoder_printf.h"
#include "motor.h"
#include "stm32f1xx_hal.h"
#include <stdio.h>

extern UART_HandleTypeDef huart1;

/* 目标速度值（可根据实际情况调整） */
static int16_t target_speed = 600;

/* 上次打印时间戳 */
static uint32_t last_print_time = 0;

/**
 * @brief  重写 fputc 实现 printf 重定向到 USART1
 * @param  ch: 要发送的字符
 * @param  f: 文件指针（通常为 stdout）
 * @retval 发送的字符
 * @note   标准库的 printf 会调用此函数逐字符输出
 */
int fputc(int ch, FILE *f)
{
  uint8_t data = (uint8_t)ch;
  HAL_UART_Transmit(&huart1, &data, 1, HAL_MAX_DELAY);
  return ch;
}

/**
 * @brief  设置目标速度
 * @param  speed: 目标速度值
 * @retval None
 */
void Task22_SetTargetSpeed(int16_t speed)
{
  target_speed = speed;
}

/**
 * @brief  初始化编码器打印模块
 * @param  None
 * @retval None
 * @note   记录初始时间戳
 */
void Task22_Init(void)
{
  last_print_time = HAL_GetTick();
  printf("\r\n=== Task 2.2: Encoder Printf Test ===\r\n");
  printf("Target Speed: %d\r\n", target_speed);
  printf("Print Interval: 100 ms\r\n");
  printf("Format: [time_ms] speed=XXX, target=XXX\r\n\r\n");
}

/**
 * @brief  周期打印编码器值（主循环调用）
 * @param  None
 * @retval None
 * @note   每 100 ms 读取 B 电机编码器增量并打印
 */
void Task22_PeriodicTask(void)
{
  uint32_t current_time = HAL_GetTick();

  /* 每 100 ms 执行一次 */
  if ((current_time - last_print_time) >= 100)
  {
    last_print_time = current_time;

    /* 读取 B 电机编码器增量（CN2 接口） */
    int16_t encoder_delta = MOTOR_B_ReadEncoderDelta();

    /* 计算速度：delta 是 100ms 内的脉冲数，乘以 10 得到每秒脉冲数 */
    int16_t speed = encoder_delta * 10;

    /* 打印格式：[时间戳] speed=实际速度, target=目标速度 */
    printf("[%lu] speed=%d, target=%d\r\n", current_time, speed, target_speed);
  }
}

/**
 * @brief  完整测试函数（可选）
 * @param  None
 * @retval None
 * @note   启动电机并持续打印编码器值，用于独立测试
 */
void Task22_EncoderPrintfTest(void)
{
  Task22_Init();

  /* 设置 B 电机转速为目标值（测试时车轮需悬空） */
  MOTOR_B_SetSpeed(target_speed);

  printf("Motor B started with speed: %d\r\n\r\n", target_speed);

  /* 持续打印 */
  while (1)
  {
    Task22_PeriodicTask();
    HAL_Delay(1); /* 短暂延时避免 CPU 占用过高 */
  }
}
