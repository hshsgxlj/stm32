/**
 * @file    task22_encoder_printf.h
 * @brief   实验2 任务2：printf 重定向打印编码器值（必做）
 * @details 提供 printf 重定向和编码器周期打印功能
 */

#ifndef __TASK22_ENCODER_PRINTF_H
#define __TASK22_ENCODER_PRINTF_H

#include <stdint.h>
#include <stdio.h>

/**
 * @brief  重写 fputc 实现 printf 重定向到 USART1
 * @param  ch: 要发送的字符
 * @param  f: 文件指针
 * @retval 发送的字符
 */
int fputc(int ch, FILE *f);

/**
 * @brief  设置目标速度
 * @param  speed: 目标速度值
 * @retval None
 */
void Task22_SetTargetSpeed(int16_t speed);

/**
 * @brief  初始化编码器打印模块
 * @param  None
 * @retval None
 */
void Task22_Init(void);

/**
 * @brief  周期打印编码器值（主循环调用）
 * @param  None
 * @retval None
 * @note   每 100 ms 自动打印一次
 */
void Task22_PeriodicTask(void);

/**
 * @brief  完整测试函数（可选）
 * @param  None
 * @retval None
 * @note   启动电机并持续打印，用于独立测试
 */
void Task22_EncoderPrintfTest(void);

#endif /* __TASK22_ENCODER_PRINTF_H */
