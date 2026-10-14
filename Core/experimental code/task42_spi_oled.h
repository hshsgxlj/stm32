#ifndef TASK42_SPI_OLED_H
#define TASK42_SPI_OLED_H

#include <stdint.h>

// 实验4任务2：通过固定接口软件 SPI 显示 OLED 字符和整数。
void Task42_SPI_OLED_Init(void);
void Task42_SPI_OLED_PeriodicTask(void);
void Task42_SPI_OLED_ShowText(const char *text);
void Task42_SPI_OLED_ShowNumber(int16_t value);

#endif /* TASK42_SPI_OLED_H */
