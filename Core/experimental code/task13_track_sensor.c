/**
 * @file    task13_track_sensor.c
 * @brief   实验1 任务3：LF04 4路循迹模块驱动
 * @details 读取 4 路巡线传感器并在 OLED 上显示
 * @note    黑色=0，白色=1
 */

#include "task13_track_sensor.h"
#include "track.h"
#include "oled.h"
#include "stm32f1xx_hal.h"

/**
 * @brief  任务1.3：巡线传感器测试
 * @param  None
 * @retval None
 * @note   在 OLED 上实时显示 4 路巡线传感器状态
 */
void Task13_Track_SensorTest(void)
{
  TRACK_StateTypeDef track_state = {0};

  TRACK_Init();
  OLED_Init();
  OLED_Clear();
  OLED_ShowString(0, 0, "Track:");

  while (1) {
    TRACK_Read(&track_state);
    OLED_ShowChar(12, 0, (char)('0' + track_state.p1));
    OLED_ShowChar(18, 0, (char)('0' + track_state.p2));
    OLED_ShowChar(24, 0, (char)('0' + track_state.p3));
    OLED_ShowChar(30, 0, (char)('0' + track_state.p4));
    HAL_Delay(20);
  }
}
