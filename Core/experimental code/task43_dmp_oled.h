#ifndef TASK43_DMP_OLED_H
#define TASK43_DMP_OLED_H

#include "stm32f1xx_hal.h"

/* 实验4任务3：DMP 姿态角读取并显示 Pitch/Roll/Yaw。 */
#ifndef TASK43_YAW_CALIBRATION_MS
#define TASK43_YAW_CALIBRATION_MS 12000U
#endif

HAL_StatusTypeDef Task43_DMP_OLED_Init(void);
void Task43_DMP_OLED_PeriodicTask(void);

#endif /* TASK43_DMP_OLED_H */
