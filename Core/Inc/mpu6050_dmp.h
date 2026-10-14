#ifndef __MPU6050_DMP_H
#define __MPU6050_DMP_H

#include "stm32f1xx_hal.h"

typedef struct {
    int16_t pitch_deg_x10;
    int16_t roll_deg_x10;
    int16_t yaw_deg_x10;
} MPU6050_DMP_AttitudeTypeDef;

HAL_StatusTypeDef MPU6050_DMP_Init(void);
HAL_StatusTypeDef MPU6050_DMP_Read(MPU6050_DMP_AttitudeTypeDef *attitude);
HAL_StatusTypeDef MPU6050_DMP_Stop(void);

#endif /* __MPU6050_DMP_H */
