#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f1xx_hal.h"

/* C10B MPU6050 socket: PB14=SCL, PB15=SDA, PB9=INT. */
#define MPU6050_SCL_PORT GPIOB
#define MPU6050_SCL_PIN  GPIO_PIN_14
#define MPU6050_SDA_PORT GPIOB
#define MPU6050_SDA_PIN  GPIO_PIN_15
#define MPU6050_INT_PORT GPIOB
#define MPU6050_INT_PIN  GPIO_PIN_9

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} MPU6050_DataTypeDef;

typedef struct {
    int16_t accel_x_mg;
    int16_t accel_y_mg;
    int16_t accel_z_mg;
    int16_t gyro_x_dps_x10;
    int16_t gyro_y_dps_x10;
    int16_t gyro_z_dps_x10;
} MPU6050_FilteredDataTypeDef;

typedef struct {
    int16_t pitch_deg_x10;
    int16_t roll_deg_x10;
    int16_t yaw_deg_x10;
} MPU6050_AttitudeTypeDef;

HAL_StatusTypeDef MPU6050_Init(void);
HAL_StatusTypeDef MPU6050_ReadData(MPU6050_DataTypeDef *data);
HAL_StatusTypeDef MPU6050_Calibrate(uint16_t sample_count);
HAL_StatusTypeDef MPU6050_ReadFiltered(MPU6050_FilteredDataTypeDef *data);
HAL_StatusTypeDef MPU6050_ReadAttitude(MPU6050_AttitudeTypeDef *attitude);
void MPU6050_BusInit(void);
HAL_StatusTypeDef MPU6050_BusWrite(uint8_t address, uint8_t reg,
                                  const uint8_t *data, uint8_t length);
HAL_StatusTypeDef MPU6050_BusRead(uint8_t address, uint8_t reg,
                                 uint8_t *data, uint8_t length);

#endif /* __MPU6050_H */
