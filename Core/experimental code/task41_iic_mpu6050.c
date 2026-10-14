#include "task41_iic_mpu6050.h"

#include "mpu6050.h"

#include <stdio.h>

#define TASK41_PRINT_PERIOD_MS       100U

static uint32_t task41_last_print_tick;

HAL_StatusTypeDef Task41_IIC_MPU6050_Init(void)
{
    /* Reuse the verified PB14/PB15 software-I2C driver used by the DMP task. */
    if (MPU6050_Init() != HAL_OK) return HAL_ERROR;

    task41_last_print_tick = HAL_GetTick();
    return HAL_OK;
}

HAL_StatusTypeDef Task41_IIC_MPU6050_Read(MPU6050_DataTypeDef *data)
{
    return MPU6050_ReadData(data);
}

void Task41_IIC_MPU6050_PeriodicTask(void)
{
    MPU6050_DataTypeDef data;
    uint32_t now = HAL_GetTick();

    if ((now - task41_last_print_tick) < TASK41_PRINT_PERIOD_MS) {
        return;
    }
    task41_last_print_tick = now;

    if (Task41_IIC_MPU6050_Read(&data) == HAL_OK) {
        printf("ACC:%d,%d,%d GYRO:%d,%d,%d\r\n",
               data.accel_x, data.accel_y, data.accel_z,
               data.gyro_x, data.gyro_y, data.gyro_z);
    } else {
        printf("MPU6050 read error\r\n");
    }
}
