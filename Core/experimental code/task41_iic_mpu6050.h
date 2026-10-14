#ifndef TASK41_IIC_MPU6050_H
#define TASK41_IIC_MPU6050_H

#include "mpu6050.h"

/* 实验4任务1：IIC 读取 MPU6050 原始加速度和陀螺仪数据。 */
HAL_StatusTypeDef Task41_IIC_MPU6050_Init(void);
HAL_StatusTypeDef Task41_IIC_MPU6050_Read(MPU6050_DataTypeDef *data);
void Task41_IIC_MPU6050_PeriodicTask(void);

#endif /* TASK41_IIC_MPU6050_H */
