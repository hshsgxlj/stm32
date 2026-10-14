#include "mpu6050_dmp.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu6050.h"
#include <math.h>

#define MPU6050_DMP_FIFO_RATE 20U
#define MPU6050_DMP_ADDRESS   0x68U
#define MPU6050_FIFO_COUNTH   0x72U
#define MPU6050_DMP_PACKET_SIZE 16U
#define MPU6050_DMP_Q30_SCALE 1073741824.0f
#define MPU6050_RAD_TO_DEG    57.2957795f

static const signed char dmp_orientation[9] = {
    -1,  0,  0,
     0, -1,  0,
     0,  0,  1
};

static unsigned short MPU6050_DMP_RowToScale(const signed char *row)
{
    if (row[0] > 0) return 0U;
    if (row[0] < 0) return 4U;
    if (row[1] > 0) return 1U;
    if (row[1] < 0) return 5U;
    if (row[2] > 0) return 2U;
    if (row[2] < 0) return 6U;
    return 7U;
}

static unsigned short MPU6050_DMP_OrientationToScalar(const signed char *matrix)
{
    unsigned short scalar = MPU6050_DMP_RowToScale(matrix);
    scalar |= (unsigned short)(MPU6050_DMP_RowToScale(matrix + 3) << 3);
    scalar |= (unsigned short)(MPU6050_DMP_RowToScale(matrix + 6) << 6);
    return scalar;
}

HAL_StatusTypeDef MPU6050_DMP_Init(void)
{
    MPU6050_BusInit();
    HAL_Delay(100);

    if (mpu_init() != 0) return HAL_ERROR;
    if (mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL) != 0) return HAL_ERROR;
    if (mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL) != 0) return HAL_ERROR;
    if (mpu_set_sample_rate(100U) != 0) return HAL_ERROR;
    if (dmp_load_motion_driver_firmware() != 0) return HAL_ERROR;
    if (dmp_set_orientation(MPU6050_DMP_OrientationToScalar(dmp_orientation)) != 0) {
        return HAL_ERROR;
    }
    if (dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT |
                           DMP_FEATURE_GYRO_CAL) != 0) {
        return HAL_ERROR;
    }
    if (dmp_set_fifo_rate(MPU6050_DMP_FIFO_RATE) != 0) return HAL_ERROR;
    if (mpu_set_dmp_state(1U) != 0) return HAL_ERROR;

    HAL_Delay(100);
    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_DMP_Stop(void)
{
    int result = 0;

    if (mpu_set_dmp_state(0U) != 0) result = -1;
    if (mpu_set_sensors(0U) != 0) result = -1;
    return (result == 0) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef MPU6050_DMP_Read(MPU6050_DMP_AttitudeTypeDef *attitude)
{
    short gyro[3];
    short accel[3];
    short sensors;
    long quaternion[4];
    unsigned long timestamp;
    unsigned char more;
    uint8_t fifo_count_bytes[2];
    uint16_t fifo_count;
    float q0;
    float q1;
    float q2;
    float q3;
    float asin_input;
    float pitch;
    float roll;
    float yaw;

    if (attitude == NULL) return HAL_ERROR;

    if (MPU6050_BusRead(MPU6050_DMP_ADDRESS, MPU6050_FIFO_COUNTH,
                        fifo_count_bytes, 2U) != HAL_OK) {
        return HAL_ERROR;
    }
    fifo_count = (uint16_t)(((uint16_t)fifo_count_bytes[0] << 8) |
                            fifo_count_bytes[1]);
    if (fifo_count < MPU6050_DMP_PACKET_SIZE) {
        return HAL_BUSY;
    }

    do {
        if (dmp_read_fifo(gyro, accel, quaternion, &timestamp, &sensors, &more) != 0) {
            return HAL_ERROR;
        }
    } while (more != 0U);

    if ((sensors & INV_WXYZ_QUAT) == 0) return HAL_ERROR;

    q0 = (float)quaternion[0] / MPU6050_DMP_Q30_SCALE;
    q1 = (float)quaternion[1] / MPU6050_DMP_Q30_SCALE;
    q2 = (float)quaternion[2] / MPU6050_DMP_Q30_SCALE;
    q3 = (float)quaternion[3] / MPU6050_DMP_Q30_SCALE;

    asin_input = (-2.0f * q1 * q3) + (2.0f * q0 * q2);
    if (asin_input > 1.0f) asin_input = 1.0f;
    if (asin_input < -1.0f) asin_input = -1.0f;

    roll = asinf(asin_input) * MPU6050_RAD_TO_DEG;
    pitch = atan2f((2.0f * q2 * q3) + (2.0f * q0 * q1),
                   1.0f - (2.0f * q1 * q1) - (2.0f * q2 * q2)) * MPU6050_RAD_TO_DEG;
    yaw = atan2f(2.0f * ((q1 * q2) + (q0 * q3)),
                 (q0 * q0) + (q1 * q1) - (q2 * q2) - (q3 * q3)) * MPU6050_RAD_TO_DEG;

    attitude->pitch_deg_x10 = (int16_t)(pitch * 10.0f);
    attitude->roll_deg_x10 = (int16_t)(roll * 10.0f);
    attitude->yaw_deg_x10 = (int16_t)(yaw * 10.0f);
    return HAL_OK;
}
