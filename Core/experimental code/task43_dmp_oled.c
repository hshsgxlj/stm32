#include "task43_dmp_oled.h"

#include "mpu6050_dmp.h"
#include "oled.h"

static MPU6050_DMP_AttitudeTypeDef task43_attitude;
static int16_t task43_yaw_zero_x10;
static uint32_t task43_calibration_start_tick;
static uint8_t task43_yaw_zero_ready;

static int16_t Task43_WrapAngleX10(int32_t angle_x10)
{
    while (angle_x10 > 1800) angle_x10 -= 3600;
    while (angle_x10 < -1800) angle_x10 += 3600;
    return (int16_t)angle_x10;
}

HAL_StatusTypeDef Task43_DMP_OLED_Init(void)
{
    task43_attitude.pitch_deg_x10 = 0;
    task43_attitude.roll_deg_x10 = 0;
    task43_attitude.yaw_deg_x10 = 0;
    task43_yaw_zero_x10 = 0;
    task43_calibration_start_tick = 0U;
    task43_yaw_zero_ready = 0U;

    if (MPU6050_DMP_Init() != HAL_OK) {
        return HAL_ERROR;
    }
    task43_calibration_start_tick = HAL_GetTick();
    return HAL_OK;
}

void Task43_DMP_OLED_PeriodicTask(void)
{
    if (MPU6050_DMP_Read(&task43_attitude) == HAL_OK) {
        uint32_t now = HAL_GetTick();

        /*
         * DMP gyro calibration starts after about eight seconds.  Keep the
         * display at zero for a short settling margin, then capture the
         * post-calibration yaw as the relative zero.  This avoids showing the
         * DMP's bias-correction step as a real rotation.
         */
        if ((task43_yaw_zero_ready == 0U) &&
            ((now - task43_calibration_start_tick) >= TASK43_YAW_CALIBRATION_MS)) {
            task43_yaw_zero_x10 = task43_attitude.yaw_deg_x10;
            task43_yaw_zero_ready = 1U;
        }

        if (task43_yaw_zero_ready != 0U) {
            task43_attitude.yaw_deg_x10 = Task43_WrapAngleX10(
                (int32_t)task43_attitude.yaw_deg_x10 - task43_yaw_zero_x10);
        } else {
            task43_attitude.yaw_deg_x10 = 0;
        }

        OLED_ShowString(0U, 0U, "P:");
        OLED_ShowAngle(12U, 0U, task43_attitude.pitch_deg_x10);
        OLED_ShowString(0U, 2U, "R:");
        OLED_ShowAngle(12U, 2U, task43_attitude.roll_deg_x10);
        OLED_ShowString(0U, 4U, "Y:");
        OLED_ShowAngle(12U, 4U, task43_attitude.yaw_deg_x10);
    }
}
