/**
 * @file    exp_mpu6050_test.c
 * @brief   MPU6050 陀螺仪姿态测试实验
 * @details 使用 DMP 读取 Pitch、Roll、Yaw 角度并在 OLED 上显示
 * @note    配合巡线传感器和按键中断使用
 */

#include "exp_mpu6050_test.h"
#include "mpu6050_dmp.h"
#include "track.h"
#include "oled.h"
#include "led.h"
#include "motor.h"
#include "badapple.h"
#include "stm32f1xx_hal.h"

extern volatile uint8_t outputs_requested_on; // 定义在 chapter.c

/**
 * @brief  MPU6050 DMP 姿态角测试
 * @param  None
 * @retval None
 * @note   完整的 MPU6050 + OLED + 巡线 + 按键中断测试
 */
void EXP_MPU6050_AttitudeTest(void)
{
  uint8_t outputs_are_on = 1U;
  TRACK_StateTypeDef track_state = {0};
  MPU6050_DMP_AttitudeTypeDef mpu_data = {0};
  uint8_t mpu_is_ready;
  uint8_t dmp_has_data = 0U;
  uint32_t last_mpu_retry = 0U;

  // KEY_Init(); /* PA0由CubeMX配置为EXTI0，不能再初始化成普通输入 */
  TRACK_Init();
  OLED_Init();
  OLED_Clear();
  OLED_ShowString(0, 1, "PITCH:");
  OLED_ShowString(0, 3, "ROLL:");
  OLED_ShowString(0, 5, "YAW:");
  OLED_ShowString(0, 7, "I:");
  mpu_is_ready = 0U;
  OLED_ShowChar(12, 7, '0');

  /* 初始时LED和OLED都开启，按下USER按键后由中断同步切换。 */
  LED_All_On();
  OLED_DisplayOn();

  /* Load the MPU6050 DMP firmware and start quaternion output. */
  mpu_is_ready = (MPU6050_DMP_Init() == HAL_OK) ? 1U : 0U;

  while (1) {
    BadApple_Start(&outputs_requested_on);

    if (outputs_are_on != outputs_requested_on) {
      outputs_are_on = outputs_requested_on;
      if (outputs_are_on != 0U) {
        LED_All_On();
        OLED_DisplayOn();
        MOTOR_SetEnabled(1U);
        dmp_has_data = 0U;
        mpu_is_ready = (MPU6050_DMP_Init() == HAL_OK) ? 1U : 0U;
        last_mpu_retry = HAL_GetTick();
      } else {
        MOTOR_SetEnabled(0U);
        (void)MPU6050_DMP_Stop();
        mpu_is_ready = 0U;
        dmp_has_data = 0U;
        LED_All_Off();
        OLED_DisplayOff();
      }
    }

    /* Software power-off: keep only the USER interrupt and main loop alive. */
    if (outputs_are_on == 0U) {
      HAL_Delay(20);
      continue;
    }

    TRACK_Read(&track_state);
    OLED_ShowChar(12, 0, (char)('0' + track_state.p1));
    OLED_ShowChar(18, 0, (char)('0' + track_state.p2));
    OLED_ShowChar(24, 0, (char)('0' + track_state.p3));
    OLED_ShowChar(30, 0, (char)('0' + track_state.p4));

    if (mpu_is_ready != 0U) {
      HAL_StatusTypeDef dmp_status = MPU6050_DMP_Read(&mpu_data);
      if (dmp_status == HAL_OK) {
        dmp_has_data = 1U;
      } else if (dmp_status == HAL_ERROR) {
        mpu_is_ready = 0U;
        dmp_has_data = 0U;
        last_mpu_retry = HAL_GetTick();
      }
    } else if ((HAL_GetTick() - last_mpu_retry) >= 1000U) {
      last_mpu_retry = HAL_GetTick();
      mpu_is_ready = (MPU6050_DMP_Init() == HAL_OK) ? 1U : 0U;
      dmp_has_data = 0U;
    }

    OLED_ShowAngle(42, 1, mpu_data.pitch_deg_x10);
    OLED_ShowAngle(42, 3, mpu_data.roll_deg_x10);
    OLED_ShowAngle(42, 5, mpu_data.yaw_deg_x10);
    OLED_ShowChar(12, 7, (char)('0' + ((mpu_is_ready != 0U) ?
                                      (1U + dmp_has_data) : 0U)));

    HAL_Delay(20);
  }
}
