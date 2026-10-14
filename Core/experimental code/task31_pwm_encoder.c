#include "task31_pwm_encoder.h"
#include "motor.h"
#include <stdio.h>

/**
 * @brief  任务3.1/3.2：初始化 PWM 和编码器
 * @param  None
 * @retval None
 */
void Task31_PWM_Encoder_Init(void)
{
    MOTOR_Init();
    MOTOR_SetEnabled(1U);
    MOTOR_ResetEncoders();

    printf("\r\n=== Task 3.1/3.2: PWM + Encoder Test ===\r\n");
    printf("PWM: TIM3_CH3/CH4 (20 kHz)\r\n");
    printf("Encoder: TIM8 (4x quadrature)\r\n");
    printf("Motor: B (CN2 interface)\r\n\r\n");
}

/**
 * @brief  设置电机速度（PWM占空比）
 * @param  speed: -1000 ~ +1000
 * @retval None
 */
void Task31_PWM_Encoder_SetSpeed(int16_t speed)
{
    MOTOR_B_SetSpeed(speed);
}

/**
 * @brief  读取编码器增量
 * @retval 编码器脉冲增量
 */
int16_t Task31_PWM_Encoder_ReadDelta(void)
{
    return MOTOR_B_ReadEncoderDelta();
}

/**
 * @brief  任务3.1/3.2：完整测试
 * @param  None
 * @retval None
 * @note   电机转速从 0 → 500 → 0，同时打印编码器值
 */
void Task31_PWM_Encoder_Test(void)
{
    int16_t speed = 0;
    int16_t target_speed = 500;
    int8_t direction = 1;

    Task31_PWM_Encoder_Init();

    printf("Starting PWM + Encoder test...\r\n");
    printf("Speed will ramp: 0 -> 500 -> 0\r\n\r\n");

    while (1)
    {
        /* 设置速度 */
        Task31_PWM_Encoder_SetSpeed(speed);

        /* 延时 100ms */
        HAL_Delay(100);

        /* 读取编码器增量 */
        int16_t encoder_delta = Task31_PWM_Encoder_ReadDelta();

        /* 计算实际速度（脉冲/秒） */
        int16_t actual_speed = encoder_delta * 10;

        /* 打印：VOFA+ FireWater 帧，逗号分隔三通道，\r\n 结尾
         * CH0 = Target 目标速度，CH1 = Actual 编码器实测（脉冲/秒），CH2 = PWM 当前控制量 */
        printf("%d,%d,%d\r\n",
               target_speed, actual_speed, speed);

        /* 改变速度 */
        speed += direction * 50;

        /* 到达边界时反向 */
        if (speed >= target_speed) {
            speed = target_speed;
            direction = -1;
        } else if (speed <= 0) {
            speed = 0;
            direction = 1;
        }
    }
}
