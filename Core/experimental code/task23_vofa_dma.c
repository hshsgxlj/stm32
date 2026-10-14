#include "task23_vofa_dma.h"

#include "main.h"
#include "motor.h"
#include "stm32f1xx_hal_dma.h"
#include "stm32f1xx_hal_uart.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

extern UART_HandleTypeDef huart1;

#define TASK23_RX_BUFFER_SIZE       64U
#define TASK23_COMMAND_SIZE         64U
#define TASK23_SAMPLE_PERIOD_MS     20U
#define TASK23_DEFAULT_TARGET       20000.0f
#define TASK23_DEFAULT_KP           0.030f
#define TASK23_DEFAULT_KI           0.020f
#define TASK23_DEFAULT_KD           0.000f

static uint8_t task23_rx_dma_buffer[TASK23_RX_BUFFER_SIZE];
static volatile uint16_t task23_rx_last_position;
static volatile uint8_t task23_command_buffer[TASK23_COMMAND_SIZE];
static volatile uint16_t task23_command_length;
static volatile uint8_t task23_command_ready;

static uint32_t task23_last_sample_time;
static float task23_target = TASK23_DEFAULT_TARGET;
static float task23_actual;
static float task23_pwm;
static float task23_integral;
static float task23_previous_error;

static void Task23_ProcessCommand(void)
{
    char command[TASK23_COMMAND_SIZE];
    char *separator;
    char *end;
    float value;
    uint16_t length;

    if (task23_command_ready == 0U) {
        return;
    }

    __disable_irq();
    length = task23_command_length;
    if (length >= TASK23_COMMAND_SIZE) {
        length = TASK23_COMMAND_SIZE - 1U;
    }
    memcpy(command, (const void *)task23_command_buffer, length);
    task23_command_length = 0U;
    task23_command_ready = 0U;
    __enable_irq();

    command[length] = '\0';
    separator = strchr(command, '=');
    if (separator == NULL) {
        separator = strchr(command, ':');
    }
    if (separator != NULL) {
        *separator = '\0';
        value = strtof(separator + 1, &end);
        if (end != separator + 1 && isfinite(value)) {
            if (strcmp(command, "target") == 0) {
                Task23_VOFA_SetTarget(value);
            } else if (strcmp(command, "pwm") == 0) {
                Task23_VOFA_SetPwm((int16_t)value);
            }
        }
    } else {
        value = strtof(command, &end);
        if (end != command && isfinite(value)) {
            Task23_VOFA_SetTarget(value);
        }
    }
}

static void Task23_SendJustFloat(void)
{
    uint8_t frame[16];
    uint32_t tail = 0x7F800000UL;

    /* JustFloat：小端序发送 3 个 float，末尾是 0x7F800000 帧尾。 */
    memcpy(&frame[0], &task23_target, sizeof(float));
    memcpy(&frame[4], &task23_actual, sizeof(float));
    memcpy(&frame[8], &task23_pwm, sizeof(float));
    memcpy(&frame[12], &tail, sizeof(tail));
    (void)HAL_UART_Transmit(&huart1, frame, sizeof(frame), 20U);
}

static int16_t Task23_UpdateSpeedPid(float target, float actual, float dt)
{
    float error = target - actual;
    float derivative;
    float output;

    if (dt <= 0.0f) {
        dt = 0.001f;
    }
    if (dt > 0.2f) {
        dt = 0.2f;
    }

    derivative = (error - task23_previous_error) / dt;
    task23_integral += error * dt;
    output = (TASK23_DEFAULT_KP * error) +
             (TASK23_DEFAULT_KI * task23_integral) +
             (TASK23_DEFAULT_KD * derivative);

    /* Anti-windup: keep the integral from growing while PWM is saturated. */
    if (output > (float)MOTOR_SPEED_MAX) {
        output = (float)MOTOR_SPEED_MAX;
        if (error > 0.0f) {
            task23_integral -= error * dt;
        }
    } else if (output < -(float)MOTOR_SPEED_MAX) {
        output = -(float)MOTOR_SPEED_MAX;
        if (error < 0.0f) {
            task23_integral -= error * dt;
        }
    }

    task23_previous_error = error;
    return (int16_t)output;
}

void Task23_VOFA_Init(void)
{
    task23_rx_last_position = 0U;
    task23_command_length = 0U;
    task23_command_ready = 0U;
    task23_last_sample_time = HAL_GetTick();
    task23_actual = 0.0f;
    task23_pwm = 0.0f;
    task23_integral = 0.0f;
    task23_previous_error = task23_target;

    /* MSP 中配置的是 DMA1_Channel5，模式为 CIRCULAR。 */
    if (HAL_UART_Receive_DMA(&huart1, task23_rx_dma_buffer,
                             sizeof(task23_rx_dma_buffer)) != HAL_OK) {
        Error_Handler();
    }
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

    MOTOR_B_SetSpeed(0);
}

void Task23_VOFA_PeriodicTask(void)
{
    uint32_t now;
    uint32_t elapsed;
    int16_t encoder_delta;
    int16_t pwm_command;

    Task23_ProcessCommand();

    now = HAL_GetTick();
    elapsed = now - task23_last_sample_time;
    if (elapsed < TASK23_SAMPLE_PERIOD_MS) {
        return;
    }

    encoder_delta = MOTOR_B_ReadEncoderDelta();
    task23_actual = ((float)encoder_delta * 1000.0f) / (float)elapsed;
    task23_last_sample_time = now;

    /* 目标值和实际值都使用 encoder counts/s，PWM 是 PID 输出。 */
    pwm_command = Task23_UpdateSpeedPid(task23_target, task23_actual,
                                        (float)elapsed / 1000.0f);
    task23_pwm = (float)pwm_command;
    MOTOR_B_SetSpeed(pwm_command);
    Task23_SendJustFloat();
}

void Task23_VOFA_SetTarget(float target)
{
    if (isfinite(target)) {
        task23_target = target;
        task23_integral = 0.0f;
        task23_previous_error = 0.0f;
    }
}

void Task23_VOFA_SetPwm(int16_t pwm)
{
    if (pwm > MOTOR_SPEED_MAX) {
        pwm = MOTOR_SPEED_MAX;
    } else if (pwm < -MOTOR_SPEED_MAX) {
        pwm = -MOTOR_SPEED_MAX;
    }
    task23_pwm = (float)pwm;
    MOTOR_B_SetSpeed(pwm);
}

void Task23_VOFA_USART1_IdleCallback(void)
{
    uint16_t position;

    if (huart1.hdmarx == NULL) {
        return;
    }

    position = (uint16_t)(TASK23_RX_BUFFER_SIZE -
                          __HAL_DMA_GET_COUNTER(huart1.hdmarx));
    if (position >= TASK23_RX_BUFFER_SIZE) {
        position = 0U;
    }

    while (task23_rx_last_position != position) {
        if (task23_command_ready == 0U &&
            task23_command_length < (TASK23_COMMAND_SIZE - 1U)) {
            task23_command_buffer[task23_command_length++] =
                task23_rx_dma_buffer[task23_rx_last_position];
        }
        task23_rx_last_position++;
        if (task23_rx_last_position >= TASK23_RX_BUFFER_SIZE) {
            task23_rx_last_position = 0U;
        }
    }

    if (task23_command_length != 0U) {
        task23_command_ready = 1U;
    }
}
