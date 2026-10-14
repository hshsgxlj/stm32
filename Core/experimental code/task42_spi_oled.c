#include "task42_spi_oled.h"

#include "main.h"
#include "oled.h"

#include <string.h>
#include <stdlib.h>
#include <limits.h>

extern UART_HandleTypeDef huart1;

#define TASK42_RX_LINE_SIZE 64U

static char task42_rx_line[TASK42_RX_LINE_SIZE];
static uint8_t task42_rx_length;
static uint32_t task42_last_rx_tick;
static uint8_t task42_rx_it_byte;
static volatile uint8_t task42_line_ready;

static void Task42_HandleLine(const char *line);

static void Task42_ProcessLine(void)
{
    if (task42_rx_length == 0U) {
        return;
    }

    task42_rx_line[task42_rx_length] = '\0';
    Task42_HandleLine(task42_rx_line);
    task42_rx_length = 0U;
}

static uint8_t Task42_Equals(const char *left, const char *right)
{
    return (strcmp(left, right) == 0) ? 1U : 0U;
}

static void Task42_HandleLine(const char *line)
{
    if (strncmp(line, "NUM:", 4U) == 0) {
        char *end;
        long value = strtol(line + 4, &end, 10);
        if ((end != (line + 4)) && (*end == '\0') && (value >= INT16_MIN) && (value <= INT16_MAX)) {
            Task42_SPI_OLED_ShowNumber((int16_t)value);
        }
        return;
    }

    if (Task42_Equals(line, "CLEAR")) {
        OLED_Clear();
    } else if (Task42_Equals(line, "ON")) {
        OLED_DisplayOn();
    } else if (Task42_Equals(line, "OFF")) {
        OLED_DisplayOff();
    } else if (Task42_Equals(line, "OLED:AE")) {
        OLED_SendCommand(0xAEU);
    } else if (Task42_Equals(line, "OLED:AF")) {
        OLED_SendCommand(0xAFU);
    } else if (Task42_Equals(line, "OLED:8D")) {
        OLED_SendCommand(0x8DU);
        OLED_SendCommand(0x14U);
    } else if (line[0] != '\0') {
        Task42_SPI_OLED_ShowText(line);
    }
}

void Task42_SPI_OLED_Init(void)
{
    task42_rx_length = 0U;
    task42_last_rx_tick = HAL_GetTick();
    task42_line_ready = 0U;
    OLED_Init();

    /* Interrupt-driven RX: bytes are captured in the ISR, so a burst at
     * 115200 baud cannot overrun while the main loop is busy elsewhere. */
    HAL_NVIC_SetPriority(USART1_IRQn, 5U, 0U);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    (void)HAL_UART_Receive_IT(&huart1, &task42_rx_it_byte, 1U);
}

void Task42_SPI_OLED_ShowText(const char *text)
{
    OLED_Clear();
    OLED_ShowString(0U, 0U, text);
}

void Task42_SPI_OLED_ShowNumber(int16_t value)
{
    OLED_Clear();
    OLED_ShowString(0U, 2U, "NUM:");
    OLED_ShowInt16(30U, 2U, value);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t received;

    if (huart->Instance != USART1) {
        return;
    }

    received = task42_rx_it_byte;
    if ((received == '\r') || (received == '\n')) {
        task42_line_ready = 1U;
    } else if (task42_rx_length < (TASK42_RX_LINE_SIZE - 1U)) {
        task42_rx_line[task42_rx_length++] = (char)received;
    } else {
        /* Line too long: drop it rather than corrupt the display. */
        task42_rx_length = 0U;
    }
    task42_last_rx_tick = HAL_GetTick();

    (void)HAL_UART_Receive_IT(huart, &task42_rx_it_byte, 1U);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    /* Self-heal on any UART error (overrun, noise, framing...). */
    if (huart->Instance == USART1) {
        (void)HAL_UART_Receive_IT(huart, &task42_rx_it_byte, 1U);
    }
}

void Task42_SPI_OLED_PeriodicTask(void)
{
    if (task42_line_ready != 0U) {
        task42_line_ready = 0U;
        Task42_ProcessLine();
    }

    // Some serial tools send no CR/LF. Treat 30 ms of silence as end of line.
    if ((task42_rx_length != 0U) &&
        ((HAL_GetTick() - task42_last_rx_tick) >= 30U)) {
        Task42_ProcessLine();
    }
}
