#include "badapple.h"

#include "main.h"
#include "oled.h"

#define BADAPPLE_READY_BYTE       0x7AU
#define BADAPPLE_ACK_BYTE         0x06U
#define BADAPPLE_FRAME_READY_BYTE 0x52U
#define BADAPPLE_HEADER_FIRST     0xBAU
#define BADAPPLE_FRAME_SECOND     0xDAU
#define BADAPPLE_STOP_SECOND      0x5AU
#define BADAPPLE_BOARD_STOP_BYTE  0x18U
#define BADAPPLE_RX_TIMEOUT_MS    3000U
#define BADAPPLE_DEBOUNCE_MS      25U
#define BADAPPLE_CLICK_GAP_MS     500U

#define BADAPPLE_READ_TIMEOUT     0U
#define BADAPPLE_READ_OK          1U
#define BADAPPLE_READ_STOP        2U

#define BA_OLED_SCL_LOW()  HAL_GPIO_WritePin(OLED_SCL_PORT, OLED_SCL_PIN, GPIO_PIN_RESET)
#define BA_OLED_SCL_HIGH() HAL_GPIO_WritePin(OLED_SCL_PORT, OLED_SCL_PIN, GPIO_PIN_SET)
#define BA_OLED_SDA_LOW()  HAL_GPIO_WritePin(OLED_SDA_PORT, OLED_SDA_PIN, GPIO_PIN_RESET)
#define BA_OLED_SDA_HIGH() HAL_GPIO_WritePin(OLED_SDA_PORT, OLED_SDA_PIN, GPIO_PIN_SET)
#define BA_OLED_DC_CMD()   HAL_GPIO_WritePin(OLED_DC_PORT, OLED_DC_PIN, GPIO_PIN_RESET)
#define BA_OLED_DC_DATA()  HAL_GPIO_WritePin(OLED_DC_PORT, OLED_DC_PIN, GPIO_PIN_SET)

/* PA0 被 TIM2_CH1 等外设占用时，CubeMX 会从 main.h 移除 USER_Pin 定义；
 * 此时按键检测自动禁用（恒为松开），恢复按键配置后自动恢复。 */
#ifdef USER_Pin
#define BADAPPLE_READ_BUTTON() \
    ((HAL_GPIO_ReadPin(USER_GPIO_Port, USER_Pin) == GPIO_PIN_SET) ? 1U : 0U)
#else
#define BADAPPLE_READ_BUTTON() (0U)
#endif

_Static_assert(BADAPPLE_FRAME_SIZE ==
               (BADAPPLE_FRAME_WIDTH * BADAPPLE_FRAME_HEIGHT / 8U),
               "Bad Apple frame size does not match the OLED dimensions");

static uint8_t frame_buffer[BADAPPLE_FRAME_SIZE];
static uint8_t launcher_initialized;
static uint8_t raw_button_state;
static uint8_t stable_button_state;
static uint8_t click_count;
static uint32_t raw_state_changed_at;
static uint32_t last_click_at;
static uint8_t playback_button_enabled;
static uint8_t playback_raw_button_state;
static uint8_t playback_stable_button_state;
static uint8_t playback_click_count;
static uint32_t playback_raw_state_changed_at;
static uint32_t playback_last_click_at;

static void BadApple_Serial_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    uint32_t pclk;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();

    gpio.Pin = GPIO_PIN_9;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin = GPIO_PIN_10;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);

    USART1->CR1 = 0U;
    USART1->CR2 = 0U;
    USART1->CR3 = 0U;

    pclk = HAL_RCC_GetPCLK2Freq();
    USART1->BRR = (pclk + (BADAPPLE_UART_BAUD_RATE / 2U)) /
                  BADAPPLE_UART_BAUD_RATE;
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

static void BadApple_Serial_SendByte(uint8_t value)
{
    while ((USART1->SR & USART_SR_TXE) == 0U) {
    }
    USART1->DR = value;
}

static void BadApple_ResetPlaybackButton(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t current_button_state = BADAPPLE_READ_BUTTON();

    playback_raw_button_state = current_button_state;
    playback_stable_button_state = current_button_state;
    playback_click_count = 0U;
    playback_raw_state_changed_at = now;
    playback_last_click_at = now;
}

static uint8_t BadApple_PlaybackStopRequested(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t current_button_state = BADAPPLE_READ_BUTTON();

    if (current_button_state != playback_raw_button_state) {
        playback_raw_button_state = current_button_state;
        playback_raw_state_changed_at = now;
    }

    if ((playback_raw_button_state != playback_stable_button_state) &&
        ((now - playback_raw_state_changed_at) >= BADAPPLE_DEBOUNCE_MS)) {
        playback_stable_button_state = playback_raw_button_state;

        if (playback_stable_button_state != 0U) {
            if ((playback_click_count == 0U) ||
                ((now - playback_last_click_at) > BADAPPLE_CLICK_GAP_MS)) {
                playback_click_count = 1U;
            } else {
                ++playback_click_count;
            }
            playback_last_click_at = now;

            if (playback_click_count >= 3U) {
                playback_click_count = 0U;
                return 1U;
            }
        }
    }

    if ((playback_click_count != 0U) &&
        ((now - playback_last_click_at) > BADAPPLE_CLICK_GAP_MS)) {
        playback_click_count = 0U;
    }

    return 0U;
}

static uint8_t BadApple_Serial_ReadByte(uint8_t *value, uint32_t timeout_ms)
{
    uint32_t started_at = HAL_GetTick();

    for (;;) {
        if ((playback_button_enabled != 0U) &&
            (BadApple_PlaybackStopRequested() != 0U)) {
            return BADAPPLE_READ_STOP;
        }

        if ((USART1->SR & USART_SR_RXNE) != 0U) {
            *value = (uint8_t)USART1->DR;
            return BADAPPLE_READ_OK;
        }

        if ((HAL_GetTick() - started_at) >= timeout_ms) {
            return BADAPPLE_READ_TIMEOUT;
        }
    }
}

static uint8_t BadApple_ReceiveFrame(void)
{
    uint32_t started_at;

    __HAL_RCC_DMA1_CLK_ENABLE();

    DMA1_Channel5->CCR = 0U;
    DMA1->IFCR = DMA_IFCR_CGIF5;
    DMA1_Channel5->CPAR = (uint32_t)(uintptr_t)&USART1->DR;
    DMA1_Channel5->CMAR = (uint32_t)(uintptr_t)frame_buffer;
    DMA1_Channel5->CNDTR = BADAPPLE_FRAME_SIZE;
    DMA1_Channel5->CCR = DMA_CCR_MINC | DMA_CCR_PL_1;
    USART1->CR3 |= USART_CR3_DMAR;
    DMA1_Channel5->CCR |= DMA_CCR_EN;

    /* The computer sends frame bytes only after this DMA-ready response. */
    BadApple_Serial_SendByte(BADAPPLE_FRAME_READY_BYTE);
    started_at = HAL_GetTick();

    while (DMA1_Channel5->CNDTR != 0U) {
        if (BadApple_PlaybackStopRequested() != 0U) {
            DMA1_Channel5->CCR &= ~DMA_CCR_EN;
            USART1->CR3 &= ~USART_CR3_DMAR;
            DMA1->IFCR = DMA_IFCR_CGIF5;
            return BADAPPLE_READ_STOP;
        }
        if ((HAL_GetTick() - started_at) >= BADAPPLE_RX_TIMEOUT_MS) {
            DMA1_Channel5->CCR &= ~DMA_CCR_EN;
            USART1->CR3 &= ~USART_CR3_DMAR;
            DMA1->IFCR = DMA_IFCR_CGIF5;
            return BADAPPLE_READ_TIMEOUT;
        }
    }

    DMA1_Channel5->CCR &= ~DMA_CCR_EN;
    USART1->CR3 &= ~USART_CR3_DMAR;
    DMA1->IFCR = DMA_IFCR_CGIF5;
    return BADAPPLE_READ_OK;
}

static void BadApple_OLED_WriteByte(uint8_t value, uint8_t is_data)
{
    uint8_t bit;

    if (is_data != 0U) {
        BA_OLED_DC_DATA();
    } else {
        BA_OLED_DC_CMD();
    }

    for (bit = 0U; bit < 8U; ++bit) {
        BA_OLED_SCL_LOW();
        if ((value & 0x80U) != 0U) {
            BA_OLED_SDA_HIGH();
        } else {
            BA_OLED_SDA_LOW();
        }
        BA_OLED_SCL_HIGH();
        value <<= 1;
    }
}

static void BadApple_OLED_SetPage(uint8_t page)
{
    BadApple_OLED_WriteByte((uint8_t)(0xB0U + page), 0U);
    BadApple_OLED_WriteByte(0x00U, 0U);
    BadApple_OLED_WriteByte(0x10U, 0U);
}

static void BadApple_ShowFrame(const uint8_t frame[BADAPPLE_FRAME_SIZE])
{
    uint8_t page;
    uint8_t column;
    uint16_t index = 0U;

    if (frame == NULL) {
        return;
    }

    for (page = 0U; page < 8U; ++page) {
        BadApple_OLED_SetPage(page);
        for (column = 0U; column < 128U; ++column) {
            BadApple_OLED_WriteByte(frame[index], 1U);
            ++index;
        }
    }
}

static void BadApple_Init(void)
{
    BadApple_Serial_Init();
    OLED_Init();
    OLED_Clear();
    OLED_ShowString(0U, 2U, "WAIT USB");
}

static void BadApple_Run(void)
{
    static const uint8_t start_magic[4] = {'B', 'A', 'D', '!'};
    uint8_t start_position = 0U;
    uint8_t first;
    uint8_t second;
    uint8_t read_status;

    /* Wait for the four-byte BAD! synchronization word. */
    playback_button_enabled = 0U;
    while (start_position < sizeof(start_magic)) {
        if (BadApple_Serial_ReadByte(&first, BADAPPLE_RX_TIMEOUT_MS) !=
            BADAPPLE_READ_OK) {
            continue;
        }
        if (first == start_magic[start_position]) {
            ++start_position;
        } else {
            start_position = (first == start_magic[0]) ? 1U : 0U;
        }
    }

    /* Finish all slow OLED work before telling the computer to transmit. */
    OLED_Clear();
    BadApple_ResetPlaybackButton();
    playback_button_enabled = 1U;
    BadApple_Serial_SendByte(BADAPPLE_READY_BYTE);

    for (;;) {
        read_status = BadApple_Serial_ReadByte(&first, BADAPPLE_RX_TIMEOUT_MS);
        if (read_status == BADAPPLE_READ_STOP) {
            BadApple_Serial_SendByte(BADAPPLE_BOARD_STOP_BYTE);
            break;
        }
        if (read_status != BADAPPLE_READ_OK) {
            continue;
        }
        if (first != BADAPPLE_HEADER_FIRST) {
            continue;
        }
        read_status = BadApple_Serial_ReadByte(&second, BADAPPLE_RX_TIMEOUT_MS);
        if (read_status == BADAPPLE_READ_STOP) {
            BadApple_Serial_SendByte(BADAPPLE_BOARD_STOP_BYTE);
            break;
        }
        if (read_status != BADAPPLE_READ_OK) {
            continue;
        }

        if (second == BADAPPLE_FRAME_SECOND) {
            read_status = BadApple_ReceiveFrame();
            if (read_status == BADAPPLE_READ_STOP) {
                BadApple_Serial_SendByte(BADAPPLE_BOARD_STOP_BYTE);
                break;
            }
            if (read_status == BADAPPLE_READ_OK) {
                BadApple_ShowFrame(frame_buffer);
                BadApple_Serial_SendByte(BADAPPLE_ACK_BYTE);
            }
        } else if (second == BADAPPLE_STOP_SECOND) {
            BadApple_Serial_SendByte(BADAPPLE_ACK_BYTE);
            break;
        }
    }

    playback_button_enabled = 0U;
}

void BadApple_Start(volatile uint8_t *outputs_requested_on)
{
    uint32_t now;
    uint8_t current_button_state;

    if (outputs_requested_on == NULL) {
        return;
    }

    now = HAL_GetTick();
    current_button_state = BADAPPLE_READ_BUTTON();

    if (launcher_initialized == 0U) {
        /* Button handling moves from EXTI to this debounced click detector. */
#ifdef USER_EXTI_IRQn
        HAL_NVIC_DisableIRQ(USER_EXTI_IRQn);
#endif
        raw_button_state = current_button_state;
        stable_button_state = current_button_state;
        raw_state_changed_at = now;
        last_click_at = now;
        click_count = 0U;
        launcher_initialized = 1U;
        return;
    }

    if (current_button_state != raw_button_state) {
        raw_button_state = current_button_state;
        raw_state_changed_at = now;
    }

    if ((raw_button_state != stable_button_state) &&
        ((now - raw_state_changed_at) >= BADAPPLE_DEBOUNCE_MS)) {
        stable_button_state = raw_button_state;

        if (stable_button_state != 0U) {
            if ((click_count == 0U) ||
                ((now - last_click_at) > BADAPPLE_CLICK_GAP_MS)) {
                click_count = 1U;
            } else {
                ++click_count;
            }
            last_click_at = now;

            if (click_count >= 3U) {
                uint8_t display_was_requested_on = *outputs_requested_on;

                click_count = 0U;
                BadApple_Init();
                BadApple_Run();

                if (display_was_requested_on != 0U) {
                    OLED_DisplayOn();
                } else {
                    OLED_DisplayOff();
                }

                now = HAL_GetTick();
                current_button_state = BADAPPLE_READ_BUTTON();
                raw_button_state = current_button_state;
                stable_button_state = current_button_state;
                raw_state_changed_at = now;
                last_click_at = now;
                return;
            }
        }
    }

    if ((click_count != 0U) &&
        ((now - last_click_at) > BADAPPLE_CLICK_GAP_MS)) {
        /* One click toggles power. Two clicks have the same net effect as two toggles. */
        if ((click_count & 1U) != 0U) {
            *outputs_requested_on ^= 1U;
        }
        click_count = 0U;
    }
}
