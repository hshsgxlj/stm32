#include "track.h"

void TRACK_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    gpio.Pin = TRACK_P1_PIN | TRACK_P2_PIN | TRACK_P3_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &gpio);

    gpio.Pin = TRACK_P4_PIN;
    HAL_GPIO_Init(GPIOB, &gpio);
}

void TRACK_Read(TRACK_StateTypeDef *state)
{
    if (state == NULL) {
        return;
    }

    state->p1 = (HAL_GPIO_ReadPin(TRACK_P1_PORT, TRACK_P1_PIN) == GPIO_PIN_SET) ? 1U : 0U;
    state->p2 = (HAL_GPIO_ReadPin(TRACK_P2_PORT, TRACK_P2_PIN) == GPIO_PIN_SET) ? 1U : 0U;
    state->p3 = (HAL_GPIO_ReadPin(TRACK_P3_PORT, TRACK_P3_PIN) == GPIO_PIN_SET) ? 1U : 0U;
    state->p4 = (HAL_GPIO_ReadPin(TRACK_P4_PORT, TRACK_P4_PIN) == GPIO_PIN_SET) ? 1U : 0U;
}
