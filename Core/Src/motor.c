#include "motor.h"

#define MOTOR_PWM_FREQUENCY_HZ 20000UL

static uint16_t motor_pwm_counts;
static int32_t motor_a_encoder_total;
static int32_t motor_b_encoder_total;
static int16_t motor_a_requested_speed;
static int16_t motor_b_requested_speed;
static uint8_t motor_is_enabled;

static int16_t MOTOR_LimitSpeed(int16_t speed)
{
    if (speed > MOTOR_SPEED_MAX) {
        return MOTOR_SPEED_MAX;
    }
    if (speed < -MOTOR_SPEED_MAX) {
        return -MOTOR_SPEED_MAX;
    }
    return speed;
}

static void MOTOR_PWM_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    uint32_t timer_clock;
    uint32_t period;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();

    timer_clock = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_HCLK_DIV1) {
        timer_clock *= 2UL;
    }

    period = timer_clock / MOTOR_PWM_FREQUENCY_HZ;
    if (period < 2UL) {
        period = 2UL;
    } else if (period > 65535UL) {
        period = 65535UL;
    }
    motor_pwm_counts = (uint16_t)period;

    TIM3->CR1 = 0U;
    TIM3->PSC = 0U;
    TIM3->ARR = period - 1UL;

    TIM3->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE |
                  TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2PE;
    TIM3->CCMR2 = TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3PE |
                  TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4PE;
    TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;

    /* AT8236 brakes when both inputs are high. */
    TIM3->CCR1 = motor_pwm_counts;
    TIM3->CCR2 = motor_pwm_counts;
    TIM3->CCR3 = motor_pwm_counts;
    TIM3->CCR4 = motor_pwm_counts;
    TIM3->EGR = TIM_EGR_UG;
    TIM3->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;

    gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    HAL_GPIO_Init(GPIOB, &gpio);
}

static void MOTOR_EncoderTimerInit(TIM_TypeDef *timer)
{
    timer->CR1 = 0U;
    timer->PSC = 0U;
    timer->ARR = 0xFFFFU;
    timer->CNT = 0U;
    timer->SMCR = TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;
    timer->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0 |
                   (6UL << 4) | (6UL << 12);
    /* Encoder mode needs both input-capture channels enabled. */
    timer->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
    timer->EGR = TIM_EGR_UG;
    timer->CR1 = TIM_CR1_CEN;
}

static void MOTOR_EncoderInit(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_TIM8_CLK_ENABLE();

    gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &gpio);
    HAL_GPIO_Init(GPIOC, &gpio);

    MOTOR_EncoderTimerInit(TIM4);
    MOTOR_EncoderTimerInit(TIM8);
}

static void MOTOR_SetChannel(uint8_t motor, int16_t speed)
{
    uint32_t magnitude;
    uint32_t pwm_value;
    volatile uint32_t *in1;
    volatile uint32_t *in2;

    speed = MOTOR_LimitSpeed(speed);
    magnitude = (speed < 0) ? (uint32_t)(-(int32_t)speed) : (uint32_t)speed;
    pwm_value = (uint32_t)motor_pwm_counts -
                ((magnitude * (uint32_t)motor_pwm_counts) / MOTOR_SPEED_MAX);

    if (motor == 0U) {
        in1 = &TIM3->CCR1;
        in2 = &TIM3->CCR2;
    } else {
        in1 = &TIM3->CCR3;
        in2 = &TIM3->CCR4;
    }

    if (speed > 0) {
        *in1 = pwm_value;
        *in2 = motor_pwm_counts;
    } else if (speed < 0) {
        *in1 = motor_pwm_counts;
        *in2 = pwm_value;
    } else {
        *in1 = motor_pwm_counts;
        *in2 = motor_pwm_counts;
    }
}

void MOTOR_Init(void)
{
    MOTOR_PWM_Init();
    MOTOR_EncoderInit();
    MOTOR_ResetEncoders();
    motor_a_requested_speed = 0;
    motor_b_requested_speed = 0;
    motor_is_enabled = 1U;
    MOTOR_AllBrake();
}

void MOTOR_SetEnabled(uint8_t enabled)
{
    motor_is_enabled = (enabled != 0U) ? 1U : 0U;

    if (motor_is_enabled != 0U) {
        MOTOR_SetChannel(0U, motor_a_requested_speed);
        MOTOR_SetChannel(1U, motor_b_requested_speed);
    } else {
        TIM3->CCR1 = motor_pwm_counts;
        TIM3->CCR2 = motor_pwm_counts;
        TIM3->CCR3 = motor_pwm_counts;
        TIM3->CCR4 = motor_pwm_counts;
    }
}

uint8_t MOTOR_IsEnabled(void)
{
    return motor_is_enabled;
}

void MOTOR_A_SetSpeed(int16_t speed)
{
    motor_a_requested_speed = MOTOR_LimitSpeed((int16_t)(speed * MOTOR_A_DIRECTION_SIGN));
    if (motor_is_enabled != 0U) {
        MOTOR_SetChannel(0U, motor_a_requested_speed);
    }
}

void MOTOR_B_SetSpeed(int16_t speed)
{
    motor_b_requested_speed = MOTOR_LimitSpeed((int16_t)(speed * MOTOR_B_DIRECTION_SIGN));
    if (motor_is_enabled != 0U) {
        MOTOR_SetChannel(1U, motor_b_requested_speed);
    }
}

void MOTOR_SetSpeeds(int16_t motor_a, int16_t motor_b)
{
    MOTOR_A_SetSpeed(motor_a);
    MOTOR_B_SetSpeed(motor_b);
}

void MOTOR_A_Brake(void)
{
    motor_a_requested_speed = 0;
    TIM3->CCR1 = motor_pwm_counts;
    TIM3->CCR2 = motor_pwm_counts;
}

void MOTOR_B_Brake(void)
{
    motor_b_requested_speed = 0;
    TIM3->CCR3 = motor_pwm_counts;
    TIM3->CCR4 = motor_pwm_counts;
}

void MOTOR_AllBrake(void)
{
    MOTOR_A_Brake();
    MOTOR_B_Brake();
}

void MOTOR_A_Coast(void)
{
    TIM3->CCR1 = 0U;
    TIM3->CCR2 = 0U;
}

void MOTOR_B_Coast(void)
{
    TIM3->CCR3 = 0U;
    TIM3->CCR4 = 0U;
}

void MOTOR_AllCoast(void)
{
    MOTOR_A_Coast();
    MOTOR_B_Coast();
}

int16_t MOTOR_A_ReadEncoderDelta(void)
{
    int16_t delta = (int16_t)TIM4->CNT;
    TIM4->CNT = 0U;
    motor_a_encoder_total += delta;
    return delta;
}

int16_t MOTOR_B_ReadEncoderDelta(void)
{
    int16_t delta = (int16_t)TIM8->CNT;
    TIM8->CNT = 0U;
    motor_b_encoder_total += delta;
    return delta;
}

int32_t MOTOR_A_GetEncoderTotal(void)
{
    return motor_a_encoder_total;
}

int32_t MOTOR_B_GetEncoderTotal(void)
{
    return motor_b_encoder_total;
}

void MOTOR_ResetEncoders(void)
{
    TIM4->CNT = 0U;
    TIM8->CNT = 0U;
    motor_a_encoder_total = 0;
    motor_b_encoder_total = 0;
}
