#include "oled.h"

#define OLED_SCL_LOW()  HAL_GPIO_WritePin(OLED_SCL_PORT, OLED_SCL_PIN, GPIO_PIN_RESET)
#define OLED_SCL_HIGH() HAL_GPIO_WritePin(OLED_SCL_PORT, OLED_SCL_PIN, GPIO_PIN_SET)
#define OLED_SDA_LOW()  HAL_GPIO_WritePin(OLED_SDA_PORT, OLED_SDA_PIN, GPIO_PIN_RESET)
#define OLED_SDA_HIGH() HAL_GPIO_WritePin(OLED_SDA_PORT, OLED_SDA_PIN, GPIO_PIN_SET)
#define OLED_RST_LOW()  HAL_GPIO_WritePin(OLED_RST_PORT, OLED_RST_PIN, GPIO_PIN_RESET)
#define OLED_RST_HIGH() HAL_GPIO_WritePin(OLED_RST_PORT, OLED_RST_PIN, GPIO_PIN_SET)
#define OLED_DC_CMD()   HAL_GPIO_WritePin(OLED_DC_PORT, OLED_DC_PIN, GPIO_PIN_RESET)
#define OLED_DC_DATA()  HAL_GPIO_WritePin(OLED_DC_PORT, OLED_DC_PIN, GPIO_PIN_SET)

static void OLED_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* PB3/PB4 are JTAG pins after reset. Disable JTAG but keep SWD enabled. */
    __HAL_AFIO_REMAP_SWJ_NOJTAG();

    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;

#if OLED_USE_SPI2
    gpio.Pin = OLED_RST_PIN | OLED_DC_PIN;
#else
    gpio.Pin = OLED_SDA_PIN | OLED_RST_PIN | OLED_DC_PIN;
#endif
    HAL_GPIO_Init(GPIOB, &gpio);

#if !OLED_USE_SPI2
    gpio.Pin = OLED_SCL_PIN;
    HAL_GPIO_Init(GPIOC, &gpio);

    OLED_SCL_HIGH();
    OLED_SDA_HIGH();
#endif
    OLED_DC_DATA();
    OLED_RST_HIGH();
}

static void OLED_WriteByte(uint8_t value, GPIO_PinState is_data)
{
    if (is_data == GPIO_PIN_SET) {
        OLED_DC_DATA();
    } else {
        OLED_DC_CMD();
    }

    for (uint8_t bit = 0; bit < 8; ++bit) {
        OLED_SCL_LOW();
        if ((value & 0x80U) != 0U) {
            OLED_SDA_HIGH();
        } else {
            OLED_SDA_LOW();
        }
        OLED_SCL_HIGH();
        value <<= 1;
    }
}

static void OLED_WriteCommand(uint8_t command)
{
    OLED_WriteByte(command, GPIO_PIN_RESET);
}

void OLED_SendCommand(uint8_t command)
{
    OLED_WriteCommand(command);
}

static void OLED_WriteData(uint8_t data)
{
    OLED_WriteByte(data, GPIO_PIN_SET);
}

static void OLED_SetCursor(uint8_t x, uint8_t page)
{
    OLED_WriteCommand((uint8_t)(0xB0U + page));
    OLED_WriteCommand((uint8_t)(x & 0x0FU));
    OLED_WriteCommand((uint8_t)(0x10U | (x >> 4)));
}

static const uint8_t *OLED_GetGlyph(char character)
{
    static const uint8_t glyph_space[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    static const uint8_t glyph_minus[6] = {0x08, 0x08, 0x08, 0x08, 0x08, 0x00};
    static const uint8_t glyph_dot[6] = {0x00, 0x60, 0x60, 0x00, 0x00, 0x00};
    static const uint8_t glyph_colon[6] = {0x00, 0x36, 0x36, 0x00, 0x00, 0x00};
    static const uint8_t glyph_a[6] = {0x7E, 0x11, 0x11, 0x11, 0x7E, 0x00};
    static const uint8_t glyph_c[6] = {0x3E, 0x41, 0x41, 0x41, 0x22, 0x00};
    static const uint8_t glyph_d[6] = {0x7F, 0x41, 0x41, 0x22, 0x1C, 0x00};
    static const uint8_t glyph_e[6] = {0x7F, 0x49, 0x49, 0x49, 0x41, 0x00};
    static const uint8_t glyph_g[6] = {0x3E, 0x41, 0x49, 0x49, 0x7A, 0x00};
    static const uint8_t glyph_h[6] = {0x7F, 0x08, 0x08, 0x08, 0x7F, 0x00};
    static const uint8_t glyph_i[6] = {0x00, 0x41, 0x7F, 0x41, 0x00, 0x00};
    static const uint8_t glyph_l[6] = {0x7F, 0x40, 0x40, 0x40, 0x40, 0x00};
    static const uint8_t glyph_o[6] = {0x3E, 0x41, 0x41, 0x41, 0x3E, 0x00};
    static const uint8_t glyph_p[6] = {0x7F, 0x09, 0x09, 0x09, 0x06, 0x00};
    static const uint8_t glyph_r[6] = {0x7F, 0x09, 0x19, 0x29, 0x46, 0x00};
    static const uint8_t glyph_t[6] = {0x01, 0x01, 0x7F, 0x01, 0x01, 0x00};
    static const uint8_t glyph_w[6] = {0x3F, 0x40, 0x38, 0x40, 0x3F, 0x00};
    static const uint8_t glyph_x[6] = {0x63, 0x14, 0x08, 0x14, 0x63, 0x00};
    static const uint8_t glyph_y[6] = {0x07, 0x08, 0x70, 0x08, 0x07, 0x00};
    static const uint8_t glyph_z[6] = {0x61, 0x51, 0x49, 0x45, 0x43, 0x00};
    static const uint8_t glyph_b[6] = {0x7F, 0x49, 0x49, 0x49, 0x36, 0x00};
    static const uint8_t glyph_f[6] = {0x7F, 0x09, 0x09, 0x09, 0x01, 0x00};
    static const uint8_t glyph_j[6] = {0x20, 0x40, 0x41, 0x3F, 0x01, 0x00};
    static const uint8_t glyph_k[6] = {0x7F, 0x08, 0x14, 0x22, 0x41, 0x00};
    static const uint8_t glyph_m[6] = {0x7F, 0x02, 0x0C, 0x02, 0x7F, 0x00};
    static const uint8_t glyph_n[6] = {0x7F, 0x04, 0x08, 0x10, 0x7F, 0x00};
    static const uint8_t glyph_q[6] = {0x3E, 0x41, 0x51, 0x21, 0x5E, 0x00};
    static const uint8_t glyph_s[6] = {0x26, 0x49, 0x49, 0x49, 0x32, 0x00};
    static const uint8_t glyph_u[6] = {0x3F, 0x40, 0x40, 0x40, 0x3F, 0x00};
    static const uint8_t glyph_v[6] = {0x1F, 0x20, 0x40, 0x20, 0x1F, 0x00};
    static const uint8_t glyph_lower[26][6] = {
        {0x20, 0x54, 0x54, 0x54, 0x78, 0x00}, /* a */
        {0x7F, 0x48, 0x44, 0x44, 0x38, 0x00}, /* b */
        {0x38, 0x44, 0x44, 0x44, 0x20, 0x00}, /* c */
        {0x38, 0x44, 0x44, 0x48, 0x7F, 0x00}, /* d */
        {0x38, 0x54, 0x54, 0x54, 0x18, 0x00}, /* e */
        {0x08, 0x7E, 0x09, 0x01, 0x02, 0x00}, /* f */
        {0x0C, 0x52, 0x52, 0x52, 0x3E, 0x00}, /* g */
        {0x7F, 0x08, 0x04, 0x04, 0x78, 0x00}, /* h */
        {0x00, 0x44, 0x7D, 0x40, 0x00, 0x00}, /* i */
        {0x20, 0x40, 0x44, 0x3D, 0x00, 0x00}, /* j */
        {0x7F, 0x10, 0x28, 0x44, 0x00, 0x00}, /* k */
        {0x00, 0x41, 0x7F, 0x40, 0x00, 0x00}, /* l */
        {0x7C, 0x04, 0x18, 0x04, 0x78, 0x00}, /* m */
        {0x7C, 0x08, 0x04, 0x04, 0x78, 0x00}, /* n */
        {0x38, 0x44, 0x44, 0x44, 0x38, 0x00}, /* o */
        {0x7C, 0x14, 0x14, 0x14, 0x08, 0x00}, /* p */
        {0x08, 0x14, 0x14, 0x18, 0x7C, 0x00}, /* q */
        {0x7C, 0x08, 0x04, 0x04, 0x08, 0x00}, /* r */
        {0x48, 0x54, 0x54, 0x54, 0x20, 0x00}, /* s */
        {0x04, 0x3F, 0x44, 0x40, 0x20, 0x00}, /* t */
        {0x3C, 0x40, 0x40, 0x20, 0x7C, 0x00}, /* u */
        {0x1C, 0x20, 0x40, 0x20, 0x1C, 0x00}, /* v */
        {0x3C, 0x40, 0x30, 0x40, 0x3C, 0x00}, /* w */
        {0x44, 0x28, 0x10, 0x28, 0x44, 0x00}, /* x */
        {0x0C, 0x50, 0x50, 0x50, 0x3C, 0x00}, /* y */
        {0x44, 0x64, 0x54, 0x4C, 0x44, 0x00}  /* z */
    };
    static const uint8_t glyph_digits[10][6] = {
        {0x3E, 0x51, 0x49, 0x45, 0x3E, 0x00},
        {0x00, 0x42, 0x7F, 0x40, 0x00, 0x00},
        {0x42, 0x61, 0x51, 0x49, 0x46, 0x00},
        {0x21, 0x41, 0x45, 0x4B, 0x31, 0x00},
        {0x18, 0x14, 0x12, 0x7F, 0x10, 0x00},
        {0x27, 0x45, 0x45, 0x45, 0x39, 0x00},
        {0x3C, 0x4A, 0x49, 0x49, 0x30, 0x00},
        {0x01, 0x71, 0x09, 0x05, 0x03, 0x00},
        {0x36, 0x49, 0x49, 0x49, 0x36, 0x00},
        {0x06, 0x49, 0x49, 0x29, 0x1E, 0x00}
    };

    if ((character >= '0') && (character <= '9')) {
        return glyph_digits[(uint8_t)(character - '0')];
    }
    if ((character >= 'a') && (character <= 'z')) {
        return glyph_lower[(uint8_t)(character - 'a')];
    }
    if (character == 'P') {
        return glyph_p;
    }
    if (character == 'R') {
        return glyph_r;
    }
    if (character == 'A') {
        return glyph_a;
    }
    if (character == 'C') {
        return glyph_c;
    }
    if (character == 'D') {
        return glyph_d;
    }
    if (character == 'E') {
        return glyph_e;
    }
    if (character == 'G') {
        return glyph_g;
    }
    if (character == 'I') {
        return glyph_i;
    }
    if (character == 'H') {
        return glyph_h;
    }
    if (character == 'L') {
        return glyph_l;
    }
    if (character == 'O') {
        return glyph_o;
    }
    if (character == 'T') {
        return glyph_t;
    }
    if (character == 'W') {
        return glyph_w;
    }
    if (character == 'X') {
        return glyph_x;
    }
    if (character == 'Y') {
        return glyph_y;
    }
    if (character == 'Z') {
        return glyph_z;
    }
    if (character == 'B') {
        return glyph_b;
    }
    if (character == 'F') {
        return glyph_f;
    }
    if (character == 'J') {
        return glyph_j;
    }
    if (character == 'K') {
        return glyph_k;
    }
    if (character == 'M') {
        return glyph_m;
    }
    if (character == 'N') {
        return glyph_n;
    }
    if (character == 'Q') {
        return glyph_q;
    }
    if (character == 'S') {
        return glyph_s;
    }
    if (character == 'U') {
        return glyph_u;
    }
    if (character == 'V') {
        return glyph_v;
    }
    if (character == '-') {
        return glyph_minus;
    }
    if (character == '.') {
        return glyph_dot;
    }
    if (character == ':') {
        return glyph_colon;
    }
    return glyph_space;
}

void OLED_DisplayOn(void)
{
    OLED_WriteCommand(0x8D);
    OLED_WriteCommand(0x14);
    OLED_WriteCommand(0xAF);
}

void OLED_DisplayOff(void)
{
    OLED_WriteCommand(0xAE);
    OLED_WriteCommand(0x8D);
    OLED_WriteCommand(0x10);
}

void OLED_Fill(uint8_t data)
{
    for (uint8_t page = 0; page < 8; ++page) {
        OLED_WriteCommand((uint8_t)(0xB0U + page));
        OLED_WriteCommand(0x00);
        OLED_WriteCommand(0x10);

        for (uint8_t column = 0; column < 128; ++column) {
            OLED_WriteData(data);
        }
    }
}

void OLED_Clear(void)
{
    OLED_Fill(0x00);
}

void OLED_ShowChar(uint8_t x, uint8_t page, char character)
{
    const uint8_t *glyph;

    if ((x > 122U) || (page > 7U)) {
        return;
    }

    glyph = OLED_GetGlyph(character);
    OLED_SetCursor(x, page);
    for (uint8_t column = 0; column < 6U; ++column) {
        OLED_WriteData(glyph[column]);
    }
}

void OLED_ShowString(uint8_t x, uint8_t page, const char *text)
{
    if (text == NULL) {
        return;
    }

    while ((*text != '\0') && (x <= 122U)) {
        OLED_ShowChar(x, page, *text);
        x = (uint8_t)(x + 6U);
        ++text;
    }
}

void OLED_ShowInt16(uint8_t x, uint8_t page, int16_t value)
{
    char text[7] = {' ', ' ', ' ', ' ', ' ', ' ', '\0'};
    uint16_t magnitude;
    int8_t position = 5;

    if (value < 0) {
        magnitude = (uint16_t)(-(int32_t)value);
    } else {
        magnitude = (uint16_t)value;
    }

    do {
        text[position] = (char)('0' + (magnitude % 10U));
        magnitude /= 10U;
        --position;
    } while ((magnitude != 0U) && (position >= 0));

    if ((value < 0) && (position >= 0)) {
        text[position] = '-';
    }

    OLED_ShowString(x, page, text);
}

void OLED_ShowAngle(uint8_t x, uint8_t page, int16_t degrees_x10)
{
    char text[7] = {' ', ' ', ' ', '0', '.', '0', '\0'};
    uint16_t magnitude;
    uint16_t whole_degrees;

    if (degrees_x10 < 0) {
        magnitude = (uint16_t)(-(int32_t)degrees_x10);
        text[0] = '-';
    } else {
        magnitude = (uint16_t)degrees_x10;
    }

    whole_degrees = magnitude / 10U;
    text[5] = (char)('0' + (magnitude % 10U));
    text[3] = (char)('0' + (whole_degrees % 10U));
    if (whole_degrees >= 10U) {
        text[2] = (char)('0' + ((whole_degrees / 10U) % 10U));
    }
    if (whole_degrees >= 100U) {
        text[1] = (char)('0' + ((whole_degrees / 100U) % 10U));
    }

    OLED_ShowString(x, page, text);
    OLED_SetCursor((uint8_t)(x + 38U), page);
    OLED_WriteData(0x06U);
    OLED_WriteData(0x09U);
    OLED_WriteData(0x09U);
    OLED_WriteData(0x06U);
    OLED_WriteData(0x00U);
}

void OLED_Init(void)
{
    OLED_GPIO_Init();

    OLED_RST_LOW();
    HAL_Delay(20);
    OLED_RST_HIGH();
    HAL_Delay(20);

    OLED_WriteCommand(0xAE); /* Display off. */
    OLED_WriteCommand(0xD5); OLED_WriteCommand(0x80);
    OLED_WriteCommand(0xA8); OLED_WriteCommand(0x3F);
    OLED_WriteCommand(0xD3); OLED_WriteCommand(0x00);
    OLED_WriteCommand(0x40);
    OLED_WriteCommand(0x8D); OLED_WriteCommand(0x14);
    OLED_WriteCommand(0x20); OLED_WriteCommand(0x02); /* Page addressing mode. */
    OLED_WriteCommand(0xA1);
    OLED_WriteCommand(0xC8);
    OLED_WriteCommand(0xDA); OLED_WriteCommand(0x12);
    OLED_WriteCommand(0x81); OLED_WriteCommand(0xCF);
    OLED_WriteCommand(0xD9); OLED_WriteCommand(0xF1);
    OLED_WriteCommand(0xDB); OLED_WriteCommand(0x40);
    OLED_WriteCommand(0xA4);
    OLED_WriteCommand(0xA6);
    OLED_WriteCommand(0x2E);

    OLED_Clear();
    OLED_DisplayOn();
}
