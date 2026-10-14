#include "mpu6050.h"
#include <math.h>

#define MPU6050_ADDRESS_LOW       0x68U
#define MPU6050_ADDRESS_HIGH      0x69U
#define MPU6050_REG_SMPLRT_DIV    0x19U
#define MPU6050_REG_CONFIG        0x1AU
#define MPU6050_REG_GYRO_CONFIG   0x1BU
#define MPU6050_REG_ACCEL_CONFIG  0x1CU
#define MPU6050_REG_ACCEL_XOUT_H  0x3BU
#define MPU6050_REG_PWR_MGMT_1    0x6BU
#define MPU6050_REG_WHO_AM_I      0x75U

#define MPU6050_ACCEL_LSB_PER_G    16384L
#define MPU6050_GYRO_LSB_PER_DPS   131L
#define MPU6050_FILTER_DIVISOR      8L
#define MPU6050_FILTER_SCALE      256L
#define MPU6050_GYRO_DEAD_BAND     30L

#define MPU6050_SCL_LOW()  HAL_GPIO_WritePin(MPU6050_SCL_PORT, MPU6050_SCL_PIN, GPIO_PIN_RESET)
#define MPU6050_SCL_HIGH() HAL_GPIO_WritePin(MPU6050_SCL_PORT, MPU6050_SCL_PIN, GPIO_PIN_SET)
#define MPU6050_SDA_LOW()  HAL_GPIO_WritePin(MPU6050_SDA_PORT, MPU6050_SDA_PIN, GPIO_PIN_RESET)
#define MPU6050_SDA_HIGH() HAL_GPIO_WritePin(MPU6050_SDA_PORT, MPU6050_SDA_PIN, GPIO_PIN_SET)
#define MPU6050_SDA_READ() HAL_GPIO_ReadPin(MPU6050_SDA_PORT, MPU6050_SDA_PIN)

static uint8_t mpu6050_address = MPU6050_ADDRESS_LOW;
static int32_t accel_offset_x;
static int32_t accel_offset_y;
static int32_t accel_offset_z;
static int32_t gyro_offset_x;
static int32_t gyro_offset_y;
static int32_t gyro_offset_z;
static int32_t accel_filter_x;
static int32_t accel_filter_y;
static int32_t accel_filter_z;
static int32_t gyro_filter_x;
static int32_t gyro_filter_y;
static int32_t gyro_filter_z;
static uint8_t filter_initialized;
static float attitude_pitch;
static float attitude_roll;
static float attitude_yaw;
static uint32_t attitude_last_tick;
static uint8_t attitude_initialized;

static void MPU6050_I2C_Delay(void)
{
    for (volatile uint32_t count = 0; count < 20U; ++count) {
        __NOP();
    }
}

static void MPU6050_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    MPU6050_SCL_HIGH();
    MPU6050_SDA_HIGH();

    gpio.Pin = MPU6050_SCL_PIN | MPU6050_SDA_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);

    MPU6050_SCL_HIGH();
    MPU6050_SDA_HIGH();
}

void MPU6050_BusInit(void)
{
    MPU6050_GPIO_Init();
}

static void MPU6050_I2C_Start(void)
{
    MPU6050_SDA_HIGH();
    MPU6050_SCL_HIGH();
    MPU6050_I2C_Delay();
    MPU6050_SDA_LOW();
    MPU6050_I2C_Delay();
    MPU6050_SCL_LOW();
}

static void MPU6050_I2C_Stop(void)
{
    MPU6050_SDA_LOW();
    MPU6050_I2C_Delay();
    MPU6050_SCL_HIGH();
    MPU6050_I2C_Delay();
    MPU6050_SDA_HIGH();
    MPU6050_I2C_Delay();
}

static uint8_t MPU6050_I2C_WriteByte(uint8_t value)
{
    for (uint8_t bit = 0; bit < 8U; ++bit) {
        MPU6050_SCL_LOW();
        if ((value & 0x80U) != 0U) {
            MPU6050_SDA_HIGH();
        } else {
            MPU6050_SDA_LOW();
        }
        MPU6050_I2C_Delay();
        MPU6050_SCL_HIGH();
        MPU6050_I2C_Delay();
        value <<= 1;
    }

    MPU6050_SCL_LOW();
    MPU6050_SDA_HIGH();
    MPU6050_I2C_Delay();
    MPU6050_SCL_HIGH();
    MPU6050_I2C_Delay();

    uint8_t acknowledged = (MPU6050_SDA_READ() == GPIO_PIN_RESET) ? 1U : 0U;
    MPU6050_SCL_LOW();
    return acknowledged;
}

static uint8_t MPU6050_I2C_ReadByte(uint8_t send_ack)
{
    uint8_t value = 0U;

    MPU6050_SDA_HIGH();
    for (uint8_t bit = 0; bit < 8U; ++bit) {
        value <<= 1;
        MPU6050_SCL_LOW();
        MPU6050_I2C_Delay();
        MPU6050_SCL_HIGH();
        MPU6050_I2C_Delay();
        if (MPU6050_SDA_READ() == GPIO_PIN_SET) {
            value |= 0x01U;
        }
    }

    MPU6050_SCL_LOW();
    if (send_ack != 0U) {
        MPU6050_SDA_LOW();
    } else {
        MPU6050_SDA_HIGH();
    }
    MPU6050_I2C_Delay();
    MPU6050_SCL_HIGH();
    MPU6050_I2C_Delay();
    MPU6050_SCL_LOW();
    MPU6050_SDA_HIGH();

    return value;
}

static HAL_StatusTypeDef MPU6050_WriteRegister(uint8_t reg, uint8_t value)
{
    MPU6050_I2C_Start();
    if ((MPU6050_I2C_WriteByte((uint8_t)(mpu6050_address << 1)) == 0U) ||
        (MPU6050_I2C_WriteByte(reg) == 0U) ||
        (MPU6050_I2C_WriteByte(value) == 0U)) {
        MPU6050_I2C_Stop();
        return HAL_ERROR;
    }
    MPU6050_I2C_Stop();
    return HAL_OK;
}

static HAL_StatusTypeDef MPU6050_ReadRegisters(uint8_t reg, uint8_t *data, uint8_t length)
{
    if ((data == NULL) || (length == 0U)) {
        return HAL_ERROR;
    }

    MPU6050_I2C_Start();
    if ((MPU6050_I2C_WriteByte((uint8_t)(mpu6050_address << 1)) == 0U) ||
        (MPU6050_I2C_WriteByte(reg) == 0U)) {
        MPU6050_I2C_Stop();
        return HAL_ERROR;
    }

    MPU6050_I2C_Start();
    if (MPU6050_I2C_WriteByte((uint8_t)((mpu6050_address << 1) | 0x01U)) == 0U) {
        MPU6050_I2C_Stop();
        return HAL_ERROR;
    }

    for (uint8_t index = 0; index < length; ++index) {
        data[index] = MPU6050_I2C_ReadByte((index + 1U < length) ? 1U : 0U);
    }
    MPU6050_I2C_Stop();
    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_BusWrite(uint8_t address, uint8_t reg,
                                  const uint8_t *data, uint8_t length)
{
    if ((data == NULL) || (length == 0U)) {
        return HAL_ERROR;
    }

    MPU6050_I2C_Start();
    if ((MPU6050_I2C_WriteByte((uint8_t)(address << 1)) == 0U) ||
        (MPU6050_I2C_WriteByte(reg) == 0U)) {
        MPU6050_I2C_Stop();
        return HAL_ERROR;
    }

    for (uint8_t index = 0; index < length; ++index) {
        if (MPU6050_I2C_WriteByte(data[index]) == 0U) {
            MPU6050_I2C_Stop();
            return HAL_ERROR;
        }
    }

    MPU6050_I2C_Stop();
    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_BusRead(uint8_t address, uint8_t reg,
                                 uint8_t *data, uint8_t length)
{
    if ((data == NULL) || (length == 0U)) {
        return HAL_ERROR;
    }

    MPU6050_I2C_Start();
    if ((MPU6050_I2C_WriteByte((uint8_t)(address << 1)) == 0U) ||
        (MPU6050_I2C_WriteByte(reg) == 0U)) {
        MPU6050_I2C_Stop();
        return HAL_ERROR;
    }

    MPU6050_I2C_Start();
    if (MPU6050_I2C_WriteByte((uint8_t)((address << 1) | 0x01U)) == 0U) {
        MPU6050_I2C_Stop();
        return HAL_ERROR;
    }

    for (uint8_t index = 0; index < length; ++index) {
        data[index] = MPU6050_I2C_ReadByte((index + 1U < length) ? 1U : 0U);
    }
    MPU6050_I2C_Stop();
    return HAL_OK;
}

static HAL_StatusTypeDef MPU6050_CheckAddress(uint8_t address)
{
    uint8_t who_am_i = 0U;

    mpu6050_address = address;
    if (MPU6050_ReadRegisters(MPU6050_REG_WHO_AM_I, &who_am_i, 1U) != HAL_OK) {
        return HAL_ERROR;
    }
    return ((who_am_i & 0x7EU) == 0x68U) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef MPU6050_Init(void)
{
    MPU6050_GPIO_Init();
    HAL_Delay(100);

    if ((MPU6050_CheckAddress(MPU6050_ADDRESS_LOW) != HAL_OK) &&
        (MPU6050_CheckAddress(MPU6050_ADDRESS_HIGH) != HAL_OK)) {
        return HAL_ERROR;
    }

    if (MPU6050_WriteRegister(MPU6050_REG_PWR_MGMT_1, 0x80U) != HAL_OK) {
        return HAL_ERROR;
    }
    HAL_Delay(100);

    if ((MPU6050_WriteRegister(MPU6050_REG_PWR_MGMT_1, 0x01U) != HAL_OK) ||
        (MPU6050_WriteRegister(MPU6050_REG_SMPLRT_DIV, 0x09U) != HAL_OK) ||
        (MPU6050_WriteRegister(MPU6050_REG_CONFIG, 0x04U) != HAL_OK) ||
        (MPU6050_WriteRegister(MPU6050_REG_GYRO_CONFIG, 0x00U) != HAL_OK) ||
        (MPU6050_WriteRegister(MPU6050_REG_ACCEL_CONFIG, 0x00U) != HAL_OK)) {
        return HAL_ERROR;
    }

    accel_offset_x = 0;
    accel_offset_y = 0;
    accel_offset_z = 0;
    gyro_offset_x = 0;
    gyro_offset_y = 0;
    gyro_offset_z = 0;
    filter_initialized = 0U;
    attitude_pitch = 0.0f;
    attitude_roll = 0.0f;
    attitude_yaw = 0.0f;
    attitude_last_tick = HAL_GetTick();
    attitude_initialized = 0U;

    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_ReadData(MPU6050_DataTypeDef *data)
{
    uint8_t raw[14];

    if (data == NULL) {
        return HAL_ERROR;
    }
    if (MPU6050_ReadRegisters(MPU6050_REG_ACCEL_XOUT_H, raw, sizeof(raw)) != HAL_OK) {
        return HAL_ERROR;
    }

    data->accel_x = (int16_t)(((uint16_t)raw[0] << 8) | raw[1]);
    data->accel_y = (int16_t)(((uint16_t)raw[2] << 8) | raw[3]);
    data->accel_z = (int16_t)(((uint16_t)raw[4] << 8) | raw[5]);
    data->gyro_x = (int16_t)(((uint16_t)raw[8] << 8) | raw[9]);
    data->gyro_y = (int16_t)(((uint16_t)raw[10] << 8) | raw[11]);
    data->gyro_z = (int16_t)(((uint16_t)raw[12] << 8) | raw[13]);

    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_Calibrate(uint16_t sample_count)
{
    MPU6050_DataTypeDef raw;
    int32_t accel_sum_x = 0;
    int32_t accel_sum_y = 0;
    int32_t accel_sum_z = 0;
    int32_t gyro_sum_x = 0;
    int32_t gyro_sum_y = 0;
    int32_t gyro_sum_z = 0;

    if ((sample_count == 0U) || (sample_count > 1000U)) {
        return HAL_ERROR;
    }

    for (uint16_t sample = 0; sample < sample_count; ++sample) {
        if (MPU6050_ReadData(&raw) != HAL_OK) {
            return HAL_ERROR;
        }

        accel_sum_x += raw.accel_x;
        accel_sum_y += raw.accel_y;
        accel_sum_z += raw.accel_z;
        gyro_sum_x += raw.gyro_x;
        gyro_sum_y += raw.gyro_y;
        gyro_sum_z += raw.gyro_z;
        HAL_Delay(10);
    }

    accel_offset_x = accel_sum_x / (int32_t)sample_count;
    accel_offset_y = accel_sum_y / (int32_t)sample_count;
    accel_offset_z = accel_sum_z / (int32_t)sample_count;
    if (accel_offset_z >= 0) {
        accel_offset_z -= MPU6050_ACCEL_LSB_PER_G;
    } else {
        accel_offset_z += MPU6050_ACCEL_LSB_PER_G;
    }

    gyro_offset_x = gyro_sum_x / (int32_t)sample_count;
    gyro_offset_y = gyro_sum_y / (int32_t)sample_count;
    gyro_offset_z = gyro_sum_z / (int32_t)sample_count;
    filter_initialized = 0U;

    return HAL_OK;
}

static int32_t MPU6050_FilterUpdate(int32_t previous, int32_t input)
{
    int32_t target = input * MPU6050_FILTER_SCALE;
    return previous + ((target - previous) / MPU6050_FILTER_DIVISOR);
}

static int32_t MPU6050_ApplyGyroDeadBand(int32_t value)
{
    if ((value > -MPU6050_GYRO_DEAD_BAND) && (value < MPU6050_GYRO_DEAD_BAND)) {
        return 0;
    }
    return value;
}

HAL_StatusTypeDef MPU6050_ReadFiltered(MPU6050_FilteredDataTypeDef *data)
{
    MPU6050_DataTypeDef raw;
    int32_t accel_x;
    int32_t accel_y;
    int32_t accel_z;
    int32_t gyro_x;
    int32_t gyro_y;
    int32_t gyro_z;

    if (data == NULL) {
        return HAL_ERROR;
    }
    if (MPU6050_ReadData(&raw) != HAL_OK) {
        return HAL_ERROR;
    }

    accel_x = (int32_t)raw.accel_x - accel_offset_x;
    accel_y = (int32_t)raw.accel_y - accel_offset_y;
    accel_z = (int32_t)raw.accel_z - accel_offset_z;
    gyro_x = (int32_t)raw.gyro_x - gyro_offset_x;
    gyro_y = (int32_t)raw.gyro_y - gyro_offset_y;
    gyro_z = (int32_t)raw.gyro_z - gyro_offset_z;

    if (filter_initialized == 0U) {
        accel_filter_x = accel_x * MPU6050_FILTER_SCALE;
        accel_filter_y = accel_y * MPU6050_FILTER_SCALE;
        accel_filter_z = accel_z * MPU6050_FILTER_SCALE;
        gyro_filter_x = gyro_x * MPU6050_FILTER_SCALE;
        gyro_filter_y = gyro_y * MPU6050_FILTER_SCALE;
        gyro_filter_z = gyro_z * MPU6050_FILTER_SCALE;
        filter_initialized = 1U;
    } else {
        accel_filter_x = MPU6050_FilterUpdate(accel_filter_x, accel_x);
        accel_filter_y = MPU6050_FilterUpdate(accel_filter_y, accel_y);
        accel_filter_z = MPU6050_FilterUpdate(accel_filter_z, accel_z);
        gyro_filter_x = MPU6050_FilterUpdate(gyro_filter_x, gyro_x);
        gyro_filter_y = MPU6050_FilterUpdate(gyro_filter_y, gyro_y);
        gyro_filter_z = MPU6050_FilterUpdate(gyro_filter_z, gyro_z);
    }

    accel_x = accel_filter_x / MPU6050_FILTER_SCALE;
    accel_y = accel_filter_y / MPU6050_FILTER_SCALE;
    accel_z = accel_filter_z / MPU6050_FILTER_SCALE;
    gyro_x = MPU6050_ApplyGyroDeadBand(gyro_filter_x / MPU6050_FILTER_SCALE);
    gyro_y = MPU6050_ApplyGyroDeadBand(gyro_filter_y / MPU6050_FILTER_SCALE);
    gyro_z = MPU6050_ApplyGyroDeadBand(gyro_filter_z / MPU6050_FILTER_SCALE);

    data->accel_x_mg = (int16_t)((accel_x * 1000L) / MPU6050_ACCEL_LSB_PER_G);
    data->accel_y_mg = (int16_t)((accel_y * 1000L) / MPU6050_ACCEL_LSB_PER_G);
    data->accel_z_mg = (int16_t)((accel_z * 1000L) / MPU6050_ACCEL_LSB_PER_G);
    data->gyro_x_dps_x10 = (int16_t)((gyro_x * 10L) / MPU6050_GYRO_LSB_PER_DPS);
    data->gyro_y_dps_x10 = (int16_t)((gyro_y * 10L) / MPU6050_GYRO_LSB_PER_DPS);
    data->gyro_z_dps_x10 = (int16_t)((gyro_z * 10L) / MPU6050_GYRO_LSB_PER_DPS);

    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_ReadAttitude(MPU6050_AttitudeTypeDef *attitude)
{
    MPU6050_DataTypeDef raw;
    float ax;
    float ay;
    float az;
    float accel_pitch;
    float accel_roll;
    float gyro_pitch_rate;
    float gyro_roll_rate;
    float gyro_yaw_rate;
    float dt;
    uint32_t now;

    if (attitude == NULL) {
        return HAL_ERROR;
    }
    if (MPU6050_ReadData(&raw) != HAL_OK) {
        return HAL_ERROR;
    }

    ax = ((float)raw.accel_x - (float)accel_offset_x) / (float)MPU6050_ACCEL_LSB_PER_G;
    ay = ((float)raw.accel_y - (float)accel_offset_y) / (float)MPU6050_ACCEL_LSB_PER_G;
    az = ((float)raw.accel_z - (float)accel_offset_z) / (float)MPU6050_ACCEL_LSB_PER_G;

    accel_roll = atan2f(ay, az) * 57.2957795f;
    accel_pitch = atan2f(-ax, sqrtf((ay * ay) + (az * az))) * 57.2957795f;
    gyro_roll_rate = ((float)raw.gyro_x - (float)gyro_offset_x) / (float)MPU6050_GYRO_LSB_PER_DPS;
    gyro_pitch_rate = ((float)raw.gyro_y - (float)gyro_offset_y) / (float)MPU6050_GYRO_LSB_PER_DPS;
    gyro_yaw_rate = ((float)raw.gyro_z - (float)gyro_offset_z) / (float)MPU6050_GYRO_LSB_PER_DPS;
    /* yaw 死区:低于 0.2°/s 视为零偏残余,不积分——防止 Y 缓慢爬行
     * (真实转向远大于此值,不受影响) */
    if ((gyro_yaw_rate > -0.2f) && (gyro_yaw_rate < 0.2f)) {
        gyro_yaw_rate = 0.0f;
    }

    now = HAL_GetTick();
    dt = (float)(now - attitude_last_tick) / 1000.0f;
    attitude_last_tick = now;
    if ((dt <= 0.0f) || (dt > 0.2f)) {
        dt = 0.02f;
    }

    if (attitude_initialized == 0U) {
        attitude_pitch = accel_pitch;
        attitude_roll = accel_roll;
        attitude_yaw = 0.0f;
        attitude_initialized = 1U;
    } else {
        /* Complementary filter: gyro responds quickly, accel removes drift. */
        attitude_pitch = (0.98f * (attitude_pitch + gyro_pitch_rate * dt)) + (0.02f * accel_pitch);
        attitude_roll = (0.98f * (attitude_roll + gyro_roll_rate * dt)) + (0.02f * accel_roll);
        attitude_yaw += gyro_yaw_rate * dt;
        if (attitude_yaw > 180.0f) {
            attitude_yaw -= 360.0f;
        } else if (attitude_yaw < -180.0f) {
            attitude_yaw += 360.0f;
        }
    }

    attitude->pitch_deg_x10 = (int16_t)(attitude_pitch * 10.0f);
    attitude->roll_deg_x10 = (int16_t)(attitude_roll * 10.0f);
    attitude->yaw_deg_x10 = (int16_t)(attitude_yaw * 10.0f);
    return HAL_OK;
}
