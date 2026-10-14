#include "dmp_port.h"
#include "mpu6050.h"

int i2cWrite(uint8_t address, uint8_t reg, uint8_t length,
             const uint8_t *data)
{
    return (MPU6050_BusWrite(address, reg, data, length) == HAL_OK) ? 0 : 1;
}

int i2cRead(uint8_t address, uint8_t reg, uint8_t length, uint8_t *data)
{
    return (MPU6050_BusRead(address, reg, data, length) == HAL_OK) ? 0 : 1;
}

void delay_ms(unsigned long milliseconds)
{
    HAL_Delay((uint32_t)milliseconds);
}

void dmp_get_ms(unsigned long *milliseconds)
{
    if (milliseconds != NULL) {
        *milliseconds = (unsigned long)HAL_GetTick();
    }
}
