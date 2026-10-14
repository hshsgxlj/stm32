#ifndef __DMP_PORT_H
#define __DMP_PORT_H

#include <stdint.h>

int i2cWrite(uint8_t address, uint8_t reg, uint8_t length,
             const uint8_t *data);
int i2cRead(uint8_t address, uint8_t reg, uint8_t length, uint8_t *data);
void delay_ms(unsigned long milliseconds);
void dmp_get_ms(unsigned long *milliseconds);
void myget_ms(unsigned long *milliseconds);

#endif /* __DMP_PORT_H */
