#ifndef I2C_H
#define I2C_H

#include <stdint.h>
#include "stm32f4xx_conf.h"
#include <stm32f4xx_i2c.h>


void I2C_write(I2C_TypeDef* I2Cx, uint8_t data);
uint8_t I2C_read_ack(I2C_TypeDef* I2Cx);
uint8_t I2C_read_nack(I2C_TypeDef* I2Cx);
void I2C_stop(I2C_TypeDef* I2Cx);
void I2C_Init1(void);
void I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction);

#endif
