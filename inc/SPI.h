#include <stm32f4xx_spi.h>

#ifndef SPI_H
#define SPI_H


uint8_t SPI_read(uint8_t adress);
void SPI_write(uint8_t adress, uint8_t data);
void SPI1_Init(uint16_t Prescaler);

#endif
