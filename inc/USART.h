#ifndef USART_H
#define USART_H

#include <stdint.h>
#include "stm32f4xx_conf.h"
#include <stm32f4xx_i2c.h>

extern uint8_t USART1_RxByte;

void USART_Init1(void);
void USART1_IRQHandler(void);
void USART1_SendByte(volatile uint8_t s);
void USART1_SendString(volatile char *s);


#endif
