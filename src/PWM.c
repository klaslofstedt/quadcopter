#include <stdint.h>
#include <math.h>

#include "stm32f4xx_conf.h"
#include "PWM.h"

void PWM_SetDutyCycle(uint8_t PinNum, uint16_t DutyCycle)
{
  switch(PinNum){
    case 12:
    TIM4->CCR1 = DutyCycle;
    break;

    case 13:
    TIM4->CCR2 = DutyCycle;
    break;

    case 14:
    TIM4->CCR3 = DutyCycle;
    break;

    case 15:
    TIM4->CCR4 = DutyCycle;
    break;

    default:
    // do nothing
    break;
  }
}

void PWM_GPIO_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOD, &GPIO_InitStruct);

  GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);
}

void PWM_TimeBase_Init(void)
{
  TIM_TimeBaseInitTypeDef TIM_BaseStruct;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

  uint16_t Prescaler = GetPrescaler();
  TIM_BaseStruct.TIM_Prescaler = Prescaler - 1;
  TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_BaseStruct.TIM_Period = (((SystemCoreClock / 2) / PWM_FREQUENCY) / Prescaler) -1;
  TIM_BaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_BaseStruct.TIM_RepetitionCounter = 0x0000;
  TIM_TimeBaseInit(TIM4, &TIM_BaseStruct);
  TIM_Cmd(TIM4, ENABLE);
}

void PWM_OutputCompare_Init(void)
{
  TIM_OCInitTypeDef TIM_OCStruct;

  TIM_OCStruct.TIM_Pulse = 0;
  TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCStruct.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM4, &TIM_OCStruct);
  TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

  TIM_OC2Init(TIM4, &TIM_OCStruct);
  TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);

  TIM_OC3Init(TIM4, &TIM_OCStruct);
  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

  TIM_OC4Init(TIM4, &TIM_OCStruct);
  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM4, ENABLE); // what is this?
  TIM_Cmd(TIM4, ENABLE);
}

uint16_t GetPrescaler(void)
{
  /*  To be able to get a value lower than 65535 for TIM_Period, we need to find
  a prescaler. It can be found by:

  SystemCoreClock / 2 / PWM_FREQUENCY / Prescaler < 65535
  */
  uint32_t Prescaler = 1;
  uint32_t TimerSize = pow(2, 8*sizeof(uint16_t));
  //uint32_t test = pow(2, 8*sizeof(PWM_TIMER));
  while((SystemCoreClock / (2 * PWM_FREQUENCY * Prescaler)) > TimerSize){
    Prescaler++;
  }
  return (uint16_t)Prescaler;
}
