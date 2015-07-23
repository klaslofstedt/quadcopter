#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#include "stm32f4xx_conf.h"
#include "stm32f4xx.h"
#include "main.h"

#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
#include "usbd_cdc_vcp.h"

// Variables used in systick
volatile uint32_t time_var1, time_var2;
// Variables used for USB
__ALIGN_BEGIN USB_OTG_CORE_HANDLE  USB_OTG_dev __ALIGN_END;

#define ESC_INIT_LOW 0
#define ESC_INIT_HIGH 850
#define ESC_THRUST 0
#define ESC_RUN_MIN 1180
#define ESC_RUN_MAX 1860

#define PWM_FREQUENCY 400

// Private function prototypes
//void Delay(volatile uint32_t nCount);
//void init();
//void calculation_test();

typedef struct{
    uint16_t init_low;
    uint16_t init_high;
    uint16_t run_min;
    uint16_t run_max;
    uint16_t lift_quad_min;
    uint16_t pin_number;
    float speed; // from 0 to 1, with a reolution of ~14 bits (14280 values)
} ESC_t;

/******************************** Functions ***********************************/
uint16_t GetPrescaler(void);


void PWM_GPIO_Init()
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

void ESC_SetSpeed(ESC_t *esc)
{
    switch(esc->pin_number){
        case 12:
        TIM4->CCR1 = esc->run_min + (uint16_t)(esc->speed * (esc->run_max - esc->run_min));
        break;

        case 13:
        TIM4->CCR2 = esc->run_min + (uint16_t)(esc->speed * (esc->run_max - esc->run_min));
        break;

        case 14:
        TIM4->CCR3 = esc->run_min + (uint16_t)(esc->speed * (esc->run_max - esc->run_min));
        break;

        case 15:
        TIM4->CCR4 = esc->run_min + (uint16_t)(esc->speed * (esc->run_max - esc->run_min));
        break;

        default:
        // do nothing
        break;
    }
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

void ESC_Init(ESC_t *esc, uint16_t pin)
{
    uint16_t Prescaler = GetPrescaler();
    esc->pin_number = pin;
    esc->speed = 0;
    // devided by 1000000 because ESC data is in the base of us instead of s
    esc->run_min = (((SystemCoreClock/1000000)/ (2 * Prescaler)) * (ESC_INIT_HIGH));
    Delay(500);
    ESC_SetSpeed(esc);
    Delay(2500);

    esc->run_min = (((SystemCoreClock/1000000)/ (2 * Prescaler)) * (ESC_RUN_MIN));
    esc->run_max = (((SystemCoreClock/1000000)/ (2 * Prescaler)) * (ESC_RUN_MAX));
    esc->lift_quad_min = 0;
}

void USB_Init(void)
{
    USBD_Init(	&USB_OTG_dev,
        USB_OTG_FS_CORE_ID,
        &USR_desc,
        &USBD_CDC_cb,
        &USR_cb);
    /*
    * Disable STDOUT buffering. Otherwise nothing will be printed
    * before a newline character or when the buffer is flushed.
    */
    setbuf(stdout, NULL);
}

int main(void)
{
    USB_Init();
    SystemCoreClockUpdate(); // Get Core Clock Frequency

    if (SysTick_Config(SystemCoreClock / 1000)) { //SysTick 1 msec interrupts
        while (1); //Capture error
    }
    Delay(500);
    PWM_GPIO_Init();
    PWM_TimeBase_Init();
    PWM_OutputCompare_Init();
    ESC_t esc1, esc2, esc3, esc4;
    ESC_Init(&esc1, 12);
    ESC_Init(&esc2, 13);
    ESC_Init(&esc3, 14);
    ESC_Init(&esc4, 15);
    Delay(1500);

    for(;;) {
        esc1.speed = 0.5;
        ESC_SetSpeed(&esc1);
        esc2.speed = 0.5;
        ESC_SetSpeed(&esc2);
        esc3.speed = 0.5;
        ESC_SetSpeed(&esc3);
        esc4.speed = 0.5;
        ESC_SetSpeed(&esc4);
    }
    return 0;
}

//Dummy function to avoid compiler error
void _init()
{

}
