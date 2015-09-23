#include <stdint.h>

#include "stm32f4xx_conf.h"

#include "ESC.h"
#include "PWM.h"
#include "delay.h"

void ESC_SetSpeed(ESC_t *esc)
{
	PWM_SetDutyCycle(esc->pin_number, esc->run_min + (uint16_t)(esc->speed * (esc->run_max - esc->run_min)));
	/*switch(esc->pin_number){
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
	}*/
}

void ESC_Init(ESC_t *esc, uint16_t pin)
{
	uint16_t Prescaler = GetPrescaler();
	esc->pin_number = pin;
	esc->speed = 0;
	// devided by 1000000 because ESC data is in the base of us instead of s
	esc->run_min = (((SystemCoreClock/1000000)/ (2 * Prescaler)) * (ESC_INIT_HIGH));
	Delay(500000);
	ESC_SetSpeed(esc);
	Delay(2500000);

	esc->run_min = (((SystemCoreClock/1000000)/ (2 * Prescaler)) * (ESC_RUN_MIN));
	esc->run_max = (((SystemCoreClock/1000000)/ (2 * Prescaler)) * (ESC_RUN_MAX));
	esc->lift_quad_min = 0;
}

void CheckBondaries(ESC_t *esc)
{
	if(esc->speed > 1.0){
		esc->speed = 1.0;
	}else if(esc->speed < 0){
		esc->speed = 0;
	}
}
