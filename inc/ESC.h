#ifndef ESC_h
#define ESC_h

#include "quadcopter_structures.h"

#define ESC_INIT_LOW 0
#define ESC_INIT_HIGH 850
#define ESC_THRUST 0
#define ESC_RUN_MIN 1173
#define ESC_RUN_MAX 1860

typedef struct{
	uint16_t init_low;
	uint16_t init_high;
	uint16_t run_min;
	uint16_t run_max;
	uint16_t lift_quad_min;
	uint8_t pin_number;
	float speed; // from 0 to 1, with a resolution of ~14 bits (14280 values)
} ESC_t;

void ESC_SetSpeed(ESC_t *esc);
void ESC_Init(ESC_t *esc, uint16_t pin);
void CheckBondaries(ESC_t *esc);

#endif
