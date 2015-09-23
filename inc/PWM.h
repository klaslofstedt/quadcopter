#ifndef PWM_h
#define PWM_h

#define PWM_FREQUENCY 400

void PWM_GPIO_Init(void);
void PWM_TimeBase_Init(void);
void PWM_OutputCompare_Init(void);
uint16_t GetPrescaler(void);

#endif
