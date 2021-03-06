/* Quadcopter project by Klas Löfstedt */
// C specific
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
// "Default"
#include "stm32f4xx_conf.h"
#include "stm32f4xx.h"
// Project specific
#include "I2C.h"
#include "delay.h"
#include "MPU9250.h"
//#include "MPU6050.h"
#include "kalman.h"
#include "PWM.h"
#include "ESC.h"
#include "USB.h"
#include "USART.h"
//#include "SPI.h"
//#include "AK8963.h"
#include "quadcopter_structures.h"

#define USB_DEBUG

#define LOOP_FREQUENCY 300
#define LOOP_PERIOD_TIME_US 3333//1/(LOOP_FREQUENCY * 1e-6)
#define MAX_ANGLE 30

ESC_t esc1, esc2, esc3, esc4;
IMU_DATA_t mpu_acc1, mpu_gyr1, mpu_mag1;
PID_DATA_t mpu_roll1, mpu_pitch1, mpu_yaw1;
/******************************** Functions ***********************************/
//#ifdef USB_DEBUG
int count2 = 0;
void DisplayRaw(IMU_DATA_t* acc1, IMU_DATA_t* gyr1, IMU_DATA_t* mag1)
{
	if(count2 >= 100)
	{
		count2 = 0;
		printf(" gyr_rol: %.5f", gyr1->Roll);
		printf(" gyr_pit: %.5f\r\n", gyr1->Pitch);
		printf(" acc_rol: %.5f", acc1->Roll);
		printf(" acc_pit: %.5f\r\n", acc1->Pitch);
		printf(" mag_rol: %.5f", mag1->Roll);
		printf(" mag_pit: %.5f\r\n", mag1->Pitch);
	}
	count2++;
}

int count3 = 0;
void DisplayMag(IMU_DATA_t* mag)
{
	if(count3 >= 100)
	{
		count3 = 0;
		printf(" mag_rol: %.5f", mag->Roll);
		printf(" mag_pit: %.5f\r\n", mag->Pitch);
		printf(" mag_rol: %.5f1", mag->Yaw);
	}
	count3++;
}

int count = 0;
void DisplayFixed(PID_DATA_t* r, PID_DATA_t* p)
{
	if(count >= 100)
	{
		count = 0;
		printf(" rol_deg: %.5f", r->Degrees);
		printf(" rol_vel: %.5f\r\n", r->Velocity);
		printf(" pit_deg: %.5f", p->Degrees);
		printf(" pit_vel: %.5f\r\n", p->Velocity);
	}
	count++;
}
//#endif


void UpdateMotors(ESC_t* esc1, ESC_t* esc2, ESC_t* esc3, ESC_t* esc4, float roll, float pitch)
{
	float thrust = 0.2;//((float)USART1_RxByte) /100;
	/*I'm not sure about these formulas. should give this pattern:

	1  front  2
	left    right
	4   back  3 */

	esc2->speed = thrust - pitch + roll; // - yawValue;
	esc1->speed = thrust + pitch + roll; // + yawValue;
	esc4->speed = thrust + pitch - roll; // - yawValue;
	esc3->speed = thrust - pitch - roll; // + yawValue;

	CheckBondaries(esc1);
	CheckBondaries(esc2);
	CheckBondaries(esc3);
	CheckBondaries(esc4);

	ESC_SetSpeed(esc1);
	ESC_SetSpeed(esc2);
	ESC_SetSpeed(esc3);
	ESC_SetSpeed(esc4);
}

void PID_Init(PID_DATA_t *pid)
{
	pid->I_Term = 0;
	pid->SetPoint = 0;
}

void SystemClock_Init(void)
{
	SystemCoreClockUpdate(); // Get Core Clock Frequency
	if (SysTick_Config(SystemCoreClock / 1000000)) { //SysTick 1 msec interrupts
		while (1); //Capture error
	}
}

void System_Init(void)
{
	SystemClock_Init();
	USB_Init();
	PWM_Init();
	USART1_Init();
	I2C1_Init();
	MPU9250_Init();
	//AK8963_Init();
	// probably doesnt need anymore?
	//SPI1_Init(SPI_BaudRatePrescaler_64);
	// and this neither?
	//SPI1_Init(SPI_BaudRatePrescaler_2);

	ESC_Init(&esc1, 12);
	ESC_Init(&esc2, 13);
	ESC_Init(&esc3, 14);
	ESC_Init(&esc4, 15);

	// onödiga?
	PID_Init(&mpu_roll1);
	PID_Init(&mpu_pitch1);
	PID_Init(&mpu_yaw1);
}
int loopTest = 0;
int main(void)
{
	System_Init();

	volatile uint32_t CurrentTime = GetMicros();
	for(;;) {
		if(GetMicros() - CurrentTime >= LOOP_PERIOD_TIME_US){
			CurrentTime = GetMicros();
			//printf("hej8\r\n");
			//DisplayMag(&mpu_mag1);
			//Delay(500000);
			/*if (loopTest > 4){
				SendData(0x02);
				loopTest = 0;
			}*/
			//USART1_SendByte(0x02);
			//USART1_SendString("hello");
			MPU9250_ReadAcc(&mpu_acc1);
			MPU9250_ReadGyr(&mpu_gyr1);
			AK8963_ReadMag(&mpu_mag1);
			//AK8963_ReadMag(&mpu_mag1);
			#ifdef USB_DEBUG
			DisplayRaw(&mpu_acc1, &mpu_gyr1, &mpu_mag1);

			#endif
			Kalman_Calc(&mpu_acc1, &mpu_gyr1, &mpu_roll1, &mpu_pitch1, &mpu_yaw1);
			//#ifdef USB_DEBUG
			//DisplayFixed(&mpu_roll1, &mpu_pitch1);
			//#endif
			PID_Calc(&mpu_roll1);
			PID_Calc(&mpu_pitch1);
			//PID_Calc(&mpu_yaw1);
			UpdateMotors(&esc1, &esc2, &esc3, &esc4, mpu_roll1.Output, mpu_pitch1.Output);
		}
	}
	return 0;
}

//Dummy function to avoid compiler error
void _init()
{

}
