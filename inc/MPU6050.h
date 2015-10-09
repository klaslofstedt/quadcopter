#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
//#include "stm32f4xx_conf.h"
//#include <stm32f4xx.h>
//#include <stm32f4xx_i2c.h>
#include "quadcopter_structures.h"

void MPU6050_Init();
void MPU6050_WriteRegiser(uint8_t RegAdress, uint8_t RegData);
void MPU6050_ReadAcc(IMU_DATA_t *acc);
void MPU6050_ReadGyr(IMU_DATA_t *gyr);


#endif
