#ifndef MPU9250_H
#define MPU9250_H

#include <stdint.h>
#include "quadcopter_structures.h"

void MPU9250_Init(void);
void MPU9250_WriteRegiser(uint8_t RegAdress, uint8_t RegData);
void MPU9250_ReadAcc(IMU_DATA_t *acc);
void MPU9250_ReadGyr(IMU_DATA_t *gyr);

void AK8963_Init(void);
void AK8963_WriteRegister(uint8_t RegAdress, uint8_t RegData);
void AK8963_ReadMag(IMU_DATA_t *mag);


#endif
