#ifndef AK8963_H
#define AK8963_H

#include <stdint.h>
#include "quadcopter_structures.h"

void AK8963_Init(void);
void AK8963_WriteRegister(uint8_t RegAdress, uint8_t RegData);
void AK8963_ReadMag(IMU_DATA_t *mag);

#endif
