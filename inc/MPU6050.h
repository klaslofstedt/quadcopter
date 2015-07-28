#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include "stm32f4xx_conf.h"
#include <stm32f4xx.h>
#include <stm32f4xx_i2c.h>
#include "quadcopter_structures.h"


#define MPU6050_ADDRESS             0x68 // 0x68 or 0x69
#define BAROMETRIC_ADDRESS          0x77
#define MAGNETOMETER_ADDRESS        0x1E

#define USER_CTRL                   0x6A
#define MASTER_AND_FIFO_DISABLED    0x00

#define BYPASS_MODE                 0x37
#define BYPASS_ON                   0x02
#define BYPASS_OFF                  0x00

#define PWR_MGMT_1          0x6B
#define PLL_X_GYRO          0x01
#define PLL_Y_GYRO          0x02
#define PLL_Z_GYRO          0x03

#define SMPRT_DIV           0x19        // sample rate = 1(kHz)/(1+SMPLTR_DIV)
#define RATE500Hz           0x01
#define RATE333Hz           0x02
#define RATE250Hz           0x03
#define RATE200Hz           0x04
#define RATE167Hz           0x05
#define RATE143Hz           0x06
#define RATE125Hz           0x07
#define RATE125Hz           0x08
#define RATE111Hz           0x09
#define RATE100Hz           0x0A

#define CONFIG              0x1A
#define BW260HZ             0x00
#define BW185HZ             0x01
#define BW95HZ              0x02
#define BW44HZ              0x03
#define BW20HZ              0x04

#define GYRO_CONFIG         0x1B
#define GYRO_RANGE_250      0x00
#define GYRO_RANGE_500      0x08
#define GYRO_RANGE_1000     0x10
#define GYRO_RANGE_2000     0x18

#define ACCEL_CONFIG        0x1C
#define ACCEL_RAGE_2G       0x00      //16384 LSB/g
#define ACCEL_RAGE_4G       0x08
#define ACCEL_RAGE_8G       0x10
#define ACCEL_RAGE_10G      0x18

#define ACCEL_XOUT     0x3B                // Accelerometer output data registers
#define GYRO_XOUT      0x43                // Gyroscopes output data registers

#define MPU6050_WHO_AM_I        0x75

#define GYRO_X_SCALE 0.00026646248//0.01526717557 (en grados)
#define GYRO_Y_SCALE 0.00026646248//0.01526717557
#define GYRO_Z_SCALE 0.00026646248//0.01526717557
#define GYRO_AVERAGE_OFFSET_X 0
#define GYRO_AVERAGE_OFFSET_Y 0
#define GYRO_AVERAGE_OFFSET_Z 0

#define ACCEL_X_SCALE 0.00006103515
#define ACCEL_Y_SCALE 0.00006103515
#define ACCEL_Z_SCALE 0.00006103515
#define ACCEL_X_OFFSET 0
#define ACCEL_Y_OFFSET 0
#define ACCEL_Z_OFFSET 0

void MPU6050_Init();
void MPU6050_WriteRegiser(uint8_t RegAdress, uint8_t RegData);
void MPU6050_ReadAcc(IMU_DATA_t *acc);
void MPU6050_ReadGyr(IMU_DATA_t *gyr);


#endif
