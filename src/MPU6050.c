#include "MPU6050.h"
#include "I2C.h"
#include "delay.h"

#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
#include "usbd_cdc_vcp.h"


void MPU6050_Init()
{
    Delay(500000);
    MPU6050_WriteRegiser(PWR_MGMT_1,PLL_Z_GYRO);
    MPU6050_WriteRegiser(USER_CTRL,MASTER_AND_FIFO_DISABLED);
    MPU6050_WriteRegiser(BYPASS_MODE,BYPASS_ON);
    MPU6050_WriteRegiser(SMPRT_DIV,RATE333Hz);
    MPU6050_WriteRegiser(CONFIG,BW20HZ);
    MPU6050_WriteRegiser(GYRO_CONFIG,GYRO_RANGE_500);
    MPU6050_WriteRegiser(ACCEL_CONFIG,ACCEL_RAGE_2G);
    Delay(500000);
}

void MPU6050_WriteRegiser(uint8_t RegAdress, uint8_t RegData)
{
    // start a transmission in Master transmitter mode
    I2C_start(I2C1, MPU6050_ADDRESS<<1, I2C_Direction_Transmitter);

	I2C_write(I2C1, RegAdress); // write one byte to the slave
	I2C_write(I2C1, RegData); // write another byte to the slave
	I2C_stop(I2C1); // stop the transmission
    Delay(200000);
}

void MPU6050_ReadAcc(IMU_DATA_t *acc)
{
	int i = 0;
	uint8_t buffer[6];
    int16_t racc[3];

    I2C_start(I2C1, MPU6050_ADDRESS<<1, I2C_Direction_Transmitter);
    I2C_write(I2C1, ACCEL_XOUT);
    I2C_stop(I2C1); // stop the transmission

    // start a transmission in Master receiver mode
    I2C_start(I2C1, MPU6050_ADDRESS<<1, I2C_Direction_Receiver);

    // read one byte and request another byte
    for(i = 0; i < 6-1; i++){
        buffer[i] = I2C_read_ack(I2C1);
    }
    // read one byte and don't request another byte, stop transmission
    buffer[6-1] = I2C_read_nack(I2C1);

    // Now multiply by -1 for coordinate system transformation here, because of double negation:
    // We want the gravity vector, which is negated acceleration vector.
    racc[0] = (((int16_t) buffer[0]) << 8) | buffer[1];  // X axis (internal sensor y axis)
    racc[1] = (((int16_t) buffer[2]) << 8) | buffer[3];  // Y axis (internal sensor x axis)
    racc[2] = (((int16_t) buffer[4]) << 8) | buffer[5];  // Z axis (internal sensor z axis)

    // Compensate accelerometer error
    // put this in a seperate inline void funcion
    acc->Roll = -(racc[0] - ACCEL_X_OFFSET) * ACCEL_X_SCALE;
    acc->Pitch = -(racc[1] - ACCEL_Y_OFFSET) * ACCEL_Y_SCALE;
    acc->Yaw = -(racc[2] - ACCEL_Z_OFFSET) * ACCEL_Z_SCALE;

}

void MPU6050_ReadGyr(IMU_DATA_t *gyr)
{
	int i = 0;
	uint8_t buffer[6];
    int16_t rgyr[3];

    I2C_start(I2C1, MPU6050_ADDRESS<<1, I2C_Direction_Transmitter);
    I2C_write(I2C1, GYRO_XOUT);
    I2C_stop(I2C1); // stop the transmission

    // start a transmission in Master receiver mode
    I2C_start(I2C1, MPU6050_ADDRESS<<1, I2C_Direction_Receiver);

    // read one byte and request another byte
    for(i = 0; i < 6-1; i++){
        buffer[i] = I2C_read_ack(I2C1);
    }
    // read one byte and don't request another byte, stop transmission
    buffer[6-1] = I2C_read_nack(I2C1);

    // Now multiply by -1 for coordinate system transformation here, because of double negation:
    // We want the gravity vector, which is negated acceleration vector.
    rgyr[0] = (((int16_t) buffer[0]) << 8) | buffer[1];  // X axis (internal sensor y axis)
    rgyr[1] = (((int16_t) buffer[2]) << 8) | buffer[3];  // Y axis (internal sensor x axis)
    rgyr[2] = (((int16_t) buffer[4]) << 8) | buffer[5];  // Z axis (internal sensor z axis)

    // Compensate accelerometer error
    // put this in a seperate inline void funcion
    /*gyr->Roll = GYRO_X_SCALE*rgyr[0];
    gyr->Roll -= GYRO_AVERAGE_OFFSET_X;
    gyr->Pitch = GYRO_Y_SCALE*rgyr[1];
    gyr->Pitch -= GYRO_AVERAGE_OFFSET_Y;
    gyr->Yaw = GYRO_Z_SCALE*rgyr[2];
    gyr->Yaw -= GYRO_AVERAGE_OFFSET_Z;*/

    gyr->Roll = GYRO_X_SCALE*rgyr[0] - GYRO_AVERAGE_OFFSET_X;
    gyr->Pitch = GYRO_Y_SCALE*rgyr[1] - GYRO_AVERAGE_OFFSET_Y;
    gyr->Yaw = GYRO_Z_SCALE*rgyr[2] - GYRO_AVERAGE_OFFSET_Z;
}
