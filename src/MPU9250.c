#include "MPU9250.h"
#include "I2C.h"
#include "delay.h"
#include "USB.h"

#define MPU9250_ADDRESS             0x68 // 0x68 or 0x69
#define BAROMETRIC_ADDRESS          0x77
//#define MAGNETOMETER_ADDRESS        0x1E

#define USER_CTRL                   0x6A
#define MASTER_AND_FIFO_DISABLED    0x00

#define BYPASS_MODE                 0x37
#define BYPASS_ON                   0x02
#define BYPASS_OFF                  0x00

#define PWR_MGMT_1          0x6B
/* these names makes no sense
should be something with clock souce PLL */
#define PLL_X_GYRO          0x01
#define PLL_Y_GYRO          0x02
#define PLL_Z_GYRO          0x03

#define SMPLRT_DIV           0x19        // sample rate = 1(kHz)/(1+SMPLTR_DIV)
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
#define BW292HZ             0x00
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

#define ACCEL_XOUT_H     0x3B                // Accelerometer output data registers
#define ACCEL_XOUT_H     0x3C
#define GYRO_XOUT_H     0x43                // Gyroscopes output data registers
#define GYRO_XOUT_L     0x44

#define MPU9250_WHO_AM_I        0x75

#define GYRO_X_SCALE 0.00026646248//0.01526717557 (in degrees)
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

/**************** Mag defines ***********************/
#define AK8963_ADDRESS   0x0C

// read only
#define WHO_AM_I_AK8963  0x00 // should return 0x48
#define I_AM_AK8963      0x48

#define INFO             0x01
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L	 0x03  // data
#define AK8963_XOUT_H	 0x04
#define AK8963_YOUT_L	 0x05
#define AK8963_YOUT_H	 0x06
#define AK8963_ZOUT_L	 0x07
#define AK8963_ZOUT_H	 0x08
#define AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
// read / write
#define AK8963_CNTL      0x0A
#define MODE0_POWER_DOWN  0x00
#define MODE1_SINGEL_MSRM 0x01
#define MODE2_CONTINUOUS_8_HZ   0x02
#define MODE3_CONTINUOUS_100_HZ 0x06
#define MODE4_EXT_TRIGGER_MSRM 0x04
#define MODE5_SELF_TEST   0x08
#define MODE6_FUSE_ROM_ACCESS 0x0F
#define RESOLUTION_16_BIT 0x10
#define RESOLUTION_14_BIT 0x00

#define AK8963_ASTC      0x0C  // Self test control
#define NORMAL           0x00
#define MAG_FIELD_SELF_TEST 0x40

#define AK8963_I2CDIS    0x0F  // I2C disable
// some i2c disable bits i don't understand from the datasheet
// take a look at the data sheet if needed in the future

// write only
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value

#define MAG_SCALE       10.0*4912./32760.0 // resolution

#define MAG_X_OFFSET 0
#define MAG_Y_OFFSET 0
#define MAG_Z_OFFSET 0

float mag_calibration[3];

/*// Configure gyroscope range
I2CwriteByte(MPU9250_ADDRESS,27,GYRO_FULL_SCALE_2000_DPS);
// Configure accelerometers range
I2CwriteByte(MPU9250_ADDRESS,28,ACC_FULL_SCALE_16_G);
// Set by pass mode for the magnetometers
I2CwriteByte(MPU9250_ADDRESS,0x37,0x02);
// Request first magnetometer single measurement
I2CwriteByte(MAG_ADDRESS,0x0A,0x01);*/



void MPU9250_Init()
{
    Delay(500000);
    MPU9250_WriteRegiser(PWR_MGMT_1,PLL_Z_GYRO); // chose clock
    MPU9250_WriteRegiser(USER_CTRL,MASTER_AND_FIFO_DISABLED); // do nothing?
    MPU9250_WriteRegiser(BYPASS_MODE,BYPASS_ON); // 3
    MPU9250_WriteRegiser(SMPLRT_DIV,RATE333Hz); // sample rate
    MPU9250_WriteRegiser(CONFIG,BW20HZ);
    MPU9250_WriteRegiser(GYRO_CONFIG,GYRO_RANGE_500); // 1
    MPU9250_WriteRegiser(ACCEL_CONFIG,ACCEL_RAGE_2G); // 2
    AK8963_WriteRegister(AK8963_CNTL, RESOLUTION_16_BIT << 4 | MODE1_SINGEL_MSRM); // mag
    //AK8963_WriteRegister(AK8963_CNTL, RESOLUTION_16_BIT << 4);
}

void MPU9250_WriteRegiser(uint8_t RegAdress, uint8_t RegData)
{
    // start a transmission in Master transmitter mode
    I2C_start(I2C1, MPU9250_ADDRESS<<1, I2C_Direction_Transmitter);

	I2C_write(I2C1, RegAdress); // write one byte to the slave
	I2C_write(I2C1, RegData); // write another byte to the slave
	I2C_stop(I2C1); // stop the transmission
    Delay(200000);
}
// only read ACCEL_XOUT_H which are the 8 MSB. ACCEL_XOUT_L should be read too!
void MPU9250_ReadAcc(IMU_DATA_t *acc)
{
	int i = 0;
	uint8_t buffer[6];
    int16_t racc[3];

    I2C_start(I2C1, MPU9250_ADDRESS<<1, I2C_Direction_Transmitter);
    I2C_write(I2C1, ACCEL_XOUT_H);
    I2C_stop(I2C1); // stop the transmission

    // start a transmission in Master receiver mode
    I2C_start(I2C1, MPU9250_ADDRESS<<1, I2C_Direction_Receiver);

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

void MPU9250_ReadGyr(IMU_DATA_t *gyr)
{
	int i = 0;
	uint8_t buffer[6];
    int16_t rgyr[3];

    I2C_start(I2C1, MPU9250_ADDRESS<<1, I2C_Direction_Transmitter);
    I2C_write(I2C1, GYRO_XOUT_H);
    I2C_stop(I2C1); // stop the transmission

    // start a transmission in Master receiver mode
    I2C_start(I2C1, MPU9250_ADDRESS<<1, I2C_Direction_Receiver);

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

/************************* MAG *********************/
float mag_calibration[3];

void AK8963_Init(void)
{
    int i = 0;
    Delay(500000);
    // First extract the factory calibration for each magnetometer axis
    uint8_t buffer[3];  // x/y/z gyro calibration data stored here
    AK8963_WriteRegister(AK8963_CNTL, MODE1_SINGEL_MSRM); // Power down magnetometer
    //AK8963_WriteRegister(AK8963_CNTL, MODE6_FUSE_ROM_ACCESS); // Enter Fuse ROM access mode

    I2C_start(I2C1, AK8963_ADDRESS<<1, I2C_Direction_Transmitter);
    I2C_write(I2C1, AK8963_ASAX);
    I2C_stop(I2C1); // stop the transmission
    // start a transmission in Master receiver mode
    I2C_start(I2C1, AK8963_ADDRESS<<1, I2C_Direction_Receiver);

    // read one byte and request another byte
    for(i = 0; i < 3-1; i++){
        buffer[i] = I2C_read_ack(I2C1);
    }
    // read one byte and don't request another byte, stop transmission
    buffer[3-1] = I2C_read_nack(I2C1);

    mag_calibration[0] =  (float)(buffer[0] - 128)/256.0 + 1.0;   // Return x-axis sensitivity adjustment values, etc.
    mag_calibration[1] =  (float)(buffer[1] - 128)/256.0 + 1.0;
    mag_calibration[2] =  (float)(buffer[2] - 128)/256.0 + 1.0;

    AK8963_WriteRegister(AK8963_CNTL, MODE0_POWER_DOWN); // Power down magnetometer
    AK8963_WriteRegister(AK8963_CNTL, RESOLUTION_16_BIT << 4 | MODE3_CONTINUOUS_100_HZ); // Set magnetometer data resolution and sample ODR
    Delay(500000);
}

void AK8963_WriteRegister(uint8_t RegAdress, uint8_t RegData)
{
    // start a transmission in Master transmitter mode
    I2C_start(I2C1, AK8963_ADDRESS<<1, I2C_Direction_Transmitter);
	I2C_write(I2C1, RegAdress); // write one byte to the slave
	I2C_write(I2C1, RegData); // write another byte to the slave
	I2C_stop(I2C1); // stop the transmission
    Delay(200000);
}


void AK8963_ReadMag(IMU_DATA_t *mag)
{
    int i = 0;
    uint8_t buffer[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
    uint16_t rmag[3];

    I2C_start(I2C1, AK8963_ADDRESS<<1, I2C_Direction_Transmitter);
    I2C_write(I2C1, AK8963_ST1);
    I2C_stop(I2C1); // stop the transmission

    // start a transmission in Master receiver mode
    I2C_start(I2C1, AK8963_ADDRESS<<1, I2C_Direction_Receiver);
    if(I2C_read_nack(I2C1) & 0x01){
        I2C_start(I2C1, AK8963_ADDRESS<<1, I2C_Direction_Transmitter);
        I2C_write(I2C1, AK8963_XOUT_L);
        I2C_stop(I2C1); // stop the transmission
        I2C_start(I2C1, AK8963_ADDRESS<<1, I2C_Direction_Receiver);

        for(i = 0; i < 7-1; i++){
            buffer[i] = I2C_read_ack(I2C1);
        }
        // read one byte and don't request another byte, stop transmission
        buffer[7-1] = I2C_read_nack(I2C1);

        /*if(!(buffer[6] & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
            // maybe change to 1, 0, 3, 2,  5, 4
            rmag[0] = (((int16_t) buffer[0]) << 8) | buffer[1];  // X axis (internal sensor y axis)
            rmag[1] = (((int16_t) buffer[2]) << 8) | buffer[3];  // Y axis (internal sensor x axis)
            rmag[2] = (((int16_t) buffer[4]) << 8) | buffer[5];  // Z axis (internal sensor z axis)

            // maybe it should look lik this:
            // (float)buffer[0]*MAG_SCALE*mag_calibration[0] - MAG_X_OFFSET;
            mag->Roll = (float)(rmag[0] - MAG_X_OFFSET) * MAG_SCALE;// * mag_calibration[0];
            mag->Pitch = (float)(rmag[1] - MAG_Y_OFFSET) * MAG_SCALE;// * mag_calibration[1];
            mag->Yaw = (float)(rmag[2] - MAG_Z_OFFSET) * MAG_SCALE;// * mag_calibration[2];
        }*/
        rmag[0] = (((int16_t) buffer[0]) << 8) | buffer[1];  // X axis (internal sensor y axis)
        rmag[1] = (((int16_t) buffer[2]) << 8) | buffer[3];  // Y axis (internal sensor x axis)
        rmag[2] = (((int16_t) buffer[4]) << 8) | buffer[5];  // Z axis (internal sensor z axis)

        // maybe it should look lik this:
        // (float)buffer[0]*MAG_SCALE*mag_calibration[0] - MAG_X_OFFSET;
        mag->Roll = (float)(rmag[0] - MAG_X_OFFSET) * MAG_SCALE;// * mag_calibration[0];
        mag->Pitch = (float)(rmag[1] - MAG_Y_OFFSET) * MAG_SCALE;// * mag_calibration[1];
        mag->Yaw = (float)(rmag[2] - MAG_Z_OFFSET) * MAG_SCALE;// * mag_calibration[2];
    }
}
