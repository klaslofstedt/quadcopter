 #include "AK8963.h"
#include "I2C.h"
#include "delay.h"
#include "USB.h"
#include "quadcopter_structures.h"


//Magnetometer Registers
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

#define MAG_SCALE       10.*4912./32760.0 // resolution

#define MAG_X_OFFSET 0
#define MAG_Y_OFFSET 0
#define MAG_Z_OFFSET 0

float mag_calibration[3];

/*void AK8963_Init(void)
{
    // First extract the factory calibration for each magnetometer axis
    uint8_t buffer[3];  // x/y/z gyro calibration data stored here
    writeByte(AK8963_ADDRESS, AK8963_CNTL, MODE0_POWER_DOWN); // Power down magnetometer
    delay(10);
    writeByte(AK8963_ADDRESS, AK8963_CNTL, MODE6_FUSE_ROM_ACCESS); // Enter Fuse ROM access mode
    delay(10);
    readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &buffer[0]);  // Read the x-, y-, and z-axis calibration values
    mag_calibration[0] =  (float)(buffer[0] - 128)/256.0 + 1.0;   // Return x-axis sensitivity adjustment values, etc.
    mag_calibration[1] =  (float)(buffer[1] - 128)/256.0 + 1.0;
    mag_calibration[2] =  (float)(buffer[2] - 128)/256.0 + 1.0;
    writeByte(AK8963_ADDRESS, AK8963_CNTL, MODE0_POWER_DOWN); // Power down magnetometer
    delay(10);
    // Configure the magnetometer for continuous read and highest resolution
    // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
    // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
    writeByte(AK8963_ADDRESS, AK8963_CNTL, RESOLUTION_16_BIT << 4 | MODE3_CONTINUOUS_100_HZ); // Set magnetometer data resolution and sample ODR
    delay(10);
}*/

void AK8963_Init(void)
{
    int i = 0;
    Delay(500000);
    // First extract the factory calibration for each magnetometer axis
    uint8_t buffer[3];  // x/y/z gyro calibration data stored here
    AK8963_WriteRegister(AK8963_CNTL, MODE0_POWER_DOWN); // Power down magnetometer
    AK8963_WriteRegister(AK8963_CNTL, MODE6_FUSE_ROM_ACCESS); // Enter Fuse ROM access mode

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

/*void AK8963_ReadMag(IMU_DATA_t *mag)
{
    uint8_t buffer[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
    uint16_t rmag[3];
    if(readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01) { // wait for magnetometer data ready bit to be set
        readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &buffer[0]);  // Read the six raw data and ST2 registers sequentially into data array
        uint8_t c = buffer[6]; // End data read by reading ST2 register
        if(!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
            // maybe change to 1, 0, 3, 2,  5, 4
            rmag[0] = (((int16_t) buffer[0]) << 8) | buffer[1];  // X axis (internal sensor y axis)
            rmag[1] = (((int16_t) buffer[2]) << 8) | buffer[3];  // Y axis (internal sensor x axis)
            rmag[2] = (((int16_t) buffer[4]) << 8) | buffer[5];  // Z axis (internal sensor z axis)

            mag->Roll = (float)buffer[0]*MAG_SCALE*mag_calibration[0] - magbias[0];  // get actual magnetometer value, this depends on scale being set
            mag->Pitch = (float)buffer[1]*MAG_SCALE*mag_calibration[1] - magbias[1];
            mag->Yaw = (float)buffer[2]*MAG_SCALE*mag_calibration[2] - magbias[2];
        }
    }
}*/

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

        if(!(buffer[6] & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
            // maybe change to 1, 0, 3, 2,  5, 4
            rmag[0] = (((int16_t) buffer[0]) << 8) | buffer[1];  // X axis (internal sensor y axis)
            rmag[1] = (((int16_t) buffer[2]) << 8) | buffer[3];  // Y axis (internal sensor x axis)
            rmag[2] = (((int16_t) buffer[4]) << 8) | buffer[5];  // Z axis (internal sensor z axis)

            // maybe it should look lik this:
            // (float)buffer[0]*MAG_SCALE*mag_calibration[0] - MAG_X_OFFSET;
            mag->Roll = (float)(rmag[0] - MAG_X_OFFSET) * MAG_SCALE * mag_calibration[0];
            mag->Pitch = (float)(rmag[1] - MAG_Y_OFFSET) * MAG_SCALE * mag_calibration[1];
            mag->Yaw = (float)(rmag[2] - MAG_Z_OFFSET) * MAG_SCALE * mag_calibration[2];
        }
    }
}
