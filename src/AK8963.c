#include "AK8963.h"
#include "I2C.h"
#include "delay.h"
#include "USB.h"


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
#define MODE1_SINGEL_MEASUREMENT 0x01
#define MODE2_SELF_TEST   0x08
#define MODE3_FUSE_ROM_ACCESS 0x0F

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

void AK8963_Init(void)
{

}
