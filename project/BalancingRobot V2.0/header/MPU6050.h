#ifndef MPU6050
#define MPU6050

#include <avr/io.h>
#include <util/delay.h>
#include "i2cmaster.h"

#define MPU6050_GYRO_ROLL			MPU6050_GYRO_X
#define MPU6050_GYRO_PITCH			MPU6050_GYRO_Z
#define MPU6050_GYRO_YAW			MPU6050_GYRO_Y

#define MPU6050_GYRO_ROLL_AVG		MPU6050_GYRO_X_AVG
#define MPU6050_GYRO_PITCH_AVG		MPU6050_GYRO_Z_AVG
#define MPU6050_GYRO_YAW_AVG		MPU6050_GYRO_Y_AVG

#define MPU6050_DDR		DDRB
#define MPU6050_PORT	PORTB
#define MPU6050_AD0		PB5
#define MPU6050_AUX_VDDIO          0x01   // R/W
#define MPU6050_SMPLRT_DIV         0x19   // R/W
#define MPU6050_CONFIG             0x1A   // R/W
#define MPU6050_GYRO_CONFIG        0x1B   // R/W
#define MPU6050_ACCEL_CONFIG       0x1C   // R/W
#define MPU6050_FF_THR             0x1D   // R/W
#define MPU6050_FF_DUR             0x1E   // R/W
#define MPU6050_MOT_THR            0x1F   // R/W
#define MPU6050_MOT_DUR            0x20   // R/W
#define MPU6050_ZRMOT_THR          0x21   // R/W
#define MPU6050_ZRMOT_DUR          0x22   // R/W
#define MPU6050_FIFO_EN            0x23   // R/W
#define MPU6050_I2C_MST_CTRL       0x24   // R/W
#define MPU6050_I2C_SLV0_ADDR      0x25   // R/W
#define MPU6050_I2C_SLV0_REG       0x26   // R/W
#define MPU6050_I2C_SLV0_CTRL      0x27   // R/W
#define MPU6050_I2C_SLV1_ADDR      0x28   // R/W
#define MPU6050_I2C_SLV1_REG       0x29   // R/W
#define MPU6050_I2C_SLV1_CTRL      0x2A   // R/W
#define MPU6050_I2C_SLV2_ADDR      0x2B   // R/W
#define MPU6050_I2C_SLV2_REG       0x2C   // R/W
#define MPU6050_I2C_SLV2_CTRL      0x2D   // R/W
#define MPU6050_I2C_SLV3_ADDR      0x2E   // R/W
#define MPU6050_I2C_SLV3_REG       0x2F   // R/W
#define MPU6050_I2C_SLV3_CTRL      0x30   // R/W
#define MPU6050_I2C_SLV4_ADDR      0x31   // R/W
#define MPU6050_I2C_SLV4_REG       0x32   // R/W
#define MPU6050_I2C_SLV4_DO        0x33   // R/W
#define MPU6050_I2C_SLV4_CTRL      0x34   // R/W
#define MPU6050_I2C_SLV4_DI        0x35   // R
#define MPU6050_I2C_MST_STATUS     0x36   // R
#define MPU6050_INT_PIN_CFG        0x37   // R/W
#define MPU6050_INT_ENABLE         0x38   // R/W
#define MPU6050_INT_STATUS         0x3A   // R
#define MPU6050_ACCEL_XOUT_H       0x3B   // R
#define MPU6050_ACCEL_XOUT_L       0x3C   // R
#define MPU6050_ACCEL_YOUT_H       0x3D   // R
#define MPU6050_ACCEL_YOUT_L       0x3E   // R
#define MPU6050_ACCEL_ZOUT_H       0x3F   // R
#define MPU6050_ACCEL_ZOUT_L       0x40   // R
#define MPU6050_TEMP_OUT_H         0x41   // R
#define MPU6050_TEMP_OUT_L         0x42   // R
#define MPU6050_GYRO_XOUT_H        0x43   // R
#define MPU6050_GYRO_XOUT_L        0x44   // R
#define MPU6050_GYRO_YOUT_H        0x45   // R
#define MPU6050_GYRO_YOUT_L        0x46   // R
#define MPU6050_GYRO_ZOUT_H        0x47   // R
#define MPU6050_GYRO_ZOUT_L        0x48   // R
#define MPU6050_EXT_SENS_DATA_00   0x49   // R
#define MPU6050_EXT_SENS_DATA_01   0x4A   // R
#define MPU6050_EXT_SENS_DATA_02   0x4B   // R
#define MPU6050_EXT_SENS_DATA_03   0x4C   // R
#define MPU6050_EXT_SENS_DATA_04   0x4D   // R
#define MPU6050_EXT_SENS_DATA_05   0x4E   // R
#define MPU6050_EXT_SENS_DATA_06   0x4F   // R
#define MPU6050_EXT_SENS_DATA_07   0x50   // R
#define MPU6050_EXT_SENS_DATA_08   0x51   // R
#define MPU6050_EXT_SENS_DATA_09   0x52   // R
#define MPU6050_EXT_SENS_DATA_10   0x53   // R
#define MPU6050_EXT_SENS_DATA_11   0x54   // R
#define MPU6050_EXT_SENS_DATA_12   0x55   // R
#define MPU6050_EXT_SENS_DATA_13   0x56   // R
#define MPU6050_EXT_SENS_DATA_14   0x57   // R
#define MPU6050_EXT_SENS_DATA_15   0x58   // R
#define MPU6050_EXT_SENS_DATA_16   0x59   // R
#define MPU6050_EXT_SENS_DATA_17   0x5A   // R
#define MPU6050_EXT_SENS_DATA_18   0x5B   // R
#define MPU6050_EXT_SENS_DATA_19   0x5C   // R
#define MPU6050_EXT_SENS_DATA_20   0x5D   // R
#define MPU6050_EXT_SENS_DATA_21   0x5E   // R
#define MPU6050_EXT_SENS_DATA_22   0x5F   // R
#define MPU6050_EXT_SENS_DATA_23   0x60   // R
#define MPU6050_MOT_DETECT_STATUS  0x61   // R
#define MPU6050_I2C_SLV0_DO        0x63   // R/W
#define MPU6050_I2C_SLV1_DO        0x64   // R/W
#define MPU6050_I2C_SLV2_DO        0x65   // R/W
#define MPU6050_I2C_SLV3_DO        0x66   // R/W
#define MPU6050_I2C_MST_DELAY_CTRL 0x67   // R/W
#define MPU6050_SIGNAL_PATH_RESET  0x68   // R/W
#define MPU6050_MOT_DETECT_CTRL    0x69   // R/W
#define MPU6050_USER_CTRL          0x6A   // R/W
#define MPU6050_PWR_MGMT_1         0x6B   // R/W
#define MPU6050_PWR_MGMT_2         0x6C   // R/W
#define MPU6050_FIFO_COUNTH        0x72   // R/W
#define MPU6050_FIFO_COUNTL        0x73   // R/W
#define MPU6050_FIFO_R_W           0x74   // R/W
#define MPU6050_WHO_AM_I           0x75   // R

#define MPU6050_ADRESS			0x68

#define MPU6050_GYRO_X			0
#define MPU6050_GYRO_Y			2
#define MPU6050_GYRO_Z			4

#define MPU6050_ACC_X			0
#define MPU6050_ACC_Y			2
#define MPU6050_ACC_Z			4

//these are the average errors when the GYRO is stand still. Must be subtract from the MPU6050_read_GYRO reading.
//these concern for the 500deg/s setup (And only for may gyro)
//change these if you get inaccurate readings
#define MPU6050_GYRO_X_AVG		-286 
#define MPU6050_GYRO_Y_AVG		-215 
#define MPU6050_GYRO_Z_AVG		-144 

//these are the average errors when the accelerometer is stand still. Must be subtract from the MPU6050_read_ACC reading.
//these concern for the default setup (And only for may accelerometer)
//change these if you get inaccurate readings
#define MPU6050_ACC_X_AVG		950
#define MPU6050_ACC_Y_AVG		330
#define MPU6050_ACC_Z_AVG		2400

//Typical Sensitivity Scale Factor from the datasheet page 12
#define MPU6050_ScaleFactor		60

#define RAD_to_DEG		57.295779513082320876798154814105
#define DEG_to_RAD		0.0174532925


typedef struct
{
		int16_t x;
		int16_t y;
		int16_t z;
}accRawSTRUCT;

typedef struct  
{
		float	roll;
		float	pitch;
}accAngleSTRUCT;

typedef struct
{
	float x;	//Roll
	float y;	//Pitch
	float z;	//Yaw
}gyroRawSTRUCT;

typedef struct
{
	float	derivatedRoll;
	float	derivatedPitch;
	float	derivatedYaw;
}eulerSTRUCT;

typedef struct
{
	float	roll;
	float	pitch;
	float	yaw;
	float	gain;
}compSTRUCT;

uint8_t MPU6050_read(uint8_t MPU6050register);
void	MPU6050_write(uint8_t MPU6050register, uint8_t MPU6050data);
float	MPU6050_read_GYRO(uint8_t GYRO_axis);
int		MPU6050_read_ACC(uint8_t GYRO_axis);

eulerSTRUCT calcEulerAngles(gyroRawSTRUCT *gRaw, compSTRUCT *compAngle);

float compFilter(float angularRate, float accAngle, float angle, float gain);


uint8_t MPU6050_read(uint8_t MPU6050register)
{
	uint8_t result = 0;
	i2c_start_wait((MPU6050_ADRESS<<1)+I2C_WRITE);
	i2c_write(MPU6050register);
	i2c_rep_start((MPU6050_ADRESS<<1)+I2C_READ);
	result = i2c_readNak();
	i2c_stop();
	return result;
}

void MPU6050_write(uint8_t MPU6050register, uint8_t MPU6050data)
{
	i2c_start_wait((MPU6050_ADRESS<<1)+I2C_WRITE);
	i2c_write(MPU6050register);
	i2c_write(MPU6050data);
	i2c_stop();
}

float MPU6050_read_GYRO(uint8_t GYRO_axis)
{
	uint8_t MPU6050_HSB = 0;
	uint8_t MPU6050_LSB = 0;
	
	MPU6050_HSB = MPU6050_read(MPU6050_GYRO_XOUT_H+GYRO_axis);
	MPU6050_LSB = MPU6050_read(MPU6050_GYRO_XOUT_H+GYRO_axis+1);
	
	return ((MPU6050_HSB<<8)+MPU6050_LSB);
}

int MPU6050_read_ACC(uint8_t ACC_axis)
{
	uint8_t MPU6050_HSB = 0;
	uint8_t MPU6050_LSB = 0;
	
	MPU6050_HSB = MPU6050_read(MPU6050_ACCEL_XOUT_H+ACC_axis);
	MPU6050_LSB = MPU6050_read(MPU6050_ACCEL_XOUT_H+ACC_axis+1);
	
	return ((MPU6050_HSB<<8)+MPU6050_LSB);
}

eulerSTRUCT calcEulerAngles(gyroRawSTRUCT *gRaw, compSTRUCT *compAngle)
{
	eulerSTRUCT euAngle;
	euAngle.derivatedRoll	=	gRaw->x+gRaw->y*sin(compAngle->roll*DEG_to_RAD)*tan(compAngle->pitch*DEG_to_RAD)+gRaw->z*cos(compAngle->roll*DEG_to_RAD)*tan(compAngle->pitch*DEG_to_RAD);
	euAngle.derivatedPitch	=	gRaw->y*cos(compAngle->roll*DEG_to_RAD)-gRaw->z*sin(compAngle->roll*DEG_to_RAD);
	euAngle.derivatedYaw	=	gRaw->y*sin(compAngle->roll*DEG_to_RAD)/cos(compAngle->pitch*DEG_to_RAD)+gRaw->z*cos(compAngle->roll*DEG_to_RAD)/cos(compAngle->pitch*DEG_to_RAD);
	return euAngle;
}

float compFilter(float angularRate, float accAngle, float angle, float gain)
{
	angle	= gain*(angle+angularRate) + (1-gain) * accAngle;
	return angle;
}
#endif
