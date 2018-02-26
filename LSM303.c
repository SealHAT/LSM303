/*
 * LSM303.c
 *
 * Created: 2/23/2018 2:56:23 PM
 *  Author: hpan5
 */ 
#include "LSM303.h"

#define SENSITIVITY_ACC		(0.06103515625)		/* LSB/mg */
#define SENSITIVITY_MAG		(0.00048828125)		/* LSB/Ga */

static struct i2c_m_sync_desc lsm303c_sync;		/* Structure for IMU communications */

/* Read a single register from the IMU */
static uint8_t readReg(const I2C_ADDR_t DEVICE_ADDRESS, const uint8_t REG)
{
	uint8_t retval;
	i2c_m_sync_set_slaveaddr(&lsm303c_sync, DEVICE_ADDRESS, I2C_M_SEVEN);
	i2c_m_sync_cmd_read(&lsm303c_sync, REG, &retval, 1);
	return retval;
}

/* Write a single register to the IMU */
static void writeReg(const I2C_ADDR_t DEVICE_ADDRESS, const uint8_t REG, uint8_t val)
{
	i2c_m_sync_set_slaveaddr(&lsm303c_sync, DEVICE_ADDRESS, I2C_M_SEVEN);
	i2c_m_sync_cmd_write(&lsm303c_sync, REG, &val, 1);
}

bool imu_init(struct i2c_m_sync_desc *const WIRE)
{
	lsm303c_sync  = *WIRE;
	i2c_m_sync_enable(&lsm303c_sync);
	return true;
}

bool acc_config(const ACC_FS_t RANGE, const ACC_BDU_t BLOCK_UPDATE, const uint8_t AXIS, const ACC_ODR_t RATE)
{
	/* Basic Read-modify-write operation to leave other values unchanged */
	uint8_t reg1 = readReg(ACC_I2C_ADDR, ACC_CTRL1);
	uint8_t reg4 = readReg(ACC_I2C_ADDR, ACC_CTRL4);
	
	reg1 |= (BLOCK_UPDATE | AXIS | RATE);
	reg4 |= (RANGE);
	
	writeReg(ACC_I2C_ADDR, ACC_CTRL1, reg1);
	writeReg(ACC_I2C_ADDR, ACC_CTRL1, reg4);
	
	return true;
}

ACC_STATUS_FLAGS_t acc_getStatus()
{
	return readReg(ACC_I2C_ADDR, ACC_STATUS);	
}

AxesRaw_t acc_read()
{
	AxesRaw_t retval;	/* Return value, all three axis */
	uint8_t dataHigh;	/* For collecting the MSB of an axis */
	uint8_t dataLow;	/* For collecting the LSB of an axis */
	
	dataLow  = readReg(ACC_I2C_ADDR, ACC_OUT_X_L);
	dataHigh = readReg(ACC_I2C_ADDR, ACC_OUT_X_H);
	retval.xAxis = (int16_t)(dataLow | (dataHigh << 8));
	
	dataLow  = readReg(ACC_I2C_ADDR, ACC_OUT_Y_L);
	dataHigh = readReg(ACC_I2C_ADDR, ACC_OUT_Y_H);
	retval.yAxis = (int16_t)(dataLow | (dataHigh << 8));
	
	dataLow  = readReg(ACC_I2C_ADDR, ACC_OUT_Z_L);
	dataHigh = readReg(ACC_I2C_ADDR, ACC_OUT_Z_H);
	retval.zAxis = (int16_t)(dataLow | (dataHigh << 8));
	
	return retval;
}
/*
bool mag_config(const MAG_DO_t RATE, const MAG_FS_t SCALE, const MAG_BDU_t BLOCK_UPDATE, const MAG_OMXY_t PWR_MODE, const MAG_OMZ_t PERFORMANCE, const MAG_MD_t CONV_MODE)
{
	
}*/

AxesRaw_t mag_read()
{
	AxesRaw_t retval;	/* Return value, all three axis */
	uint8_t dataHigh;	/* For collecting the MSB of an axis */
	uint8_t dataLow;	/* For collecting the LSB of an axis */
	
	dataLow  = readReg(MAG_I2C_ADDR, MAG_OUTX_L);
	dataHigh = readReg(MAG_I2C_ADDR, MAG_OUTX_H);
	retval.xAxis = (int16_t)(dataLow | (dataHigh << 8));
		
	dataLow  = readReg(MAG_I2C_ADDR, MAG_OUTY_L);
	dataHigh = readReg(MAG_I2C_ADDR, MAG_OUTY_H);
	retval.yAxis = (int16_t)(dataLow | (dataHigh << 8));
		
	dataLow  = readReg(MAG_I2C_ADDR, MAG_OUTZ_L);
	dataHigh = readReg(MAG_I2C_ADDR, MAG_OUTZ_L);
	retval.zAxis = (int16_t)(dataLow | (dataHigh << 8));
		
	return retval;
}

int16_t imu_readTemp()
{
	uint8_t dataHigh;	/* For collecting the MSB of an axis */
	uint8_t dataLow;	/* For collecting the LSB of an axis */
	
	dataLow  = readReg(MAG_I2C_ADDR, MAG_TEMP_OUT_L);
	dataHigh = readReg(MAG_I2C_ADDR, MAG_TEMP_OUT_H);
	
	return ((dataHigh << 8) | dataLow );
}
