/*
 * LSM303.c
 *
 * Created: 2/23/2018 2:56:23 PM
 *  Author: hpan5
 */ 
#include "LSM303.h"
#include "LSM303CTypes.h"

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

bool acc_config()
{
	
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

void acc_clearREADYbit()
{
	readReg(ACC_I2C_ADDR, ACC_OUT_X_L);
	readReg(ACC_I2C_ADDR, ACC_OUT_X_H);
	readReg(ACC_I2C_ADDR, ACC_OUT_Y_L);
	readReg(ACC_I2C_ADDR, ACC_OUT_Y_H);
	readReg(ACC_I2C_ADDR, ACC_OUT_Z_L);
	readReg(ACC_I2C_ADDR, ACC_OUT_Z_H);
}

int32_t acc_SelfTest()
{
	uint8_t Status = 0x00;
	int OUTX_NOST, OUTY_NOST, OUTZ_NOST;
	int OUTX_ST, OUTY_ST, OUTZ_ST;
	
	writeReg(ACC_I2C_ADDR, ACC_CTRL1, ACC_ODR_50_Hz | ACC_BDU_ENABLE | ACC_X_ENABLE | ACC_Y_ENABLE | ACC_Z_ENABLE);
	writeReg(ACC_I2C_ADDR, ACC_CTRL4, ACC_FS_4g); //FS = 2g
	writeReg(ACC_I2C_ADDR, ACC_CTRL5, 0x00); //Disable acc self-test
	delay_ms(200);
	
	do{
		acc_clearREADYbit();
		Status = readReg(ACC_I2C_ADDR, ACC_STATUS);
	}while((Status & ACC_ZYX_NEW_DATA_AVAILABLE) == 0);
	
	if((Status & ACC_ZYX_NEW_DATA_AVAILABLE) != 0)
	{
		acc_readXYZ(&OUTX_NOST,&OUTY_NOST,&OUTZ_NOST);
	}
	
	acc_writeReg1(&wire,ACC_CTRL5, 0x04); //Enable acc self-test
	delay_ms(80);
	
	do{
		acc_clearREADYbit();
		Status = acc_readReg1(&wire,ACC_STATUS);
	}while((Status & ACC_ZYX_NEW_DATA_AVAILABLE) == 0);
	
	if((Status & ACC_ZYX_NEW_DATA_AVAILABLE) != 0){
		acc_readXYZ(&OUTX_ST,&OUTY_ST,&OUTZ_ST);
		gpio_set_pin_level(LED_BUILTIN,true);
	}
	
	int abs_X = abs(OUTX_ST - OUTX_NOST);
	int abs_Y = abs(OUTY_ST - OUTY_NOST);
	int abs_Z = abs(OUTZ_ST - OUTZ_NOST);
	
	if(	   (((abs_X*0.061) <= 1500) && ((abs_X*0.061) >= 70))
	&& (((abs_Y*0.061) <= 1500) && ((abs_Y*0.061) >= 70))
	&& (((abs_Z*0.061) <= 1500) && ((abs_Z*0.061) >= 70))
	)
	{
		return 0;
	}
	else
	{
		return -1;
	}
	acc_writeReg1(&wire,ACC_CTRL5, 0x00); //Disable acc self-test
}

bool mag_config()
{
	
}

void mag_clearREADYbit()
{
	readReg(MAG_I2C_ADDR, MAG_OUTX_L);
	readReg(MAG_I2C_ADDR, MAG_OUTX_H);
	readReg(MAG_I2C_ADDR, MAG_OUTY_L);
	readReg(MAG_I2C_ADDR, MAG_OUTY_H);
	readReg(MAG_I2C_ADDR, MAG_OUTZ_L);
	readReg(MAG_I2C_ADDR, MAG_OUTZ_H);
}

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