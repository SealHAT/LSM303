/*
 * LSM303.c
 *
 * Created: 2/23/2018 2:56:23 PM
 *  Author: hpan5
 */ 
#include "LSM303.h"
#include "math.h"


static struct i2c_m_sync_desc lsm303c_sync; /* Structure for IMU communications */

/* Read a single register for accelerometer*/
 uint8_t readReg(const LSM303_DEV_ADDR_t SLAVE_ADDRESS, const uint8_t REG){
	uint8_t retval;
	i2c_m_sync_set_slaveaddr(&lsm303c_sync, SLAVE_ADDRESS, I2C_M_SEVEN);
	i2c_m_sync_cmd_read(&lsm303c_sync, REG, &retval, 1);
	return retval;
}

/* Read a single register*/
 void writeReg(const LSM303_DEV_ADDR_t SLAVE_ADDRESS, const uint8_t Reg, uint8_t val)
 {
	struct _i2c_m_msg msg;
	uint8_t buff[2];
	buff[0] = Reg;
	buff[1] = val;
	
	msg.addr   = SLAVE_ADDRESS;
	msg.len    = 2;
	msg.flags  = I2C_M_STOP;
	msg.buffer = buff;
	
	_i2c_m_sync_transfer(&lsm303c_sync.device, &msg);
	
//	i2c_m_sync_set_slaveaddr(&lsm303c_sync, SLAVE_ADDRESS, I2C_M_SEVEN);
//	i2c_m_sync_cmd_write(&lsm303c_sync, Reg, &val, 1);
}

bool acc_config(const ACC_FS_t RANGE, const ACC_BDU_t BLOCK_UPDATE, const ACC_AXIS_EN_t AXIS, const ACC_ODR_t RATE, const ACC_INCREMENT_t INCREMENT)
{
	/* Basic Read-modify-write operation to leave other values unchanged */
//	volatile uint8_t reg1 = readReg(LSM303_ACCEL, ACC_CTRL1);
//	volatile uint8_t reg4 = readReg(LSM303_ACCEL, ACC_CTRL4);
	
//	reg1 |= (BLOCK_UPDATE | AXIS | RATE);
//	reg4 |= (RANGE | INCREMENT);
	
	writeReg(LSM303_ACCEL, ACC_CTRL1, (0x2F));
	writeReg(LSM303_ACCEL, ACC_CTRL1, (0x2F));
	writeReg(LSM303_ACCEL, ACC_CTRL4, (0x04));
	writeReg(LSM303_ACCEL, ACC_CTRL5, (0x00));
	
	volatile uint8_t reg1 = readReg(LSM303_ACCEL, ACC_CTRL1);
	volatile uint8_t reg4 = readReg(LSM303_ACCEL, ACC_CTRL4);
	volatile uint8_t reg5 = readReg(LSM303_ACCEL, ACC_CTRL5);
	
	return true;
}

bool imu_init(struct i2c_m_sync_desc *const WIRE)
{
	lsm303c_sync  = *WIRE;
	i2c_m_sync_enable(&lsm303c_sync);
	writeReg(LSM303_ACCEL, ACC_CTRL1, 0x2F);
	volatile uint8_t reg1 = readReg(LSM303_ACCEL, ACC_CTRL1);
	return true;
}

 AxesRaw_t acc_readXYZ()
{
	uint8_t valX[2];
	uint8_t valY[2];
	uint8_t valZ[2];
	AxesRaw_t Axes;
	
	valX[0] = readReg(LSM303_ACCEL, ACC_OUT_X_L);
	valX[1] = readReg(LSM303_ACCEL, ACC_OUT_X_H);
	Axes.xAxis = (int16_t)(valX[0] | (valX[1]<<8));
	
	valY[0] = readReg(LSM303_ACCEL, ACC_OUT_Y_L);
	valY[1] = readReg(LSM303_ACCEL, ACC_OUT_Y_H);
	Axes.yAxis = (int16_t)(valY[0] | (valY[1]<<8));
	
	valZ[0] =readReg(LSM303_ACCEL, ACC_OUT_Z_L);
	valZ[1] = readReg(LSM303_ACCEL,ACC_OUT_Z_H);
	Axes.zAxis = (int16_t)(valZ[0] | (valZ[1]<<8));
	
	return Axes;
}

ACC_STATUS_FLAGS_t acc_getStatus()
{
	return readReg(LSM303_ACCEL, ACC_STATUS);
}
