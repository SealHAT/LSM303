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
 static uint8_t readReg(const LSM303_DEV_ADDR_t SLAVE_ADDRESS, const uint8_t REG){
	uint8_t retval;
	i2c_m_sync_set_slaveaddr(&lsm303c_sync, SLAVE_ADDRESS, I2C_M_SEVEN);
	i2c_m_sync_cmd_read(&lsm303c_sync, REG, &retval, 1);
	return retval;
}

static uint32_t readContinous(const LSM303_DEV_ADDR_t SLAVE_ADDRESS, const uint8_t STARTING_REG, uint8_t* buf, const uint32_t LEN)
{
	uint32_t retval = I2C_ERR_BAD_ADDRESS;
	
	if(LEN < 12) {
		struct _i2c_m_msg msg;
		uint8_t tempBuf[12];
		tempBuf[0] = STARTING_REG;
	
		msg.addr   = SLAVE_ADDRESS;
		msg.len    = LEN;
		msg.flags  = I2C_M_STOP;
		msg.buffer = tempBuf;
		
		retval = _i2c_m_sync_transfer(&lsm303c_sync.device, &msg);
		
		memcpy(buf, tempBuf+1, LEN);
	}

	return retval;
}

/* @brief Read a single register
 *
 * @param SLAVE_ADDRESS [IN] The I2C Slave Address of the desired device
 * @param Reg [IN] The register to write to
 * @param val [IN] The value to write to the register
 * @return Zero if successful, otherwise an error code listed below
 *     -1: Received ACK from device on I2C bus
 *     -2: Received NACK from device on I2C bus
 *     -3: Arbitration lost
 *     -4: Bad address
 *     -5: Bus error
 *     -6: Device busy
 *     -7: Package collision
 */
 static int32_t writeReg(const LSM303_DEV_ADDR_t SLAVE_ADDRESS, const uint8_t Reg, uint8_t val)
 {
	struct _i2c_m_msg msg;
	uint8_t buff[2];
	buff[0] = Reg;
	buff[1] = val;
	
	msg.addr   = SLAVE_ADDRESS;
	msg.len    = 2;
	msg.flags  = I2C_M_STOP;
	msg.buffer = buff;
	
	return _i2c_m_sync_transfer(&lsm303c_sync.device, &msg);
}

bool acc_config(const ACC_FS_t RANGE, const ACC_BDU_t BLOCK_UPDATE, const ACC_AXIS_EN_t AXIS, const ACC_ODR_t RATE, const ACC_INCREMENT_t INCREMENT)
{
	/* Basic Read-modify-write operation to leave other values unchanged */
	volatile uint8_t reg1 = readReg(LSM303_ACCEL, ACC_CTRL1);
	volatile uint8_t reg4 = readReg(LSM303_ACCEL, ACC_CTRL4);
	volatile uint8_t reg5 = readReg(LSM303_ACCEL, ACC_CTRL5);
	
	reg1 |= (BLOCK_UPDATE | AXIS | RATE);
	reg4 |= (RANGE | INCREMENT);
	reg5 |= ACC_SELF_TEST_OFF;
	
	writeReg(LSM303_ACCEL, ACC_CTRL1, reg1);
	writeReg(LSM303_ACCEL, ACC_CTRL4, reg4);
	writeReg(LSM303_ACCEL, ACC_CTRL5, reg5);

	return true;
}

bool imu_init(struct i2c_m_sync_desc *const WIRE)
{
	lsm303c_sync  = *WIRE;
	i2c_m_sync_enable(&lsm303c_sync);
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

AxesRaw_t acc_readContinous()
{
	AxesRaw_t Axes;
	readContinous(LSM303_ACCEL, ACC_OUT_X_L, (uint8_t*)&Axes, 6);
	return Axes;
}

ACC_STATUS_FLAGS_t acc_getStatus()
{
	return readReg(LSM303_ACCEL, ACC_STATUS);
}
