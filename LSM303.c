/*
 * LSM303.c
 *
 * Created: 2/23/2018 2:56:23 PM
 *  Author: hpan5
 */ 
#include "LSM303.h"
#include "math.h"

static struct i2c_m_sync_desc lsm303c_sync; /* Structure for IMU communications */
static ACC_FS_t currentScale;

static const accConfig_t accDefault = {
	.reg1 = ACC_HR_DISABLE | ACC_ODR_POWER_DOWN | ACC_BDU_ENABLE | ACC_ENABLE_ALL,
	.reg2 = ACC_HIGHPASS_ODR_50 | ACC_HIGHPASS_NORMAL | ACC_FDS_INTERNAL_BYPASS | ACC_ISR1_HP_BYPASS | ACC_ISR2_HP_BYPASS,
	.reg3 = ACC_FIFO_OFF | ACC_FIFO_THRESHOLD_OFF | ACC_ISR_NONE,
	.reg4 = ACC_BW_400 | ACC_FS_2G | ACC_SCALE_ODR_OFF | ACC_INCREMENT | ACC_I2C_ON | ACC_SPI_OFF,
	.reg5 = ACC_DEBUG_OFF | ACC_NO_RESET | ACC_NO_DECIMATION | ACC_SELF_TEST_OFF | ACC_ISR_ACTIVE_HI | ACC_ISR_PUSHPULL,
	.reg6 = ACC_NO_REBOOT,
	.reg7 = ACC_DCRM1_OFF | ACC_DCRM2_OFF | ACC_ISR1_LATCH_OFF | ACC_ISR2_LATCH_OFF | ACC_ISR1_4D_OFF | ACC_ISR2_4D_OFF
};

static const magConfig_t magDefault = {
	.reg1 = MAG_TEMP_DISABLE | MAG_SELFTEST_OFF | MAG_OMXY_LOW_POWER | MAG_DO_10_Hz,
	.reg2 = MAG_FS_16_Ga | MAG_RESET_OFF,
	.reg3 = MAG_I2C_ON | MAG_LOWPOWER_OFF | MAG_SPI_OFF | MAG_MODE_OFF,
	.reg4 = MAG_OMZ_LOW_POWER | MAG_BIG_ENDIAN,
	.reg5 = MAG_BDU_ENABLE
};

/* Read a single register for accelerometer*/
 static uint8_t readReg(const LSM303_DEV_ADDR_t SLAVE_ADDRESS, const uint8_t REG){
	uint8_t retval;
	i2c_m_sync_set_slaveaddr(&lsm303c_sync, SLAVE_ADDRESS, I2C_M_SEVEN);
	i2c_m_sync_cmd_read(&lsm303c_sync, REG, &retval, 1);
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

static uint32_t readContinous(const LSM303_DEV_ADDR_t SLAVE_ADDRESS, const uint8_t STARTING_REG, uint8_t* buf, const uint32_t LEN)
{
	uint32_t retval = I2C_ERR_BAD_ADDRESS;
	struct _i2c_m_msg msg;
	
	msg.addr   = SLAVE_ADDRESS;
	msg.len    = 1;
	msg.flags  = 0;
	msg.buffer = (uint8_t*)&STARTING_REG;
	
	retval = _i2c_m_sync_transfer(&lsm303c_sync.device, &msg);
	
	msg.addr   = SLAVE_ADDRESS;
	msg.len    = LEN;
	msg.flags  = I2C_M_RD | I2C_M_STOP;
	msg.buffer = buf;
	
	retval = _i2c_m_sync_transfer(&lsm303c_sync.device, &msg);

	return retval;
}

bool lsm303_init(struct i2c_m_sync_desc *const WIRE)
{
	lsm303c_sync  = *WIRE;
	i2c_m_sync_enable(&lsm303c_sync);
	
	/* Configure Accelerometer with default settings */
	writeReg(LSM303_ACCEL, ACC_CTRL1, accDefault.reg1);
	writeReg(LSM303_ACCEL, ACC_CTRL2, accDefault.reg2);
	writeReg(LSM303_ACCEL, ACC_CTRL3, accDefault.reg3);
	writeReg(LSM303_ACCEL, ACC_CTRL4, accDefault.reg4);
	writeReg(LSM303_ACCEL, ACC_CTRL5, accDefault.reg5);
	writeReg(LSM303_ACCEL, ACC_CTRL6, accDefault.reg6);
	writeReg(LSM303_ACCEL, ACC_CTRL7, accDefault.reg7);
	
	currentScale = (accDefault.reg4 & ACC_CTRL4_FS);
	
	/* Configure Magnetometer with default settings */
	writeReg(LSM303_MAG, MAG_CTRL1, magDefault.reg1);
	writeReg(LSM303_MAG, MAG_CTRL2, magDefault.reg2);
	writeReg(LSM303_MAG, MAG_CTRL3, magDefault.reg3);
	writeReg(LSM303_MAG, MAG_CTRL4, magDefault.reg4);
	writeReg(LSM303_MAG, MAG_CTRL5, magDefault.reg5);
	
	return true;
}

bool lsm303_startAcc(const ACC_FS_t RANGE, const ACC_ODR_t RATE)
{
	uint8_t reg1 = accDefault.reg1;
	uint8_t reg4 = accDefault.reg4;

	currentScale = RANGE;
	
	reg1 &= ~(ACC_CTRL1_ODR);
	reg4 &= ~(ACC_CTRL4_FS);
	reg1 |= (RATE);
	reg4 |= (RANGE);

	writeReg(LSM303_ACCEL, ACC_CTRL1, reg1);
	writeReg(LSM303_ACCEL, ACC_CTRL4, reg4);
	return true;
}

bool lsm303_startMag(const MAG_MODE_t MODE, const MAG_DO_t RATE, const MAG_TEMP_EN_t TEMPERATURE)
{
	uint8_t reg1 = readReg(LSM303_MAG, MAG_CTRL1);
	uint8_t	reg3 = readReg(LSM303_MAG, MAG_CTRL3);
	
	reg1 &= ~(MAG_CTRL1_DO | MAG_CTRL1_TEMP);
	reg3 &= ~(MAG_CTRL3_MODE);
	reg1 |= (RATE | TEMPERATURE);
	reg3 |= (MODE);

	writeReg(LSM303_MAG, MAG_CTRL1, reg1);
	writeReg(LSM303_MAG, MAG_CTRL3, reg3);
	return true;
}

IMU_STATUS_t lsm303_statusAcc()
{
	return (IMU_STATUS_t)readReg(LSM303_ACCEL, ACC_STATUS);
}

IMU_STATUS_t lsm303_statusMag()
{
	return (IMU_STATUS_t)readReg(LSM303_MAG, MAG_STATUS_REG);
}

AxesRaw_t lsm303_readAcc()
{
	AxesRaw_t Axes;
	readContinous(LSM303_ACCEL, ACC_OUT_X_L, (uint8_t*)&Axes, 6);
	return Axes;
}

AxesRaw_t lsm303_readMag()
{
	AxesRaw_t Axes;
	readContinous(LSM303_MAG, MAG_OUTX_L, (uint8_t*)&Axes, 6);
	return Axes;
}

int16_t lsm303_readTemp()
{
	int16_t temperature;
	readContinous(LSM303_MAG, MAG_TEMP_OUT_L, (uint8_t*)&temperature, 2);
	return temperature;
}

float lsm303_getGravity(const int16_t AXIS)
{
	float scale;
	
	switch(currentScale) {
		case ACC_FS_2G: scale = 0.061;
						break;
		case ACC_FS_4G: scale = 0.122;
						break;
		case ACC_FS_8G: scale = 0.244;
						break;
		default: scale = 0.0;
	};
	
	return (AXIS * scale / 1000.0);
}

float lsm303_getGauss(const int16_t AXIS)
{
	return (AXIS * 0.58 / 1000.0);
}