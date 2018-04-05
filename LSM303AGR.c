/*
 * LSM303.c
 *
 * Created: 2/23/2018 2:56:23 PM
 *  Author: hpan5
 */ 
#include "LSM303AGR.h"
#include "LSM303AGRTypes.h"
#include "math.h"

static struct i2c_m_sync_desc lsm303c_sync; /* Structure for IMU communications */
static ACC_FS_t currentScale;

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

	return true;
}

bool lsm303_startAcc(const IMU_AXIS_t AXIS, const ACC_FULL_SCALE_t RANGE, const ACC_OPMODE_t MODE)
{
	uint8_t reg1 = 0x00;
	uint8_t reg4 = 0x00;

    // set the ODR and the LPen bit in register 1
    reg1 |= (MODE & 0xF8);
    // set the axis to enable in register 1
    reg1 |= AXIS;

    // set the HR bit mode in register 4 with bit #2 of the MODE parameter
    reg4 |= (MODE & 0x04) << 1;
    reg4 |= (ACC_CTRL4_BDU | RANGE);

	currentScale = RANGE;

	writeReg(LSM303_ACCEL, ACC_CTRL1, reg1);
	writeReg(LSM303_ACCEL, ACC_CTRL4, reg4);
	return true;
}

bool lsm303_startMag(const MAG_OPMODE_t MODE)
{
	uint8_t regA = MAG_TEMPCOMP_ENABLE | (MODE & 0x1F);
    uint8_t regB = MAG_CFGB_LOWPASS_EN;
    uint8_t regC = MAG_CFGC_BDU | MAG_CFGC_INT_MAG;

	writeReg(LSM303_MAG, MAG_CFG_A, regA);
	writeReg(LSM303_MAG, MAG_CFG_B, regB);
    writeReg(LSM303_MAG, MAG_CFG_C, regC);
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

// TODO - enable temp readings in a function, and read them correctly
int16_t lsm303_readTemp()
{
	int16_t temperature;
	readContinous(LSM303_ACCEL, ACC_TEMP_L, (uint8_t*)&temperature, 2);
	return temperature;
}

// TODO - scale is dependant on rate AND power mode. possibly set the scale in the mode function?
AxesSI_t lsm303_getGravity()
{
	float scale;
	AxesSI_t  results;
	AxesRaw_t accel = lsm303_readAcc();
	
	switch(currentScale) {
		case ACC_FS_2G: scale = 0.061;
						break;
		case ACC_FS_4G: scale = 0.122;
						break;
		case ACC_FS_8G: scale = 0.244;
						break;
        case ACC_FS_16G: scale = 0.0;
                        break;
		default: scale = 0.0;
	};
	
	results.xAxis = ( accel.xAxis * scale / 1000.0);
	results.yAxis = ( accel.yAxis * scale / 1000.0);
	results.zAxis = ( accel.zAxis * scale / 1000.0);
	
	return results;
}

AxesSI_t lsm303_getGauss()
{
	AxesSI_t  results;
	AxesRaw_t mag = lsm303_readMag();
	
	results.xAxis = (mag.xAxis * 0.58 / 1000.0);
	results.yAxis = (mag.yAxis * 0.58 / 1000.0);
	results.zAxis = (mag.zAxis * 0.58 / 1000.0);
	
	return results;
}