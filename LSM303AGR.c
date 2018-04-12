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
static ACC_FULL_SCALE_t currentScale = ACC_SCALE_2G;
static ACC_OPMODE_t     currentMode  = ACC_POWER_DOWN;

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

static uint32_t readContinous(const LSM303_DEV_ADDR_t SLAVE_ADDRESS, uint8_t STARTING_REG, uint8_t* buf, const uint32_t LEN)
{
	uint32_t retval = I2C_ERR_BAD_ADDRESS;
	struct _i2c_m_msg msg;

    STARTING_REG |= 0x80;
	
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
	int32_t err  = 0;       // error return for the function

    // set the ODR, LPen bit, and enabled axis in register 1
    uint8_t reg1 = (MODE & 0xF8) | AXIS;

    // set the HR bit mode in register 4 with bit #2 of the MODE parameter
	uint8_t reg4 = (MODE & 0x04) << 1;
    reg4 |= (ACC_CTRL4_BDU | RANGE);

	currentScale = RANGE;
    currentMode  = MODE;

	err |= writeReg(LSM303_ACCEL, ACC_CTRL1, reg1);
	err |= writeReg(LSM303_ACCEL, ACC_CTRL4, reg4);
	return (err == 0);
}

bool lsm303_stopAcc()
{
    int32_t err  = 0;       // error return for the function
    uint8_t reg1 = readReg(LSM303_ACCEL, ACC_CTRL1);

    reg1 &= ~(ACC_CTRL1_ODR);

    err = writeReg(LSM303_ACCEL, ACC_CTRL1, reg1);
    return (err == 0);
}

bool lsm303_resumeAcc()
{
    int32_t err  = 0;       // error return for the function
    uint8_t reg1 = readReg(LSM303_ACCEL, ACC_CTRL1);

    if(currentMode == ACC_POWER_DOWN) {
        lsm303_startAcc(AXIS_ENABLE_ALL, ACC_SCALE_2G, ACC_HR_50_HZ);
    }
    else {
        reg1 &= ~(ACC_CTRL1_ODR);
    }

    err = writeReg(LSM303_ACCEL, ACC_CTRL1, reg1);
    return (err == 0);
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

ACC_FIFO_STATUS_t lsm303_statusFIFO()
{
	return (ACC_FIFO_STATUS_t)readReg(LSM303_ACCEL, ACC_FIFO_SRC);
}

IMU_STATUS_t lsm303_statusMag()
{
	return (IMU_STATUS_t)readReg(LSM303_MAG, MAG_STATUS_REG);
}

AxesRaw_t lsm303_readAcc()
{
    int32_t err  = 0;       // catch error value
    uint_fast8_t shift = 0; // the shift amount depends on operating mode 
	AxesRaw_t Axes;         // the return value

    // get a new reading of raw data
	err = readContinous(LSM303_ACCEL, ACC_OUT_X_L, (uint8_t*)&Axes, 6);

    switch(currentMode) {
        case ACC_HR_1_HZ:
        case ACC_HR_10_HZ:
        case ACC_HR_25_HZ:
        case ACC_HR_50_HZ:
        case ACC_HR_100_HZ:
        case ACC_HR_200_HZ:
        case ACC_HR_400_HZ:
        case ACC_HR_1344_HZ: shift = 4;
                             break;
        case ACC_NORM_1_HZ:
        case ACC_NORM_10_HZ:
        case ACC_NORM_25_HZ:
        case ACC_NORM_50_HZ:
        case ACC_NORM_100_HZ:
        case ACC_NORM_200_HZ:
        case ACC_NORM_400_HZ:
        case ACC_NORM_1344_HZ: shift = 6;
                               break;
        case ACC_LP_1_HZ:
        case ACC_LP_10_HZ:
        case ACC_LP_25_HZ:
        case ACC_LP_50_HZ:
        case ACC_LP_100_HZ:
        case ACC_LP_200_HZ:
        case ACC_LP_400_HZ:
        case ACC_LP_1620_HZ:
        case ACC_LP_5376_HZ: shift = 8;
                             break;
        default: err = -1;
    };
	
    if(err == 0) {
        Axes.xAxis >>= shift;
        Axes.yAxis >>= shift;
        Axes.zAxis >>= shift;
    }
    else {
        Axes.xAxis = 0xFF;
        Axes.yAxis = 0xFF;
        Axes.zAxis = 0xFF;
    }

    return Axes;
}

bool lsm303_setFIFOenabled()
{
	int32_t err  = 0;       // error return for the function
	uint8_t fifoenable_reg = readReg(LSM303_ACCEL, ACC_CTRL5);
	uint8_t fifomode_reg = readReg(LSM303_ACCEL, ACC_FIFO_CTRL);
	
	fifoenable_reg |= ACC_CTRL5_FIFO_EN;	//Enable FIFO
	fifomode_reg |= ACC_FIFO_STREAM;	//Set FIFO to stream mode
	
	err |= writeReg(LSM303_ACCEL, ACC_CTRL5, fifoenable_reg);	
	err |= writeReg(LSM303_ACCEL, ACC_FIFO_CTRL, fifomode_reg);	
	
	return (err == 0);
}


bool lsm303_setFIFOmode(mode)
{
	int32_t err  = 0;       // error return for the function
	
	//Enable FIFO
	err |= writeReg(LSM303_ACCEL, ACC_CTRL5, ACC_CTRL5_FIFO_EN);
	
	return (err == 0);
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

AxesSI_t lsm303_getGravity()
{
    // different Scales, there are 3 modes and 4 full scale settings. these are mg/LSB values from datasheet page 13.
	static const float scale[3][4] = {{0.98, 1.95, 3.9, 11.72},
                                      {3.9, 7.82, 15.63, 46.9},
                                      {15.63, 31.26, 62.52, 187.58}};
    int i,j;                            // index for the array above
	AxesSI_t  results;                  // stores the results of the reading
	
    // get a new reading
    AxesRaw_t accel = lsm303_readAcc();

    switch(currentMode) {
        case ACC_HR_1_HZ:
        case ACC_HR_10_HZ:
        case ACC_HR_25_HZ:
        case ACC_HR_50_HZ:
        case ACC_HR_100_HZ:
        case ACC_HR_200_HZ:
        case ACC_HR_400_HZ:
        case ACC_HR_1344_HZ: i = 0;
                             break;
        case ACC_NORM_1_HZ:
        case ACC_NORM_10_HZ:
        case ACC_NORM_25_HZ:
        case ACC_NORM_50_HZ:
        case ACC_NORM_100_HZ:
        case ACC_NORM_200_HZ:
        case ACC_NORM_400_HZ:
        case ACC_NORM_1344_HZ: i = 1;
                               break;
        case ACC_LP_1_HZ:
        case ACC_LP_10_HZ:
        case ACC_LP_25_HZ:
        case ACC_LP_50_HZ:
        case ACC_LP_100_HZ:
        case ACC_LP_200_HZ:
        case ACC_LP_400_HZ:
        case ACC_LP_1620_HZ:
        case ACC_LP_5376_HZ: i = 2;
                               break;
        default: i = 3;
    };

	switch(currentScale) {
    	case ACC_FS_2G:  j = 0;
    	break;
    	case ACC_FS_4G:  j = 1;
    	break;
    	case ACC_FS_8G:  j = 2;
    	break;
    	case ACC_FS_16G: j = 3;
    	break;
    	default: j = 4;
	};
	
    // return error value if the index are out of range
    if(i >= 3 || j >= 4 || accel.xAxis == 0xFF) {
        results.xAxis = NAN;
        results.yAxis = NAN;
        results.zAxis = NAN;
        return results;
    }

    // calculate the axis in Gs
	results.xAxis = ( accel.xAxis * scale[i][j] / 1000.0);
	results.yAxis = ( accel.yAxis * scale[i][j] / 1000.0);
	results.zAxis = ( accel.zAxis * scale[i][j] / 1000.0);
	
	return results;
}

AxesSI_t lsm303_getGauss()
{
	AxesSI_t  results;
	AxesRaw_t mag = lsm303_readMag();
	
	results.xAxis = (mag.xAxis * 1.5 / 1000.0);
	results.yAxis = (mag.yAxis * 1.5 / 1000.0);
	results.zAxis = (mag.zAxis * 1.5 / 1000.0);
	
	return results;
}