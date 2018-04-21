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
static ACC_FULL_SCALE_t currentScale    = ACC_SCALE_2G;
static ACC_OPMODE_t     currentAccMode  = ACC_POWER_DOWN;
static MAG_OPMODE_t     currentMagMode  = MAG_IDLE;

/* Read a single register for accelerometer*/
static int32_t readReg(const LSM303_DEV_ADDR_t SLAVE_ADDRESS, const uint8_t REG, uint8_t* dest)
{
	i2c_m_sync_set_slaveaddr(&lsm303c_sync, SLAVE_ADDRESS, I2C_M_SEVEN);
	return i2c_m_sync_cmd_read(&lsm303c_sync, REG, dest, 1);
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

static int32_t readContinous(const LSM303_DEV_ADDR_t SLAVE_ADDRESS, uint8_t STARTING_REG, uint8_t* buf, const uint32_t LEN)
{
	struct  _i2c_m_msg msg;
    int32_t err;        // error return value

    STARTING_REG |= 0x80;
	
	msg.addr   = SLAVE_ADDRESS;
	msg.len    = 1;
	msg.flags  = 0;
	msg.buffer = (uint8_t*)&STARTING_REG;
	
	err = _i2c_m_sync_transfer(&lsm303c_sync.device, &msg);
    if(err != ERR_NONE) { return err; }
	
	msg.addr   = SLAVE_ADDRESS;
	msg.len    = LEN;
	msg.flags  = I2C_M_RD | I2C_M_STOP;
	msg.buffer = buf;
	
	return _i2c_m_sync_transfer(&lsm303c_sync.device, &msg);
}

int32_t lsm303_init(struct i2c_m_sync_desc *const WIRE)
{
  	lsm303c_sync  = *WIRE;
    return i2c_m_sync_enable(&lsm303c_sync);
}

int32_t lsm303_startAcc(const ACC_FULL_SCALE_t RANGE, const ACC_OPMODE_t MODE)
{
    int32_t err;        // error return value

    // set the ODR, LPen bit, and enabled axis in register 1
    uint8_t reg1 = (MODE & 0xF8) | ACC_ENABLE_ALL;

    // set the HR bit mode in register 4 with bit #2 of the MODE parameter
	uint8_t reg4 = (MODE & 0x04) << 1;
    reg4 |= (ACC_CTRL4_BDU | RANGE);

	currentScale    = RANGE;
    currentAccMode  = MODE;

	err = writeReg(LSM303_ACCEL, ACC_CTRL1, reg1);
    if(err == ERR_NONE) {
        err = writeReg(LSM303_ACCEL, ACC_CTRL4, reg4);
    }
    return err;
}

int32_t lsm303_startFIFO(ACC_FIFO_MODE_t MODE)
{	
	int32_t err = ERR_NONE;       // error return for the function
	
	uint8_t fifoctrl_reg = (ACC_FIFOCTRL_MODE & MODE);
	
	//Enable FIFO
	err |= writeReg(LSM303_ACCEL, ACC_CTRL5, ACC_CTRL5_FIFO_EN);
	err |= writeReg(LSM303_ACCEL, ACC_FIFO_CTRL, fifoctrl_reg);
	err |= writeReg(LSM303_ACCEL, ACC_CTRL3, ACC_CTRL3_I1_OVERRUN);
	return err;
}

int32_t lsm303_stopAcc()
{
    int32_t err;        // error return for the function
    uint8_t reg1;       // hold the first control register
    
    err = readReg(LSM303_ACCEL, ACC_CTRL1, &reg1);
    if(err != ERR_NONE) { return err; }

    reg1 &= ~(ACC_CTRL1_ODR);

    return writeReg(LSM303_ACCEL, ACC_CTRL1, reg1);
}

int32_t lsm303_resumeAcc()
{
    int32_t err;       // error return for the function
    uint8_t reg1;      // holds register value
    
    if(currentAccMode == ACC_POWER_DOWN) {
        err = lsm303_startAcc(ACC_SCALE_2G, ACC_HR_50_HZ);
    }
    else {
        err = readReg(LSM303_ACCEL, ACC_CTRL1, &reg1);
        if(err != ERR_NONE) { return err; }

        reg1 &= ~(ACC_CTRL1_ODR);

        err = writeReg(LSM303_ACCEL, ACC_CTRL1, reg1);
    }

    return err;
}

int32_t lsm303_startMag(const MAG_OPMODE_t MODE)
{
	int32_t err  = ERR_NONE;        // err return value

    uint8_t regA = MAG_TEMPCOMP_ENABLE | (MODE & 0x1F);
    uint8_t regB = MAG_CFGB_LOWPASS_EN;
    uint8_t regC = MAG_CFGC_BDU | MAG_CFGC_INT_MAG;

    currentMagMode = MODE;

	err |= writeReg(LSM303_MAG, MAG_CFG_A, regA);
	err |= writeReg(LSM303_MAG, MAG_CFG_B, regB);
    err |= writeReg(LSM303_MAG, MAG_CFG_C, regC);
	return err;
}

int32_t lsm303_stopMag()
{
    int32_t err;        // error return for the function
    uint8_t regA;       // hold the control register
    
    err = readReg(LSM303_MAG, MAG_CFG_A, &regA);
    if(err != ERR_NONE) { return err; }

    // no need to clear first, idle is set when both mode bits are high
    regA |= MAG_MODE_IDLE;

    return writeReg(LSM303_MAG, MAG_CFG_A, regA);
}

int32_t lsm303_resumeMag()
{
    int32_t err;       // error return for the function
    uint8_t regA;      // holds register value
    
    if(currentMagMode == MAG_IDLE) {
        err = lsm303_startMag(MAG_LP_20_HZ);
    }
    else {
        err = readReg(LSM303_MAG, MAG_CFG_A, &regA);
        if(err != ERR_NONE) { return err; }

        // clear and set
        regA &= ~(MAG_CFGA_MD | MAG_CFGA_LP);
        regA |= currentMagMode;

        err = writeReg(LSM303_MAG, MAG_CFG_A, regA);
    }

    return err;
}

int32_t lsm303_acc_dataready(void)
{
    int32_t err = ERR_NONE;     // err return value
    uint8_t statusReg;          // register

    err = readReg(LSM303_ACCEL, ACC_STATUS, &statusReg);
    if(err != ERR_NONE) { return err; }

    // return overflow error if any data overflow bits are set.
    // this also implies new data, so we make the error code positive
    if(statusReg & IMU_STATUS_DOVF) {
        return -ERR_OVERFLOW;
    }

    // return the ZYX DRDY bit, this will be a positive true value but not 1
    return (statusReg & ZYX_NEW_DATA_AVAILABLE);
}

int32_t lsm303_mag_dataready(void)
{
    int32_t err = ERR_NONE;     // err return value
    uint8_t statusReg;          // register

    err = readReg(LSM303_MAG, MAG_STATUS_REG, &statusReg);
    if(err != ERR_NONE) { return err; }

    // return overflow error if any data overflow bits are set.
    // this also implies new data, so we make the error code positive
    if(statusReg & IMU_STATUS_DOVF) {
        return -ERR_OVERFLOW;
    }

    // return the ZYX DRDY bit, this will be a positive true value but not 1
    return (statusReg & ZYX_NEW_DATA_AVAILABLE);
}

AxesRaw_t lsm303_readAcc()
{
    int32_t err  = ERR_NONE;        // catch error value
    uint_fast8_t shift = 0;         // the shift amount depends on operating mode 
	AxesRaw_t Axes;                 // the return value

    // get a new reading of raw data
	err = readContinous(LSM303_ACCEL, ACC_OUT_X_L, (uint8_t*)&Axes, 6);

    switch(currentAccMode) {
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
        default: err = ERR_INVALID_ARG;
    };
	
    if(err == ERR_NONE) {
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

AxesRaw_t lsm303_readMag()
{
    int32_t err;
	AxesRaw_t Axes;
	
    err = readContinous(LSM303_MAG, MAG_OUTX_L, (uint8_t*)&Axes, 6);

    return Axes;
}

// TODO - enable temp readings in a function, and read them correctly
int16_t lsm303_readTemp()
{
    int32_t err;
	int16_t temperature;

	err = readContinous(LSM303_ACCEL, ACC_TEMP_L, (uint8_t*)&temperature, 2);
	
    return temperature;
}

AxesSI_t lsm303_getGravity()
{
    // different Scales, there are 3 modes and 4 full scale settings. these are mg/LSB values from data sheet page 13.
	static const float scale[3][4] = {{0.98, 1.95, 3.9, 11.72},
                                      {3.9, 7.82, 15.63, 46.9},
                                      {15.63, 31.26, 62.52, 187.58}};
    int i,j;                            // index for the array above
	AxesSI_t  results;                  // stores the results of the reading
	
    // get a new reading
    AxesRaw_t accel = lsm303_readAcc();

    switch(currentAccMode) {
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