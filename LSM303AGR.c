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

int32_t lsm303_acc_startBypass(const ACC_FULL_SCALE_t RANGE, const ACC_OPMODE_t MODE)
{
    int32_t err;        // error return value

    // reboot accelerometer memory contents
    err = writeReg(LSM303_ACCEL, ACC_CTRL5, ACC_CTRL5_BOOT);

    // set the ODR, LPen bit, and enabled axis in register 1
    uint8_t reg1 = (MODE & 0xF8) | ACC_ENABLE_ALL;

    // set the HR bit mode in register 4 with bit #2 of the MODE parameter
	uint8_t reg4 = (MODE & 0x04) << 1;
    reg4 |= (ACC_CTRL4_BDU | RANGE);

	currentScale    = RANGE;
    currentAccMode  = MODE;

    // write the control registers
    err = writeReg(LSM303_ACCEL, ACC_CTRL3, ACC_CTRL3_I1_DRDY1);
    if(err < 0) { return err; }

    err = writeReg(LSM303_ACCEL, ACC_CTRL4, reg4);
    if(err < 0) { return err; }

    err = writeReg(LSM303_ACCEL, ACC_CTRL1, reg1);
    return err;
}

int32_t lsm303_acc_startFIFO(const ACC_FULL_SCALE_t RANGE, const ACC_OPMODE_t MODE)
{
    int32_t err;        // error return value

    // set FIFO to bypass and watermark to zero to get rid of any existing watermark interrupts
    err = writeReg(LSM303_ACCEL, ACC_FIFO_CTRL, ACC_FIFO_BYPASS);
    if(err < 0) { return err; }

    // reboot accelerometer memory contents
    err = writeReg(LSM303_ACCEL, ACC_CTRL5, ACC_CTRL5_BOOT);
    if(err < 0) { return err; }

    // set the HR bit mode in register 4 with bit #2 of the MODE parameter
	uint8_t reg4 = (MODE & 0x04) << 1;
    reg4 |= (ACC_CTRL4_BDU | RANGE);

    err = writeReg(LSM303_ACCEL, ACC_CTRL4, reg4);
    if(err < 0) { return err; }

    // enable the FIFO
    err = writeReg(LSM303_ACCEL, ACC_CTRL5, ACC_CTRL5_FIFO_EN);
    if(err < 0) { return err; }

    // set FIFO watermark to 25 and set to stream mode
    err = writeReg(LSM303_ACCEL, ACC_FIFO_CTRL, (ACC_FIFO_STREAM|0x19));
    if(err < 0) { return err; }

    // set watermark interrupt on pin 1
    err = writeReg(LSM303_ACCEL, ACC_CTRL3, ACC_CTRL3_I1_WTM);
    if(err < 0) { return err; }

    // set the ODR, LPen bit, and enabled axis in register 1
    err = writeReg(LSM303_ACCEL, ACC_CTRL1, (MODE & 0xF8) | ACC_ENABLE_ALL);

	currentScale    = RANGE;
    currentAccMode  = MODE;

    return err;
}

int32_t lsm303_acc_stop(void)
{
    int32_t err;        // error return for the function
    uint8_t reg1;       // hold the first control register

    err = readReg(LSM303_ACCEL, ACC_CTRL1, &reg1);

    if(!err) {
        reg1 &= ~(ACC_CTRL1_ODR);
        err = writeReg(LSM303_ACCEL, ACC_CTRL1, reg1);
    }

    return err;
}

int32_t lsm303_acc_motionDetectStart(const uint8_t sensitivity, uint16_t threshold, uint8_t duration)
{
	int32_t err = ERR_NONE; // error return for the function
    uint8_t ctrl5;

	// verify the threshold argument. do this first since it can set error to ERR_NONE
    switch(currentScale) {
		case ACC_FS_2G:  err = (threshold <= 2000u ? ERR_NONE : ERR_INVALID_ARG);
                         threshold /= 16u;
		                 break;
		case ACC_FS_4G:  err = (threshold <= 4000u ? ERR_NONE : ERR_INVALID_ARG);
                         threshold /= 32u;
		                 break;
        // because the LSB value is 62 and our max setting is 0x7F, 7874 is the max threshold in 8g mode
		case ACC_FS_8G:  err = (threshold <= 8000u ? ERR_NONE : ERR_INVALID_ARG);
                         threshold  = (threshold <= 7874u ? threshold : 7874u);
                         threshold /= 62u;
		                 break;
		case ACC_FS_16G: err = (threshold <= 16000u ? ERR_NONE : ERR_INVALID_ARG);
                         threshold /= 186u;
		                 break;
		default: err = ERR_FAILURE;
	};

    // verify the duration argument
    if(duration > 127) {
        err = ERR_INVALID_ARG;
    }

    if(!err) {
//         err = readReg(LSM303_ACCEL, ACC_CTRL5, &ctrl5);
        readReg(LSM303_ACCEL, ACC_INT1_SRC, &ctrl5);

        if(!err) {
	        err = writeReg(LSM303_ACCEL, ACC_INT1_THS, (threshold & ACC_THS_MASK));
        }

        if(!err) {
            err = writeReg(LSM303_ACCEL, ACC_INT1_DUR, (duration  & ACC_DUR_MASK));
        }

        if(!err) {
            err = writeReg(LSM303_ACCEL, ACC_INT1_CFG, (/*ACC_INTMODE_6DMOVE |*/ (sensitivity & ACC_INTCFG_AXIS_MASK)));
        }

//         if(!err) {
//             err = writeReg(LSM303_ACCEL, ACC_CTRL5, (ctrl5 | ACC_CTRL5_D4D_INT2));
//         }

        if(!err) {
            err = writeReg(LSM303_ACCEL, ACC_CTRL6, ACC_CTRL6_I2_INT1);
        }
    }

	return err;
}

int32_t lsm303_acc_motionDetectRead(uint8_t* detect)
{
    return readReg(LSM303_ACCEL, ACC_INT1_SRC, detect);
}

int32_t lsm303_mag_start(const MAG_OPMODE_t MODE)
{
	int32_t err;        // err return value

    // reset the magnetometer memories then wait for restart
    err = writeReg(LSM303_MAG, MAG_CFG_A, MAG_CFGA_SOFTRST);
    if(err < 0) { return err; }
    delay_ms(5);

    // set BDU and enable interrupt
    err = writeReg(LSM303_MAG, MAG_CFG_C, (MAG_CFGC_BDU | MAG_CFGC_INT_MAG) );
    if(err < 0) { return err; }

    // enable low pass filter
    err |= writeReg(LSM303_MAG, MAG_CFG_B, 0x00 /* MAG_CFGB_LOWPASS_EN */);
    if(err < 0) { return err; }

    // enable temperature compensation and set the mode and rate
    err |= writeReg(LSM303_MAG, MAG_CFG_A, (MAG_TEMPCOMP_ENABLE | (MODE & 0x1F)) );
    if(err < 0) { return err; }

    currentMagMode = MODE;
	return err;
}

int32_t lsm303_mag_stop(void)
{
    int32_t err;        // error return for the function
    uint8_t regA;       // hold the control register

    err = readReg(LSM303_MAG, MAG_CFG_A, &regA);

    if(!err) {
        // no need to clear first, idle is set when both mode bits are high
        regA |= MAG_MODE_IDLE;
        err = writeReg(LSM303_MAG, MAG_CFG_A, regA);
    }

    return err;
}


int32_t lsm303_acc_dataready(void)
{
    int32_t err;            // err return value
    uint8_t statusReg;      // register

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
    int32_t err;            // err return value
    uint8_t statusReg;      // register

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

int32_t lsm303_acc_rawRead(AxesRaw_t* rawRead)
{
    // get a new reading of raw data
	return readContinous(LSM303_ACCEL, ACC_OUT_X_L, (uint8_t*)rawRead, 6);
}

int32_t lsm303_acc_FIFOWatermark(bool* overflow)
{
	int32_t err;               // error return for the function
    uint8_t statusfifo_reg;

    err = readReg(LSM303_ACCEL, ACC_FIFO_SRC, &statusfifo_reg);

    if(ERR_NONE == err) {
        err = (statusfifo_reg & ACC_FIFOSRC_WTM);

        // set overflow if not NULL
        if(overflow != NULL) {
            *overflow = (statusfifo_reg & ACC_FIFOSRC_OVRN);
        }
    }
	return err;
}

int32_t lsm303_acc_FIFOOverrun(void)
{
    int32_t err;               // error return for the function
	uint8_t statusfifo_reg;

    err = readReg(LSM303_ACCEL, ACC_FIFO_SRC, &statusfifo_reg);

    if(ERR_NONE == err) {
        err = (statusfifo_reg & ACC_FIFOSRC_OVRN);
    }
	return err;
}

int32_t lsm303_acc_FIFOEmpty(void)
{
    int32_t err;               // error return for the function
	uint8_t statusfifo_reg;

	err = readReg(LSM303_ACCEL, ACC_FIFO_SRC, &statusfifo_reg);

    if(ERR_NONE == err) {
        err = (statusfifo_reg & ACC_FIFOSRC_EMPTY);
    }
	return err;
}

int32_t lsm303_acc_FIFOCount(void)
{
    int32_t err;               // error return for the function
	uint8_t statusfifo_reg;

    err = readReg(LSM303_ACCEL, ACC_FIFO_SRC, &statusfifo_reg);

    if(ERR_NONE == err) {
        err = (statusfifo_reg & ACC_FIFOSRC_FSS);
    }
    return err;
}

int32_t lsm303_acc_FIFOread(AxesRaw_t* buf, const uint32_t LEN, bool* overrun)
{
	int32_t err;        // catch error value
	uint8_t	count;      // the number of unread samples, and fifo status register

    // get the FIFO status, return error code if I2C fails
    err = readReg(LSM303_ACCEL, ACC_FIFO_SRC, &count);
    if(err != ERR_NONE) { return err; }

    // set the overrun flag if it is not null
    if(overrun != NULL) {
        *overrun = count & ACC_FIFOSRC_OVRN;
    }

    // get the number of unread samples by masking the lower 5 bits
	count = count & ACC_FIFOSRC_FSS;

	// adjust read size to be the smallest of the buffer length or available samples
    count = (count >= LEN ? LEN : count);

    // read the number of samples calculated
	err = readContinous(LSM303_ACCEL, ACC_OUT_X_L, (uint8_t*)buf, count*6);

    // return the error code, or the number of samples read if there is no error
    err = (err == ERR_NONE ? count*6 : err);
	return err;
}

int32_t lsm303_mag_rawRead(AxesRaw_t* rawMag)
{
    return readContinous(LSM303_MAG, MAG_OUTX_L, (uint8_t*)rawMag, 6);
}

AxesSI_t lsm303_acc_getSI(AxesRaw_t* rawAccel)
{
    // different Scales, there are 3 modes and 4 full scale settings. these are mg/LSB values from data sheet page 13.
	static const float scale[3][4] = {{0.98, 1.95, 3.9, 11.72},
                                      {3.9, 7.82, 15.63, 46.9},
                                      {15.63, 31.26, 62.52, 187.58}};

    int i;                      // holds index for power mode
    int j;                      // index for scale setting
    uint_fast8_t shift = 0;     // the shift amount depends on operating mode
    AxesSI_t siAccel;           // the return value

    switch(currentAccMode) {
        case ACC_HR_1_HZ:
        case ACC_HR_10_HZ:
        case ACC_HR_25_HZ:
        case ACC_HR_50_HZ:
        case ACC_HR_100_HZ:
        case ACC_HR_200_HZ:
        case ACC_HR_400_HZ:
        case ACC_HR_1344_HZ: i = 0;
                             shift = 4;
                             break;
        case ACC_NORM_1_HZ:
        case ACC_NORM_10_HZ:
        case ACC_NORM_25_HZ:
        case ACC_NORM_50_HZ:
        case ACC_NORM_100_HZ:
        case ACC_NORM_200_HZ:
        case ACC_NORM_400_HZ:
        case ACC_NORM_1344_HZ: i = 1;
                               shift = 6;
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
                             shift = 8;
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
    if(i >= 3 || j >= 4) {
        siAccel.xAxis = NAN;
        siAccel.yAxis = NAN;
        siAccel.zAxis = NAN;
    }

    // shift values to proper placement
    rawAccel->xAxis >>= shift;
    rawAccel->yAxis >>= shift;
    rawAccel->zAxis >>= shift;

    // calculate the axis in Gs
	siAccel.xAxis = ( rawAccel->xAxis * scale[i][j] / 1000.0);
	siAccel.yAxis = ( rawAccel->yAxis * scale[i][j] / 1000.0);
	siAccel.zAxis = ( rawAccel->zAxis * scale[i][j] / 1000.0);

	return siAccel;
}

AxesSI_t lsm303_mag_getSI(AxesRaw_t* rawMag)
{
    AxesSI_t siMag;        // return value

    siMag.xAxis = (rawMag->xAxis * 1.5 / 1000.0);
    siMag.yAxis = (rawMag->yAxis * 1.5 / 1000.0);
    siMag.zAxis = (rawMag->zAxis * 1.5 / 1000.0);

	return siMag;
}

// TODO - enable temp readings in a function, and read them correctly
int32_t lsm303_readTemp(int16_t* temperature)
{
    return readContinous(LSM303_ACCEL, ACC_TEMP_L, (uint8_t*)temperature, 2);
}