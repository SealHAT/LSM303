// Data types used by the SparkFun LSM303C driver
// Heavily based on code from ST
#ifndef __LSM303C_TYPES_H__
#define __LSM303C_TYPES_H__


/*************************************/
/*** COMMON TYPES AND ENUMERATIONS ***/
/*************************************/
typedef enum {
	LSM303_ACCEL     = 0x19,
	LSM303_MAG       = 0x1E
} LSM303_DEV_ADDR_t;

typedef enum {
	LSM303_READ      = 0x80,
	LSM303_WRITE     = 0x00,
	LSM303_SINGLE    = 0x00,
	LSM303_MULTI     = 0x40
} LSM303_SPI_MODES_t;

typedef enum {
	ACC_LSM303AGR	= 0x33,
	MAG_LSM303AGR	= 0x40
} IMU_DEV_ID_t;

typedef struct {
	uint8_t reg1;
	uint8_t reg2;
	uint8_t reg3;
	uint8_t reg4;
	uint8_t reg5;
	uint8_t reg6;
	uint8_t reg7;
} accConfig_t;

typedef struct {
	uint8_t reg1;
	uint8_t reg2;
	uint8_t reg3;
	uint8_t reg4;
	uint8_t reg5;
} magConfig_t;

/***********************************************/
/***     LSM303C Accelerometer Registers     ***/
/***********************************************/
typedef enum {
    // 00 - 06 Reserved
	ACC_STATUS_AUX_A = 0x07,
    // 08 - 0B Reserved
	ACC_TEMP_L       = 0x0C,
	ACC_TEMP_H       = 0x0D,
	ACC_COUNT_REG	 = 0x0E,
    ACC_WHO_AM_I     = 0x0F,
    // 10 - 1E Reserved
	ACC_TEMP_CFG	 = 0x1F,
	ACC_CTRL1        = 0x20,
	ACC_CTRL2        = 0x21,
	ACC_CTRL3        = 0x22,
	ACC_CTRL4        = 0x23,
	ACC_CTRL5        = 0x24,
	ACC_CTRL6        = 0x25,
	ACC_REF          = 0x26,
	ACC_STATUS       = 0x27,
	ACC_OUT_X_L      = 0x28,
	ACC_OUT_X_H      = 0x29,
	ACC_OUT_Y_L      = 0x2A,
	ACC_OUT_Y_H      = 0x2B,
	ACC_OUT_Z_L      = 0x2C,
	ACC_OUT_Z_H      = 0x2D,
	ACC_FIFO_CTRL    = 0x2E,
	ACC_FIFO_SRC     = 0x2F,
	ACC_INT1_CFG     = 0x30,
	ACC_INT1_SRC     = 0x31,
    ACC_INT1_THS     = 0x32,
    ACC_INT1_DUR     = 0x33,
    ACC_INT2_CFG     = 0x34,
    ACC_INT2_SRC     = 0x35,
    ACC_INT2_THS     = 0x36,
    ACC_INT2_DUR     = 0x37,
    ACC_CLICK_CFG    = 0x38,
    ACC_CLICK_SRC    = 0x39,
    ACC_CLICK_THS    = 0x3A,
    ACC_TIME_LIM     = 0x3B,
    ACC_TIME_LATENCY = 0x3C,
    ACC_TIME_WINDOW  = 0x3D,
    ACC_ACT_TSH      = 0x3E,
    ACC_ACT_DUR      = 0x3F
} ACC_REG_t;

/*******************************************************/
/*** ACCELEROMETER AUX STATUS REGISTER (TEMPERATURE) ***/
/*******************************************************/
typedef enum {
    ACC_TMP_OVERRUN     = 0x40,
    ACC_TMP_NEW         = 0x04
} ACC_STATUS_AUX_A_MASKS_t;

typedef enum {
    ACC_TMP_ENABLE     = 0xC0,
    ACC_TMP_DISABLE    = 0x00
} ACC_TEMP_CFG_t;

/*****************************************/
/*** ACCELEROMETER CONTROL REGISTER #1 ***/
/*****************************************/
typedef enum {
	ACC_CTRL1_ODR		= 0xF0,
	ACC_CTRL1_LOWPWR    = 0x08,
	ACC_CTRL1_ZEN		= 0x04,
    ACC_CTRL1_YEN		= 0x02,
    ACC_CTRL1_XEN		= 0x01
} ACC_CTRL1_MASKS_t;

typedef enum {
	ACC_ODR_POWER_DOWN  = 0x00,
    ACC_ODR_1_HZ        = 0x01,
    ACC_ODR_10_HZ       = 0x02,
    ACC_ODR_25_HZ       = 0x03,
    ACC_ODR_50_HZ       = 0x04,
    ACC_ODR_100_HZ      = 0x05,
    ACC_ODR_200_HZ      = 0x06,
    ACC_ODR_400_HZ      = 0x07,
    ACC_ODRLP_1620_HZ   = 0x08,
    ACC_ODRLP_5376_HZ   = 0x09
} ACC_ODR_t;

typedef enum {
	ACC_DISABLE_ALL = 0x00,
	ACC_X_ENABLE    = 0x01,
	ACC_Y_ENABLE    = 0x02,
	ACC_Z_ENABLE    = 0x04,
	ACC_ENABLE_ALL	= 0x07
} ACC_AXIS_EN_t;

/*****************************************/
/*** ACCELEROMETER CONTROL REGISTER #2 ***/
/*****************************************/
typedef enum {
	ACC_CTRL2_HPM			= 0xC0,
    ACC_CTRL2_HPCF          = 0x30, /* Values unknown? no table in datasheet */
    ACC_CTRL2_FDS           = 0x08,
    ACC_CTRL2_HPCLICK       = 0x04,
    ACC_CTRL2_HPIS2         = 0x02,
    ACC_CTRL2_HPIS1         = 0x01
} ACC_CTRL2_MASKS_t;

typedef enum {
	ACC_HIGHPASS_NORMAL		= 0x00,	/* also valid with 0x10 */
	ACC_HIGHPASS_REFERENCE	= 0x80,
    ACC_HIGHPASS_INTERRUPT  = 0xC0
} ACC_HIGHPASS_MODE_t;

/*****************************************/
/*** ACCELEROMETER CONTROL REGISTER #3 ***/
/*****************************************/
typedef enum {
	ACC_CTRL3_I1_CLICK		= 0x80,
    ACC_CTRL3_I1_AOI1		= 0x40,
    ACC_CTRL3_I1_AOI2		= 0x20,
    ACC_CTRL3_I1_DRDY1		= 0x10,
    ACC_CTRL3_I1_DRDY2		= 0x08,
    ACC_CTRL3_I1_WTM		= 0x04,
    ACC_CTRL3_I1_OVERRUN	= 0x02,
} ACC_CTRL3_MASKS_t;

/*****************************************/
/*** ACCELEROMETER CONTROL REGISTER #4 ***/
/*****************************************/
typedef enum {
    ACC_CTRL4_BDU           = 0x80,
    ACC_CTRL4_BLE           = 0x40, /* Only available in high-res mode */
    ACC_CTRL4_FS            = 0x30,
    ACC_CTRL4_HR            = 0x08, /* Must be set in conjunction with  */
    ACC_CTRL4_ST            = 0x06,
    ACC_CTRL4_SPI_EN        = 0x01
} ACC_CTRL4_MASKS_t;

typedef enum {
	ACC_FS_2G                   = 0x00,
	ACC_FS_4G                   = 0x10,
	ACC_FS_8G                   = 0x20,
    ACC_FS_16G                  = 0x30
} ACC_FS_t;

typedef enum {
    ACC_SELFTEST_OFF            = 0x00,
    ACC_SELFTEST_1              = 0x40,
    ACC_SELFTEST_2              = 0x60,
} ACC_FS_t;

typedef enum {
	ACC_SPI_OFF		        = 0x00,
	ACC_SPI_ON		        = 0x01
} ACC_SPI_t;

/*****************************************/
/*** ACCELEROMETER CONTROL REGISTER #5 ***/
/*****************************************/
typedef enum {
    ACC_CTRL5_BOOT          = 0x80, 
    ACC_CTRL5_FIFO_EN       = 0x40,
    ACC_CTRL5_LIR_INT1      = 0x08,
    ACC_CTRL5_D4D_INT1      = 0x04,
    ACC_CTRL5_LIR_INT2      = 0x02,
    ACC_CTRL5_D4D_INT2      = 0x01
} ACC_CTRL5_MASKS_t;

typedef enum {
    ACC_FIFO_ON		        = 0x40,
    ACC_FIFO_OFF	        = 0x00
} ACC_FIFO_t;

/*****************************************/
/*** ACCELEROMETER CONTROL REGISTER #6 ***/
/*****************************************/
typedef enum {
    ACC_CTRL6_I2_CLICK_EN   = 0x80,
    ACC_CTRL6_I2_INT1       = 0x40,
    ACC_CTRL6_I2_INT2       = 0x20,
    ACC_CTRL6_I2_BOOT       = 0x10,
    ACC_CTRL6_I2_ACT        = 0x08,
    ACC_CTRL6_I2_ACTIVE_MD  = 0x02
} ACC_CTRL6_MASKS_t;

/*****************************************/
/*** ACCELEROMETER STATUS REGISTER     ***/
/*****************************************/
typedef enum {
	ACC_NULL_STATUS				= 0x00,
	ACC_X_NEW_DATA_AVAILABLE    = 0x01,
	ACC_Y_NEW_DATA_AVAILABLE    = 0x02,
	ACC_Z_NEW_DATA_AVAILABLE    = 0x04,
	ACC_ZYX_NEW_DATA_AVAILABLE  = 0x08,
	ACC_X_OVERRUN               = 0x10,
	ACC_Y_OVERRUN               = 0x20,
	ACC_Z_OVERRUN               = 0x40,
	ACC_ZYX_OVERRUN             = 0x80
} ACC_STATUS_t;

/*****************************************/
/*** ACCELEROMETER FIFO CONTROL        ***/
/*****************************************/
typedef enum {
	ACC_FIFOCTRL_MODE       = 0xC0,
    ACC_FIFOCTRL_TRIG       = 0x40,
    ACC_FIFOCTRL_THRESH     = 0x1F
} ACC_FIFO_CTRL_MASKS_t;

typedef enum {
	ACC_FIFO_BYPASS         = 0x00,
    ACC_FIFO_FIFO           = 0x40,
    ACC_FIFO_STREAM         = 0x80,
    ACC_FIFO_STRM2FIFO      = 0xC0,
} ACC_FIFO_MODE_t;

/*****************************************/
/*** ACCELEROMETER FIFO SOURCE         ***/
/*****************************************/
typedef enum {
    ACC_FIFOSRC_WTM         = 0x80, // set high when FIFO contents exceed watermark
    ACC_FIFOSRC_OVRN        = 0x40, // set high when the FIFO is full (32 samples) and the NEXT reading will overwrite the oldest
    ACC_FIFOSRC_EMPTY       = 0x20, // set high when all samples are read and FIFo is empty
    ACC_FIFOSRC_FSS         = 0x1F, // the current number of unread samples
} ACC_FIFO_SRC_MASKS_t;

/*****************************************/
/*** ACCELEROMETER INTTERUPT CONFIG  ***/
/*****************************************/
typedef enum {
    ACC_INTCFG_AOI          = 0x80,
    ACC_INTCFG_6D           = 0x40,
    ACC_INTCFG_ZHIE         = 0x20,
    ACC_INTCFG_ZLIE         = 0x10,
    ACC_INTCFG_YHIE         = 0x08,
    ACC_INTCFG_YLIE         = 0x04,
    ACC_INTCFG_XHIE         = 0x02,
    ACC_INTCFG_XLIE         = 0x01,
} ACC_INT_CFG_MASKS_t;

typedef enum {
    ACC_INTMODE_OR          = 0x00,
    ACC_INTMODE_6DMOVE      = 0x40,
    ACC_INTMODE_AND         = 0x80,
    ACC_INTMODE_6DPOS       = 0xC0
} ACC_INT_MODE_t;

/*****************************************/
/*** ACCELEROMETER INTTERUPT SOURCE    ***/
/*****************************************/
typedef enum {
    ACC_INTSRC_IA           = 0x40,
    ACC_INTSRC_ZH           = 0x20,
    ACC_INTSRC_ZL           = 0x10,
    ACC_INTSRC_YH           = 0x08,
    ACC_INTSRC_YL           = 0x04,
    ACC_INTSRC_XH           = 0x02,
    ACC_INTSRC_XL           = 0x01,
} ACC_INT_SRC_MASKS_t;

/********************************************/
/*** ACCELEROMETER INTTERUPT THRESHOLD  ***/
/********************************************/
typedef enum {
    ACC_THS                 = 0x7F  // based on LSB value for each Full Scale Setting, see datasheet
} ACC_THRESHOLD_MASKS_t;

/********************************************/
/*** ACCELEROMETER INTTERUPT DURATION  ***/
/********************************************/
typedef enum {
    ACC_DUR                 = 0x7F  // time measured in N/ODR where N is the contents of this register
} ACC_DURATION_MASKS_t;

/********************************************/
/*** ACCELEROMETER CLICK CONFIGURATION  ***/
/********************************************/
typedef enum {
    ACC_CLICKCFG_ZDIE       = 0x20, // double click on Z axis above threshold
    ACC_CLICKCFG_ZSIE       = 0x10, // single click on Z axis above threshold
    ACC_CLICKCFG_YDIE       = 0x08, // double click on Y axis above threshold
    ACC_CLICKCFG_YSIE       = 0x04, // single click on Y axis above threshold
    ACC_CLICKCFG_XDIE       = 0x02, // double click on X axis above threshold
    ACC_CLICKCFG_XSIE       = 0x01, // single click on X axis above threshold
} ACC_CLICK_CFG_MASKS_t;

/********************************************/
/*** ACCELEROMETER CLICK SOURCE           ***/
/********************************************/
typedef enum {
    ACC_CLICKSRC_IA         = 0x40,
    ACC_CLICKSRC_DCLICKEN   = 0x20,
    ACC_CLICKSRC_SCLICKEN   = 0x10,
    ACC_CLICKSRC_SIGN       = 0x08,
    ACC_CLICKSRC_Z          = 0x04,
    ACC_CLICKSRC_Y          = 0x02,
    ACC_CLICKSRC_X          = 0x01
} ACC_CLICK_SRC_MASKS_t;

/***********************************************/
/***     LSM303C Magnetometer Registers      ***/
/***********************************************/
typedef enum {
    // 40 - 44 Reserved
    MAG_OFFSET_X_LO = 0x45,
    MAG_OFFSET_X_HI = 0x46,
    MAG_OFFSET_Y_LO = 0x47,
    MAG_OFFSET_Y_HI = 0x48,
    MAG_OFFSET_Z_LO = 0x49,
    MAG_OFFSET_Z_HI = 0x4A,
    // 4B - 4C Reserved
	MAG_WHO_AM_I    = 0x4F,
    // 50 - 5F Reserved
	MAG_CFG_A	    = 0x60,
    MAG_CFG_B	    = 0x61,
    MAG_CFG_C	    = 0x62,
    MAG_INT_CTRL    = 0x63,
    MAG_INT_SRC     = 0x64,
    MAG_INT_THR_L   = 0x65,
    MAG_INT_THR_H   = 0x66,
    MAG_STATUS_REG  = 0x67,
    MAG_OUTX_L      = 0x68,
    MAG_OUTX_H      = 0x69,
    MAG_OUTY_L      = 0x6A,
    MAG_OUTY_H      = 0x6B,
    MAG_OUTZ_L      = 0x6C,
    MAG_OUTZ_H      = 0x6D,
    // 6E - 6F Reserved
} MAG_REG_t;

/*****************************************/
/*** MAGNETOMETER CONTROL REGISTER #1  ***/
/*****************************************/
typedef enum {
	MAG_CTRL1_TEMP			= 0x80,
	MAG_CTRL1_OMXY			= 0x60,
	MAG_CTRL1_DO			= 0x1C,
	MAG_CTRL1_SELFTEST		= 0x01
} MAG_CTRL1_MASKS_t;

typedef enum {
	MAG_TEMP_DISABLE = 0x00,
	MAG_TEMP_ENABLE  = 0x80
} MAG_TEMP_EN_t;

typedef enum {
	MAG_SELFTEST_OFF	= 0x00,
	MAG_SELFTEST_ON		= 0x01
} MAG_SELF_TEST_t;

typedef enum {
	MAG_OMXY_LOW_POWER              = 0x00,
	MAG_OMXY_MEDIUM_PERFORMANCE     = 0x20,
	MAG_OMXY_HIGH_PERFORMANCE       = 0x40,
	MAG_OMXY_ULTRA_HIGH_PERFORMANCE = 0x60
} MAG_OMXY_t;

typedef enum {
	MAG_DO_0_625_Hz = 0x00,
	MAG_DO_1_25_Hz  = 0x04,
	MAG_DO_2_5_Hz   = 0x08,
	MAG_DO_5_Hz     = 0x0C,
	MAG_DO_10_Hz    = 0x10,
	MAG_DO_20_Hz    = 0x14,
	MAG_DO_40_Hz    = 0x18,
	MAG_DO_80_Hz    = 0x1C
} MAG_DO_t;

/*****************************************/
/*** MAGNETOMETER CONTROL REGISTER #2  ***/
/*****************************************/
typedef enum {
	MAG_FS_16_Ga  =  0x60
} MAG_FS_t;

typedef enum {
	MAG_RESET_OFF		= 0x00,
	MAG_REBOOT			= 0x08,
	MAG_SOFT_RST		= 0x04
} MAG_RESET_t;

/*****************************************/
/*** MAGNETOMETER CONTROL REGISTER #3  ***/
/*****************************************/
typedef enum {
	MAG_CTRL3_I2C			= 0x80,
	MAG_CTRL3_LOWPOWER		= 0x20,
	MAG_CTRL3_SPI			= 0x04,
	MAG_CTRL3_MODE			= 0x03
} MAG_CTRL3_MASKS_t;

typedef enum {
	MAG_I2C_ON		= 0x00,
	MAG_I2C_OFF		= 0x80
} MAG_I2C_t;

typedef enum {
	MAG_LOWPOWER_OFF = 0x00,
	MAG_LOWPOWER_ON	 = 0x20
} MAG_LOWPOWER_t;

typedef enum {
	MAG_SPI_OFF		= 0x00,
	MAG_SPI_ON		= 0x04
} MAG_SPI_t;

typedef enum {
	MAG_MODE_CONTINUOUS   = 0x00,
	MAG_MODE_SINGLE       = 0x01,
	MAG_MODE_OFF		  = 0x03
} MAG_MODE_t;

/*****************************************/
/*** MAGNETOMETER CONTROL REGISTER #4  ***/
/*****************************************/
typedef enum {
	MAG_CTRL4_OMZ					= 0x0C,
	MAG_CTRL4_ENDIAN				= 0x02
} MAG_CTRL4_MASKS_t;

typedef enum {
	MAG_OMZ_LOW_POWER               =  0x00,
	MAG_OMZ_MEDIUM_PERFORMANCE      =  0x04,
	MAG_OMZ_HIGH_PERFORMANCE        =  0x08,
	MAG_OMZ_ULTRA_HIGH_PERFORMANCE  =  0x0C
} MAG_OMZ_t;

typedef enum {
	MAG_BIG_ENDIAN		= 0x00,  /* Unsure if this is BYTE or bits endian-ness */
	MAG_LITTLE_ENDIAN	= 0x02
} MAG_ENDIAN_t;

/*****************************************/
/*** MAGNETOMETER CONTROL REGISTER #5  ***/
/*****************************************/
typedef enum {
	MAG_BDU_DISABLE = 0x00,
	MAG_BDU_ENABLE  = 0x40
} MAG_BDU_t;

/*****************************************/
/***   MAGNETOMETER STATUS REGISTER    ***/
/*****************************************/
typedef enum {
	MAG_NULL_STATUS			= 0x00,
	MAG_STATUS_ZYXOR		= 0x80,
	MAG_STATUS_ZOR			= 0x40,
	MAG_STATUS_YOR			= 0x20,
	MAG_STATUS_XOR			= 0x10,
	MAG_STATUS_ZYXDA		= 0x08,
	MAG_STATUS_ZDA			= 0x04,
	MAG_STATUS_YDA			= 0x02,
	MAG_STATUS_XDA			= 0x01
} MAG_STATUS_t;

typedef enum {
	MAG_XYZDA_NO  = 0x00,
	MAG_XYZDA_YES = 0x08
} MAG_XYZDA_t;

/*****************************************/
/***   MAGNETOMETER INTERRUPT CONFIG   ***/
/*****************************************/
#define MAG_ISRCONFIG_MUSTSET		(0x08)

typedef enum {
	MAG_INTCFG_XEN			= (0x80 | MAG_ISRCONFIG_MUSTSET),
	MAG_INTCFG_YEN			= (0x40 | MAG_ISRCONFIG_MUSTSET),
	MAG_INTCFG_ZEN			= (0x20 | MAG_ISRCONFIG_MUSTSET)
} MAG_INT_CFG_t;

typedef enum {
	MAG_ISR_ACTIVE_LO		= (0x00 | MAG_ISRCONFIG_MUSTSET),
	MAG_ISR_ACTIVE_HI		= (0x04 | MAG_ISRCONFIG_MUSTSET)
} MAG_ISR_LEVEL_t;

typedef enum {
	MAG_ISR2_LATCH_ON		= (0x00 | MAG_ISRCONFIG_MUSTSET),
	MAG_ISR2_LATCH_OFF		= (0x02 | MAG_ISRCONFIG_MUSTSET)
} MAG_ISR_LATCH_t;

typedef enum {
	MAG_ISR_DISABLE			= (0x00 | MAG_ISRCONFIG_MUSTSET),
	MAG_ISR_ENABLE			= (0x01 | MAG_ISRCONFIG_MUSTSET)
} MAG_ISR_ENABLE_t;

/*****************************************/
/***   MAGNETOMETER INTERRUPT SOURCE   ***/
/*****************************************/
typedef enum {
	MAG_INTSRC_PTHX			= 0x80,
	MAG_INTSRC_PTHY			= 0x40,
	MAG_INTSRC_PTHZ			= 0x20,
	MAG_INTSRC_NTHX			= 0x10,
	MAG_INTSRC_NTHY			= 0x08,
	MAG_INTSRC_NTHZ			= 0x04,
	MAG_INTSRC_MROI			= 0x02,
	MAG_INTSRC_INT			= 0x01
} MAG_INT_SRC_t;

#endif