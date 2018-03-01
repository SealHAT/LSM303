// Data types used by the SparkFun LSM303C driver
// Heavily based on code from ST
#ifndef __LSM303C_TYPES_H__
#define __LSM303C_TYPES_H__


/*************************************/
/*** COMMON TYPES AND ENUMERATIONS ***/
/*************************************/
typedef enum {
	LSM303_ACCEL     = 0x1D,
	LSM303_MAG       = 0x1E
} LSM303_DEV_ADDR_t;

typedef enum {
	LSM303_READ      = 0x80,
	LSM303_WRITE     = 0x00,
	LSM303_SINGLE    = 0x00,
	LSM303_MULTI     = 0x40
} LSM303_SPI_MODES_t;

typedef enum {
	ACC_WHOAMI		= 0x41,
	MAG_WHOAMI		= 0x3D
} IMU_DEV_ID_t;

typedef enum {
	MODE_SPI,
	MODE_I2C,
} InterfaceMode_t;

typedef enum {
	MAG,
	ACC
} CHIP_t;

typedef enum {
	xAxis,
	yAxis,
	zAxis
} AXIS_t;

/***********************************************/
/***     LSM303C Accelerometer Registers     ***/
/***********************************************/
typedef enum {
	ACC_TEMP_L       = 0x0B,
	ACC_TEMP_H       = 0x0C,
	ACC_WHO_AM_I     = 0x0F,
	ACC_ACT_TSH      = 0x1E,
	ACC_ACT_DUR      = 0x1F,
	ACC_CTRL1        = 0x20,
	ACC_CTRL2        = 0x21,
	ACC_CTRL3        = 0x22,
	ACC_CTRL4        = 0x23,
	ACC_CTRL5        = 0x24,
	ACC_CTRL6        = 0x25,
	ACC_CTRL7        = 0x26,
	ACC_STATUS       = 0x27,
	ACC_OUT_X_L      = 0x28,
	ACC_OUT_X_H      = 0x29,
	ACC_OUT_Y_L      = 0x2A,
	ACC_OUT_Y_H      = 0x2B,
	ACC_OUT_Z_L      = 0x2C,
	ACC_OUT_Z_H      = 0x2D,
	ACC_FIFO_CTRL    = 0x2E,
	ACC_FIFO_SRC     = 0x2F,
	ACC_IG_CFG1      = 0x30,
	ACC_IG_SRC1      = 0x31,
	ACC_IG_THS_X1    = 0x32,
	ACC_IG_THS_Y1    = 0x33,
	ACC_IG_THS_Z1    = 0x34,
	ACC_IG_DUR1      = 0x35,
	ACC_IG_CFG2      = 0x36,
	ACC_IG_SRC2      = 0x37,
	ACC_IG_THS2      = 0x38,
	ACC_IG_DUR2      = 0x39,
	ACC_XL_REFERENCE = 0x3A,
	ACC_XH_REFERENCE = 0x3B,
	ACC_YL_REFERENCE = 0x3C,
	ACC_YH_REFERENCE = 0x3D,
	ACC_ZL_REFERENCE = 0x3E,
	ACC_ZH_REFERENCE = 0x3F
} ACC_REG_t;

/*****************************************/
/*** ACCELEROMETER CONTROL REGISTER #1 ***/
/*****************************************/
typedef enum {
	ACC_HR_DISABLE		= 0x80,
	ACC_HR_ENABLE		= 0x80,
} ACC_HR_t;

typedef enum {
	ACC_ODR_POWER_DOWN  = 0x00,
	ACC_ODR_10_Hz       = 0x10,
	ACC_ODR_50_Hz       = 0x20,
	ACC_ODR_100_Hz      = 0x30,
	ACC_ODR_200_Hz      = 0x40,
	ACC_ODR_400_Hz      = 0x50,
	ACC_ODR_800_Hz      = 0x60,
	ACC_ODR_MASK        = 0x60
} ACC_ODR_t;

typedef enum {
	ACC_BDU_DISABLE = 0x00,
	ACC_BDU_ENABLE  = 0x08
} ACC_BDU_t;

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
	ACC_CTRL2_DFC			= 0x60,
	ACC_CTRL2_HIGHPASS_NORM	= 0x00,
	ACC_CTRL2_HIGHPASS_REF	= 0x18,
	ACC_CTRL2_FDS_INTERNAL	= 0x00,
	ACC_CTRL2_FDS_OUTPUT	= 0x04,
	ACC_CTRL2_HPIS			= 0x03
} ACC_CTRL_REG2_t;

/*****************************************/
/*** ACCELEROMETER CONTROL REGISTER #3 ***/
/*****************************************/
typedef enum {
	ACC_CTRL3_FIFO_EN		= 0x80,
	ACC_CTRL3_STOP_FTH		= 0x40,
	ACC_CTRL3_INT_INTAC		= 0x20,
	ACC_CTRL3_INT_IG2		= 0x10,
	ACC_CTRL3_INT_IG1		= 0x08,
	ACC_CTRL3_INT_XL_OVR	= 0x04,
	ACC_CTRL3_INT_XL_FTH	= 0x02,
	ACC_CTRL3_INT_XL_DRDY	= 0x01
} ACC_CTRL_REG3_t;

/*****************************************/
/*** ACCELEROMETER CONTROL REGISTER #4 ***/
/*****************************************/
typedef enum {
	ACC_BW_400			= 0x00,
	ACC_BW_200			= 0x10,
	ACC_BW_100			= 0x80,
	ACC_BW_50			= 0xC0
} ACC_BW_t;

typedef enum {
	ACC_FS_2g = 0x00,
	ACC_FS_4g = 0x20,
	ACC_FS_8g = 0x30
} ACC_FS_t;

typedef enum {
	ACC_SCALE_ODR_OFF	= 0x00,
	ACC_SCALE_ODR_ON	= 0x08
} ACC_ODR_SCALE_t;

typedef enum {
	ACC_INCREMENT		= 0x04,
	ACC_NO_INCREMENT	= 0x00
} ACC_INCREMENT_t;

typedef enum {
	ACC_I2C_ON		= 0x00,
	ACC_I2C_OFF		= 0x02
} ACC_I2C_t;

typedef enum {
	ACC_SPI_OFF		= 0x00,
	ACC_SPI_ON		= 0x01
} ACC_SPI_t;

/*****************************************/
/*** ACCELEROMETER CONTROL REGISTER #5 ***/
/*****************************************/
typedef enum {
	ACC_CTRL5_DEBUG			= 0x80,
	ACC_CTRL5_SOFT_RST		= 0x40,
	ACC_CTRL5_ISR_ALOW		= 0x02,
	ACC_CTRL5_ISR_ODRAIN	= 0x01
} ACC_CTRL_REG5_t;

typedef enum {
	ACC_NO_DECIMATION		= 0x00,
	ACC_DECIMATION_2		= 0x10,
	ACC_DECIMATION_4		= 0x20,
	ACC_DECIMATION_8		= 0x30
} ACC_DECIMATION_t;

typedef enum {
	ACC_SELF_TEST_OFF		= 0x00,
	ACC_SELF_TEST_POS		= 0x04,
	ACC_SELF_TEST_NEG		= 0x08
} ACC_SELF_TEST_t;

/*****************************************/
/*** ACCELEROMETER CONTROL REGISTER #6 ***/
/*****************************************/
typedef enum {
	ACC_CTRL6_FORCE_REBOOT	= 0x80
} ACC_CTRL_REG6_t;

/*****************************************/
/*** ACCELEROMETER CONTROL REGISTER #7 ***/
/*****************************************/
typedef enum {
	ACC_CTRL7_DCRM2			= 0x20,
	ACC_CTRL7_DCRM1			= 0x10,
	ACC_CTRL7_LIR2			= 0x08,
	ACC_CTRL7_LIR1			= 0x04,
	ACC_CTRL7_4D_IG1		= 0x02,
	ACC_CTRL7_4D_IG2		= 0x01
} ACC_CTRL_REG7_t;

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
} ACC_STATUS_FLAGS_t;

typedef enum {
	ACC_FIFOCTRL_BYPASS			= 0x00,
	ACC_FIFOCTRL_FIFO			= 0x20,
	ACC_FIFOCTRL_STREAM			= 0x40,
	ACC_FIFOCTRL_STREAM2STOP	= 0x60,
	ACC_FIFOCTRL_BYPASS2STREAM	= 0x80,
	ACC_FIFOCTRL_BYPASS2FIFO	= 0xF0,
	ACC_FIFOCTRL_FTH_MASK		= 0x1F
} ACC_FIFO_CTRL_t;

typedef enum {
	ACC_FIFOSRC_FTH			= 0x80,
	ACC_FIFOSRC_OVR			= 0x40,
	ACC_FIFOSRC_EMPTY		= 0x20,
	ACC_FIFOSRC_FSS_MASK	= 0x1F
} ACC_FIFO_SRC_t;

typedef enum {
	ACC_IGCFG_ANDOR			= 0x80,
	ACC_IGCFG_6D			= 0x40,
	ACC_IGCFG_ZHI			= 0x20,
	ACC_IGCFG_ZLO			= 0x10,
	ACC_IGCFG_YHI			= 0x08,
	ACC_IGCFG_YLO			= 0x04,
	ACC_IGCFG_XHI			= 0x02,
	ACC_IGCFG_XLO			= 0x01,
} ACC_IG_CFG_t;

typedef enum {
	ACC_IGSRC_IA			= 0x40,
	ACC_IGSRC_ZHI			= 0x20,
	ACC_IGSRC_ZLO			= 0x10,
	ACC_IGSRC_YHI			= 0x08,
	ACC_IGSRC_YLO			= 0x04,
	ACC_IGSRC_XHI			= 0x02,
	ACC_IGSRC_XLO			= 0x01,
} ACC_IG_SRC_t;

typedef enum {
	ACC_IGDUR_WAIT			= 0x80
} ACC_IG_DUR_t;

/***********************************************/
/***     LSM303C Magnetometer Registers      ***/
/***********************************************/
typedef enum {
	MAG_WHO_AM_I   = 0x0F,
	MAG_CTRL_REG1  = 0x20,
	MAG_CTRL_REG2  = 0x21,
	MAG_CTRL_REG3  = 0x22,
	MAG_CTRL_REG4  = 0x23,
	MAG_CTRL_REG5  = 0x24,
	MAG_STATUS_REG = 0x27,
	MAG_OUTX_L     = 0x28,
	MAG_OUTX_H     = 0x29,
	MAG_OUTY_L     = 0x2A,
	MAG_OUTY_H     = 0x2B,
	MAG_OUTZ_L     = 0x2C,
	MAG_OUTZ_H     = 0x2D,
	MAG_TEMP_OUT_L = 0x2E,
	MAG_TEMP_OUT_H = 0x2F,
	MAG_INT_CFG    = 0x30,
	MAG_INT_SRC    = 0x31,
	MAG_INT_THS_L  = 0x32,
	MAG_INT_THS_H  = 0x33
} MAG_REG_t;

/*****************************************/
/*** MAGNETOMETER CONTROL REGISTER #1  ***/
/*****************************************/
typedef enum {
	MAG_TEMP_EN_DISABLE = 0x00,
	MAG_TEMP_EN_ENABLE  = 0x80
} MAG_TEMP_EN_t;

typedef enum {
	MAG_SELFTEST		= 0x01
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
	MAG_CTRL2_REBOOT	= 0x08,
	MAG_CTRL2_SOFT_RST	= 0x04
} MAG_CTRL_REG2_t;

/*****************************************/
/*** MAGNETOMETER CONTROL REGISTER #3  ***/
/*****************************************/
typedef enum {
	MAG_CTRL_REG3_I2C_OFF	 = 0x80,
	MAG_CTRL_REG3_LOWPOWER	 = 0x20,
	MAG_CTRL_REG3_SPI_ON	 = 0x04
} MAG_CTRL_REG3_t;

typedef enum {
	MAG_MD_CONTINUOUS   = 0x00,
	MAG_MD_SINGLE       = 0x01,
	MAG_MD_POWER_DOWN_1 = 0x02,
	MAG_MD_POWER_DOWN_2 = 0x03
} MAG_MD_t;

/*****************************************/
/*** MAGNETOMETER CONTROL REGISTER #4  ***/
/*****************************************/
typedef enum {
	MAG_OMZ_LOW_PW                  =  0x00,
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
typedef enum {
	MAG_INTCFG_XEN			= 0x80,
	MAG_INTCFG_YEN			= 0x40,
	MAG_INTCFG_ZEN			= 0x20,
	MAG_INTCFG_MUSTSET		= 0x08,
	MAG_INTCFG_IEA			= 0x04,
	MAG_INTCFG_IEL			= 0x02,
	MAG_INTCFG_IEN			= 0x01
} MAG_INT_CFG_t;

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