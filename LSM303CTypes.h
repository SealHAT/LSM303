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
	ACC_CTRL1_HR		= 0x80,
	ACC_CTRL1_ODR		= 0x70,
	ACC_CTRL1_BDU		= 0x08,
	ACC_CTRL1_AXIS		= 0x07
} ACC_CTRL1_MASKS_t;

typedef enum {
	ACC_HR_DISABLE		= 0x00,
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
	ACC_CTRL2_HPM			= 0x18,
	ACC_CTRL2_FDS			= 0x04,
	ACC_CTRL2_HPIS		    = 0x03
} ACC_CTRL2_MASKS_t;

typedef enum {
	ACC_HIGHPASS_ODR_50		= 0x00,
	ACC_HIGHPASS_ODR_100	= 0x20,
	ACC_HIGHPASS_ODR_9		= 0x40,
	ACC_HIGHPASS_ODR_400	= 0x60
} ACC_HIGHPASS_SELECT_t;

typedef enum {
	ACC_HIGHPASS_NORMAL		= 0x00,	/* also valid with 0x10 */
	ACC_HIGHPASS_REFERENCE	= 0x08
} ACC_HIGHPASS_MODE_t;

typedef enum {
	ACC_FDS_INTERNAL_BYPASS	= 0x00,
	ACC_FDS_OUTPUT_REGISTER	= 0x04
} ACC_HIGHPASS_DATA_t;

typedef enum {
	ACC_ISR1_HP_BYPASS	= 0x00,
	ACC_ISR1_HP_ENABLE	= 0x01,
	ACC_ISR2_HP_BYPASS	= 0x00,
	ACC_ISR2_HP_ENABLE	= 0x02
} ACC_HIGHPASS_ISR_t;

/*****************************************/
/*** ACCELEROMETER CONTROL REGISTER #3 ***/
/*****************************************/
typedef enum {
	ACC_CTRL3_FIFOEN		= 0x80,
	ACC_CTRL3_THRESH		= 0x40,
	ACC_CTRL3_ISREN			= 0x3F
} ACC_CTRL3_MASKS_t;

typedef enum {
	ACC_FIFO_OFF			= 0x00,
	ACC_FIFO_ON				= 0x80
} ACC_FIFO_ENABLE_t;

typedef enum {
	ACC_FIFO_THRESHOLD_OFF	= 0x00,
	ACC_FIFO_THRESHOLD_ON	= 0x40
} ACC_FIFO_THRESHOLD_t;

typedef enum {
	ACC_ISR_NONE			= 0x00,
	ACC_ISR_INACTIVITYT		= 0x20,
	ACC_ISR_GENERATOR2		= 0x10,
	ACC_ISR_GENERATOR1		= 0x08,
	ACC_ISR_FIFO_OVERRUN	= 0x04,
	ACC_ISR_FIFO_THRESHOLD	= 0x02,
	ACC_ISR_DATA_READY		= 0x01
} ACC_ISR_ENABLE_t;

/*****************************************/
/*** ACCELEROMETER CONTROL REGISTER #4 ***/
/*****************************************/
typedef enum {
	ACC_CTRL4_BW			= 0xC0,
	ACC_CTRL4_FS			= 0x30,
	ACC_CTRL4_SCALEODR		= 0x08,
	ACC_CTRL4_INCREMENT		= 0x04,
	ACC_CTRL4_I2C			= 0x02,
	ACC_CTRL4_SPI			= 0x01
} ACC_CTRL4_MASKS_t;

typedef enum {
	ACC_BW_400			= 0x00,
	ACC_BW_200			= 0x10,
	ACC_BW_100			= 0x80,
	ACC_BW_50			= 0xC0
} ACC_BW_t;

typedef enum {
	ACC_FS_2G = 0x00,
	ACC_FS_4G = 0x20,
	ACC_FS_8G = 0x30
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
	ACC_DEBUG_ON			= 0x80,
	ACC_DEBUG_OFF			= 0x00
} ACC_ENABLE_DEBUG_t;

typedef enum {
	ACC_SOFT_RESET			= 0x40,
	ACC_NO_RESET			= 0x00
} ACC_SOFT_RESET_t;

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

typedef enum {
	ACC_ISR_ACTIVE_HI		= 0x00,
	ACC_ISR_ACTIVE_LOW		= 0x02
} ACC_ISR_LEVEL_t;

typedef enum {
	ACC_ISR_PUSHPULL		= 0x00,
	ACC_ISR_OPENDRAIN		= 0x01
} ACC_ISR_OUTPUT_MODE_t;

/*****************************************/
/*** ACCELEROMETER CONTROL REGISTER #6 ***/
/*****************************************/
typedef enum {
	ACC_FORCE_REBOOT	= 0x80,
	ACC_NO_REBOOT		= 0x00
} ACC_FORCE_REBOOT_t;

/*****************************************/
/*** ACCELEROMETER CONTROL REGISTER #7 ***/
/*****************************************/
typedef enum {
	ACC_DCRM1_OFF			= 0x00,
	ACC_DCRM1_ON			= 0x10,
	ACC_DCRM2_OFF			= 0x00,
	ACC_DCRM2_ON			= 0x20
} ACC_COUNT_RESET_MODE_t;

typedef enum {
	ACC_ISR1_LATCH_OFF	= 0x00,
	ACC_ISR1_LATCH_ON	= 0x04,
	ACC_ISR2_LATCH_OFF	= 0x00,
	ACC_ISR2_LATCH_ON	= 0x08
} ACC_ISR_LATCH_t;

typedef enum {
	ACC_ISR1_4D_OFF		= 0x00,
	ACC_ISR1_4D_ON		= 0x02,
	ACC_ISR2_4D_OFF		= 0x00,
	ACC_ISR2_4D_ON		= 0x01
} ACC_ISR_4D_t;

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
	MAG_CTRL1	   = 0x20,
	MAG_CTRL2      = 0x21,
	MAG_CTRL3      = 0x22,
	MAG_CTRL4      = 0x23,
	MAG_CTRL5      = 0x24,
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