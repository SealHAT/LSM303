/*
 * LSM303.h
 *
 * Created: 2/23/2018 2:58:51 PM
 *  Author: hpan5
 */ 


#ifndef LSM303_H_
#define LSM303_H_

#include <atmel_start.h>	/* where the IO functions live */
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include "LSM303CTypes.h"

typedef struct {
	int16_t xAxis;
	int16_t yAxis;
	int16_t zAxis;
} AxesRaw_t;

typedef struct {
	ACC_HR_t         highResMode;	/* CTRL_REG1:  */
	ACC_ODR_t        sampleRate;	/* CTRL_REG1:  */
	ACC_BDU_t        BlockUpdate;	/* CTRL_REG1:  */
	ACC_AXIS_EN_t    enableAxis;	/* CTRL_REG1:  */
	ACC_CTRL_REG2_t  ctrlReg2;		/* CTRL_REG2:  */
	ACC_CTRL_REG3_t  ctrlReg3;		/* CTRL_REG3:  */
	ACC_BW_t		 antiAliasing;	/* CTRL_REG4:  */
	ACC_FS_t         fullScale;		/* CTRL_REG4:  */
	ACC_ODR_SCALE_t  odrScaling;	/* CTRL_REG4:  */
	ACC_INCREMENT_t  autoIncrement;	/* CTRL_REG4:  */
	ACC_I2C_t        i2cMode;		/* CTRL_REG4:  */
	ACC_SPI_t        spiMode;		/* CTRL_REG4:  */
	ACC_CTRL_REG5_t  ctrlReg5;		/* CTRL_REG5:  */
	ACC_DECIMATION_t decimation;	/* CTRL_REG5:  */
	ACC_SELF_TEST_t  selfTest;		/* CTRL_REG5:  */
	ACC_CTRL_REG6_t  forceReboot;	/* CTRL_REG6:  */
	ACC_CTRL_REG7_t  ctrlReg7;		/* CTRL_REG7:  */
} AccelConfig_t;

typedef struct {
	MAG_TEMP_EN_t    highResMode;	/* CTRL_REG1:  */
	MAG_SELF_TEST_t  selfTest;		/* CTRL_REG1:  */
	MAG_OMXY_t       xyMode;		/* CTRL_REG1:  */
	MAG_DO_t         sampleRate;	/* CTRL_REG1:  */
	MAG_FS_t         fullScale;     /* CTRL_REG2:  */
	MAG_RESET_t      reboot;		/* CTRL_REG2:  */
	MAG_LOWPOWER_t   lowpower;		/* CTRL_REG3:  */
	MAG_I2C_t        i2cMode;		/* CTRL_REG3:  */
	MAG_SPI_t        spiMode;		/* CTRL_REG3:  */
	MAG_MD_t         operationMode; /* CTRL_REG3:  */
} AccelConfig_t;

bool imu_init(struct i2c_m_sync_desc *const WIRE);

bool acc_config(const ACC_FS_t RANGE, const ACC_BDU_t BLOCK_UPDATE, const ACC_AXIS_EN_t AXIS, const ACC_ODR_t RATE, const ACC_INCREMENT_t INCREMENT);

ACC_STATUS_FLAGS_t acc_getStatus();

AxesRaw_t acc_read();

bool mag_config();

AxesRaw_t mag_read();

#endif /* LSM303_H_ */