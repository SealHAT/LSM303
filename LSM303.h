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
	AxesRaw_t acc;
	AxesRaw_t mag;
	int16_t temp;
} ImuReading_t;

typedef enum {
	NULL_STATUS				= 0x00,
	X_NEW_DATA_AVAILABLE    = 0x01,
	Y_NEW_DATA_AVAILABLE    = 0x02,
	Z_NEW_DATA_AVAILABLE    = 0x04,
	ZYX_NEW_DATA_AVAILABLE  = 0x08,
	X_OVERRUN               = 0x10,
	Y_OVERRUN               = 0x20,
	Z_OVERRUN               = 0x40,
	ZYX_OVERRUN             = 0x80
} IMU_STATUS_t;

bool imu_init(struct i2c_m_sync_desc *const WIRE);

bool acc_config(const ACC_FS_t RANGE, const ACC_BDU_t BLOCK_UPDATE, const ACC_AXIS_EN_t AXIS, const ACC_ODR_t RATE, const ACC_INCREMENT_t INCREMENT);

IMU_STATUS_t acc_getStatus();

AxesRaw_t acc_read();

bool mag_config(const MAG_TEMP_EN_t TEMP, const MAG_DO_t DO, const MAG_FS_t FS, const MAG_MD_t MD, const MAG_BDU_t BDU);

IMU_STATUS_t mag_getStatus();

AxesRaw_t mag_read();

ImuReading_t imu_read();

#endif /* LSM303_H_ */