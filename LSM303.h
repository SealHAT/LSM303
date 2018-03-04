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
#include "LSM303CTypes.h"

typedef struct {
	int16_t xAxis;
	int16_t yAxis;
	int16_t zAxis;
} AxesRaw_t;

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

bool lsm303_init(struct i2c_m_sync_desc *const WIRE);

bool lsm303_startAcc(const ACC_FS_t RANGE, const ACC_ODR_t RATE);

bool lsm303_startMag(const MAG_MODE_t MODE, const MAG_DO_t RATE, const MAG_TEMP_EN_t TEMPERATURE);

IMU_STATUS_t lsm303_statusAcc();

IMU_STATUS_t lsm303_statusMag();

AxesRaw_t lsm303_readAcc();

AxesRaw_t lsm303_readMag();

int16_t lsm303_readTemp();

float lsm303_getGravity(const int16_t axis);

float lsm303_getGauss(const int16_t axis);

inline float lsm303_getCelcius(const int16_t TEMP) { return (TEMP / 8.0) + 25.0;  }

#endif /* LSM303_H_ */