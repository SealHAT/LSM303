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

 uint8_t readReg(const LSM303_DEV_ADDR_t SLAVE_ADDRESS, const uint8_t REG);

 void writeReg(const LSM303_DEV_ADDR_t SLAVE_ADDRESS, const uint8_t Reg, uint8_t val);

bool acc_config(const ACC_FS_t RANGE, const ACC_BDU_t BLOCK_UPDATE, const ACC_AXIS_EN_t AXIS, const ACC_ODR_t RATE, const ACC_INCREMENT_t INCREMENT);

bool imu_init(struct i2c_m_sync_desc *const WIRE);

AxesRaw_t acc_readXYZ();

ACC_STATUS_FLAGS_t acc_getStatus();

#endif /* LSM303_H_ */