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
#include "LSM303CTypes.h"

static uint8_t acc_readReg(const uint8_t Reg);

static uint8_t mag_readReg(const uint8_t Reg);

static uint8_t mag_readReg(const uint8_t Reg);

static void acc_writeReg(const uint8_t Reg, const uint8_t Val);

static void acc_init(struct i2c_m_sync_desc *const wire);

static void mag_init(struct i2c_m_sync_desc *const wire);


#endif /* LSM303_H_ */