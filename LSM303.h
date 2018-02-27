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

 uint8_t acc_readReg1(struct i2c_m_sync_desc *const wire, const uint8_t Reg);

 void acc_readReg2(struct i2c_m_sync_desc *const wire, const uint8_t Reg, uint8_t* const first, uint8_t* const second);

 uint8_t mag_readReg1(struct i2c_m_sync_desc *const wire, const uint8_t Reg);

 void mag_readReg2(struct i2c_m_sync_desc *const wire, const uint8_t Reg, uint8_t* const first, uint8_t* const second);

 void acc_writeReg1(struct i2c_m_sync_desc *const wire, const uint8_t Reg, const uint8_t Val);

 void acc_writeReg2(struct i2c_m_sync_desc *const wire, const uint8_t Reg, const uint8_t first, const uint8_t second);

 void mag_writeReg1(struct i2c_m_sync_desc *const wire, const uint8_t Reg, const uint8_t Val);

 void mag_writeReg2(struct i2c_m_sync_desc *const wire, const uint8_t Reg, const uint8_t first, const uint8_t second);

 void imu_init(struct i2c_m_sync_desc *const wire);

 void acc_clearREADYbit();

 void acc_readXYZ(int* X, int* Y, int* Z);
 void mag_readXYZ(int* X, int* Y, int* Z);
//int32_t acc_SelfTest();

 void mag_clearREADYbit();

 void mag_readXYZ(int* X, int* Y, int* Z);

#endif /* LSM303_H_ */