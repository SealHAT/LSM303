/*
 * LSM303.c
 *
 * Created: 2/23/2018 2:56:23 PM
 *  Author: hpan5
 */ 
#include "LSM303.h"
#include "math.h"

static struct{
	struct io_descriptor *acc_io;
	struct io_descriptor *mag_io;
}imu;

/* Read a single register for accelerometer*/
static uint8_t acc_readReg(const uint8_t Reg){
	uint8_t retval;
	io_write(imu.acc_io, &Reg, 1);
	io_read(imu.acc_io, &retval, 1);
	return retval;
}

/* Read a single register for magnetometer*/
static uint8_t mag_readReg(const uint8_t Reg){
	uint8_t retval;
	io_write(imu.mag_io, &Reg, 1);
	io_read(imu.mag_io, &retval, 1);
	return retval;
}

/* Read a single register*/
static void acc_writeReg(const uint8_t Reg, const uint8_t Val){
	uint8_t sendvals[2];
	sendvals[0] = Reg;
	sendvals[1] = Val;
	io_read(imu.acc_io, sendvals, 2);
}

static void acc_init(struct i2c_m_sync_desc *const wire){
	i2c_m_sync_get_io_descriptor(wire, &imu.acc_io);
	i2c_m_sync_enable(wire);
	i2c_m_sync_set_slaveaddr(wire, ACC_I2C_ADDR, I2C_M_SEVEN);
}

static void mag_init(struct i2c_m_sync_desc *const wire){
	i2c_m_sync_get_io_descriptor(wire, &imu.mag_io);
	i2c_m_sync_enable(wire);
	i2c_m_sync_set_slaveaddr(wire, MAG_I2C_ADDR, I2C_M_SEVEN);
}
