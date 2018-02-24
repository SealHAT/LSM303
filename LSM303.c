/*
 * LSM303.c
 *
 * Created: 2/23/2018 2:56:23 PM
 *  Author: hpan5
 */ 
#include "LSM303.h"
#include "math.h"

static struct{
	struct io_descriptor *imu_io;
}imu;

/* Read a single register for accelerometer*/
static uint8_t acc_readReg1(struct i2c_m_sync_desc *const wire, const uint8_t Reg){
	i2c_m_sync_set_slaveaddr(wire, ACC_I2C_ADDR, I2C_M_SEVEN);
	uint8_t retval;
	io_write(imu.imu_io, &Reg, 1);
	io_read(imu.imu_io, &retval, 1);
	return retval;
}

static void acc_readReg2(struct i2c_m_sync_desc *const wire, const uint8_t Reg, uint8_t* const first, uint8_t* const second)
{
	i2c_m_sync_set_slaveaddr(wire, ACC_I2C_ADDR, I2C_M_SEVEN);
	uint8_t retvals[2];
	io_write(imu.imu_io, &Reg, 1);
	io_read(imu.imu_io, retvals, 2);
	*first  = retvals[0];
	*second = retvals[1];
}


/* Read a single register for magnetometer*/
static uint8_t mag_readReg1(struct i2c_m_sync_desc *const wire, const uint8_t Reg){
	i2c_m_sync_set_slaveaddr(wire, MAG_I2C_ADDR, I2C_M_SEVEN);
	uint8_t retval;
	io_write(imu.imu_io, &Reg, 1);
	io_read(imu.imu_io, &retval, 1);
	return retval;
}

static void mag_readReg2(struct i2c_m_sync_desc *const wire, const uint8_t Reg, uint8_t* const first, uint8_t* const second)
{
	i2c_m_sync_set_slaveaddr(wire, MAG_I2C_ADDR, I2C_M_SEVEN);
	uint8_t retvals[2];
	io_write(imu.imu_io, &Reg, 1);
	io_read(imu.imu_io, retvals, 2);
	*first  = retvals[0];
	*second = retvals[1];
}

/* Read a single register*/
static void acc_writeReg1(struct i2c_m_sync_desc *const wire, const uint8_t Reg, const uint8_t Val){
	i2c_m_sync_set_slaveaddr(wire, ACC_I2C_ADDR, I2C_M_SEVEN);
	uint8_t sendvals[2];
	sendvals[0] = Reg;
	sendvals[1] = Val;
	io_write(imu.imu_io, sendvals, 2);
}

static void acc_writeReg2(struct i2c_m_sync_desc *const wire, const uint8_t Reg, const uint8_t first, const uint8_t second){
	i2c_m_sync_set_slaveaddr(wire, ACC_I2C_ADDR, I2C_M_SEVEN);
	uint8_t sendvals[3];
	sendvals[0] = Reg;
	sendvals[1] = first;
	sendvals[2] = second;
	io_write(imu.imu_io, sendvals, 3);
}

/* Read a single register*/
static void mag_writeReg1(struct i2c_m_sync_desc *const wire, const uint8_t Reg, const uint8_t Val){
	i2c_m_sync_set_slaveaddr(wire, MAG_I2C_ADDR, I2C_M_SEVEN);
	uint8_t sendvals[2];
	sendvals[0] = Reg;
	sendvals[1] = Val;
	io_write(imu.imu_io, sendvals, 2);
}

static void mag_writeReg2(struct i2c_m_sync_desc *const wire, const uint8_t Reg, const uint8_t first, const uint8_t second){
	i2c_m_sync_set_slaveaddr(wire, MAG_I2C_ADDR, I2C_M_SEVEN);
	uint8_t sendvals[3];
	sendvals[0] = Reg;
	sendvals[1] = first;
	sendvals[2] = second;
	io_write(imu.imu_io, sendvals, 3);
}

static void imu_init(struct i2c_m_sync_desc *const wire){
	i2c_m_sync_get_io_descriptor(wire, &imu.imu_io);
	i2c_m_sync_enable(wire);
}