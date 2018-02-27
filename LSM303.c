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
 uint8_t acc_readReg1(struct i2c_m_sync_desc *const wire, const uint8_t Reg){
	i2c_m_sync_set_slaveaddr(wire, ACC_I2C_ADDR, I2C_M_SEVEN);
	uint8_t retval;
	io_write(imu.imu_io, &Reg, 1);
	io_read(imu.imu_io, &retval, 1);
	return retval;
}

 void acc_readReg2(struct i2c_m_sync_desc *const wire, const uint8_t Reg, uint8_t* const first, uint8_t* const second)
{
	i2c_m_sync_set_slaveaddr(wire, ACC_I2C_ADDR, I2C_M_SEVEN);
	uint8_t retvals[2];
	io_write(imu.imu_io, &Reg, 1);
	io_read(imu.imu_io, retvals, 2);
	*first  = retvals[0];
	*second = retvals[1];
}


/* Read a single register for magnetometer*/
 uint8_t mag_readReg1(struct i2c_m_sync_desc *const wire, const uint8_t Reg){
	i2c_m_sync_set_slaveaddr(wire, MAG_I2C_ADDR, I2C_M_SEVEN);
	uint8_t retval;
	io_write(imu.imu_io, &Reg, 1);
	io_read(imu.imu_io, &retval, 1);
	return retval;
}

 void mag_readReg2(struct i2c_m_sync_desc *const wire, const uint8_t Reg, uint8_t* const first, uint8_t* const second)
{
	i2c_m_sync_set_slaveaddr(wire, MAG_I2C_ADDR, I2C_M_SEVEN);
	uint8_t retvals[2];
	io_write(imu.imu_io, &Reg, 1);
	io_read(imu.imu_io, retvals, 2);
	*first  = retvals[0];
	*second = retvals[1];
}

/* Read a single register*/
 void acc_writeReg1(struct i2c_m_sync_desc *const wire, const uint8_t Reg, const uint8_t Val){
	i2c_m_sync_set_slaveaddr(wire, ACC_I2C_ADDR, I2C_M_SEVEN);
	uint8_t sendvals[2];
	sendvals[0] = Reg;
	sendvals[1] = Val;
	io_write(imu.imu_io, sendvals, 2);
}

 void acc_writeReg2(struct i2c_m_sync_desc *const wire, const uint8_t Reg, const uint8_t first, const uint8_t second){
	i2c_m_sync_set_slaveaddr(wire, ACC_I2C_ADDR, I2C_M_SEVEN);
	uint8_t sendvals[3];
	sendvals[0] = Reg;
	sendvals[1] = first;
	sendvals[2] = second;
	io_write(imu.imu_io, sendvals, 3);
}

/* Read a single register*/
 void mag_writeReg1(struct i2c_m_sync_desc *const wire, const uint8_t Reg, const uint8_t Val){
	i2c_m_sync_set_slaveaddr(wire, MAG_I2C_ADDR, I2C_M_SEVEN);
	uint8_t sendvals[2];
	sendvals[0] = Reg;
	sendvals[1] = Val;
	io_write(imu.imu_io, sendvals, 2);
}

 void mag_writeReg2(struct i2c_m_sync_desc *const wire, const uint8_t Reg, const uint8_t first, const uint8_t second){
	i2c_m_sync_set_slaveaddr(wire, MAG_I2C_ADDR, I2C_M_SEVEN);
	uint8_t sendvals[3];
	sendvals[0] = Reg;
	sendvals[1] = first;
	sendvals[2] = second;
	io_write(imu.imu_io, sendvals, 3);
}

 void imu_init(struct i2c_m_sync_desc *const wire){
	i2c_m_sync_get_io_descriptor(wire, &imu.imu_io);
	i2c_m_sync_enable(wire);
}

 void acc_clearREADYbit()
{
	acc_readReg1(&wire,ACC_OUT_X_L);
	acc_readReg1(&wire,ACC_OUT_X_H);
	acc_readReg1(&wire,ACC_OUT_Y_L);
	acc_readReg1(&wire,ACC_OUT_Y_H);
	acc_readReg1(&wire,ACC_OUT_Z_L);
	acc_readReg1(&wire,ACC_OUT_Z_H);
}

 void acc_readXYZ(int* X, int* Y, int* Z)
{
	uint8_t valX[2];
	uint8_t valY[2];
	uint8_t valZ[2];
	
	valX[0] = acc_readReg1(&wire,ACC_OUT_X_L);
	valX[1] = acc_readReg1(&wire,ACC_OUT_X_H);
	*X = (valX[0] | (valX[1]<<8));
	if(*X >=32768)
	{
		*X -= 65536;
	}
	
	valY[0] = acc_readReg1(&wire,ACC_OUT_Y_L);
	valY[1] = acc_readReg1(&wire,ACC_OUT_Y_H);
	*Y = (valY[0] | (valY[1]<<8));
	if(*Y >=32768)
	{
		*Y -= 65536;
	}
	
	valZ[0] = acc_readReg1(&wire,ACC_OUT_Z_L);
	valZ[1] = acc_readReg1(&wire,ACC_OUT_Z_H);
	*Z = (valZ[0] | (valZ[1]<<8));
	if(*Z >=32768)
	{
		*Z -= 65536;
	}
}

 void mag_clearREADYbit()
{
	mag_readReg1(&wire,MAG_OUTX_L);
	mag_readReg1(&wire,MAG_OUTX_H);
	mag_readReg1(&wire,MAG_OUTY_L);
	mag_readReg1(&wire,MAG_OUTY_H);
	mag_readReg1(&wire,MAG_OUTZ_L);
	mag_readReg1(&wire,MAG_OUTZ_H);
}

 void mag_readXYZ(int* X, int* Y, int* Z)
{
	uint8_t valX[2];
	uint8_t valY[2];
	uint8_t valZ[2];
	
	valX[0] = mag_readReg1(&wire,MAG_OUTX_L);
	valX[1] = mag_readReg1(&wire,MAG_OUTX_H);
	*X = (valX[0] | (valX[1]<<8));
	if(*X >=32768)
	{
		*X -= 65536;
	}
	
	valY[0] = mag_readReg1(&wire,MAG_OUTY_L);
	valY[1] = mag_readReg1(&wire,MAG_OUTY_H);
	*Y = (valY[0] | (valY[1]<<8));
	if(*Y >=32768)
	{
		*Y -= 65536;
	}
	
	valZ[0] = mag_readReg1(&wire,MAG_OUTZ_L);
	valZ[1] = mag_readReg1(&wire,MAG_OUTZ_H);
	*Z = (valZ[0] | (valZ[1]<<8));
	if(*Z >=32768)
	{
		*Z -= 65536;
	}
}