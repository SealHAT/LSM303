#include <atmel_start.h>
 #include "LSM303.c"
 
const uint8_t MASK = 0b00001000;
const uint8_t IamLX= 0b01000001;
const uint8_t IamMAG = 0b00111101;

void clearREADYbit()
{
	acc_readReg1(&wire,ACC_OUT_X_L);
	acc_readReg1(&wire,ACC_OUT_X_H);
	acc_readReg1(&wire,ACC_OUT_Y_L);
	acc_readReg1(&wire,ACC_OUT_Y_H);
	acc_readReg1(&wire,ACC_OUT_Z_L);
	acc_readReg1(&wire,ACC_OUT_Z_H);
}

int main(void)
{
	//gpio_set_pin_level(LED0, true);
	uint8_t OUT = 0b00000000;
	//uint8_t NameMAG = 0b00000000;
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();
	imu_init(&wire);
	////////////////////Accelerometer self-test
	acc_writeReg1(&wire,ACC_CTRL1, 0x2F); //Initialize sensor, turn on sensor
	acc_writeReg1(&wire,ACC_CTRL4, 0x04); //FS = 2g
	do{
		clearREADYbit();
		OUT = acc_readReg1(&wire,ACC_STATUS);
		gpio_toggle_pin_level(LED0);
		delay_ms(2000);
	}while((OUT&MASK) != 0);
	
	acc_writeReg1(&wire,ACC_CTRL5, 0x04); //Enable self-test
}

