#include <atmel_start.h>
#include "LSM303.c"
#include "math.h" 
#include <stdio.h>
#include <stdlib.h>

const uint8_t ReadybitMASK = 0b00001000;
const uint8_t IamLX= 0b01000001;
const uint8_t IamMAG = 0b00111101;

static void clearREADYbit()
{
	acc_readReg1(&wire,ACC_OUT_X_L);
	acc_readReg1(&wire,ACC_OUT_X_H);
	acc_readReg1(&wire,ACC_OUT_Y_L);
	acc_readReg1(&wire,ACC_OUT_Y_H);
	acc_readReg1(&wire,ACC_OUT_Z_L);
	acc_readReg1(&wire,ACC_OUT_Z_H);
}
static void readXYZ(int* X, int* Y, int* Z)
{
	uint8_t valX[2];
	uint8_t valY[2];
	uint8_t valZ[2];
	
	valX[0] = acc_readReg1(&wire,ACC_OUT_X_L);
	valX[1] = acc_readReg1(&wire,ACC_OUT_X_H);
	*X = (valX[0] | (valX[1]<<8));
	
	valY[0] = acc_readReg1(&wire,ACC_OUT_Y_L);
	valY[1] = acc_readReg1(&wire,ACC_OUT_Y_H);
	*Y = (valY[0] | (valY[1]<<8));
	
	valZ[0] = acc_readReg1(&wire,ACC_OUT_Z_L);
	valZ[1] = acc_readReg1(&wire,ACC_OUT_Z_H);
	*Z = (valZ[0] | (valZ[1]<<8));
}
int main(void)
{
	//gpio_set_pin_level(LED0, true);
	uint8_t Status = 0b00000000;
	int OUTX_NOST, OUTY_NOST, OUTZ_NOST;
	int OUTX_ST, OUTY_ST, OUTZ_ST;
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();
	imu_init(&wire);
	
	////////////////////Accelerometer self-test
	acc_writeReg1(&wire,ACC_CTRL1, 0x2F); //Initialize sensor, turn on sensor
	acc_writeReg1(&wire,ACC_CTRL4, 0x04); //FS = 2g
	delay_ms(200);
	for(int i=0;i<4;i++){
		clearREADYbit();
		Status = acc_readReg1(&wire,ACC_STATUS);
		//gpio_toggle_pin_level(LED_BUILTIN);
		//gpio_set_pin_level(LED_BUILTIN,false);
		//delay_ms(100);
	}
	if((Status&ReadybitMASK) != 0){
		
	}
	readXYZ(&OUTX_NOST,&OUTY_NOST,&OUTZ_NOST);
	
	acc_writeReg1(&wire,ACC_CTRL5, 0x04); //Enable self-test
	delay_ms(80);
	for(int i=0;i<4;i++){
		clearREADYbit();
		Status = acc_readReg1(&wire,ACC_STATUS);
		//gpio_toggle_pin_level(LED_BUILTIN);
		//gpio_set_pin_level(LED_BUILTIN,false);
		//delay_ms(100);
	}
	if((Status&ReadybitMASK) != 0){
		gpio_set_pin_level(LED_BUILTIN,true);
		readXYZ(&OUTX_ST,&OUTY_ST,&OUTZ_ST);
	}
	int ab = abs(OUTX_ST - OUTX_NOST);
	
	
	if(((ab*0.06) <= 1500) )//&& ((ab*0.061) >= 70)
	{
		
	}
	
	
	
}

