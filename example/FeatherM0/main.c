#include <atmel_start.h>
#include "LSM303.c"
#include "math.h" 

const uint8_t ReadybitMASK = 0b00001000;

int32_t acc_SelfTest()
{
	uint8_t Status = 0b00000000;
	int OUTX_NOST, OUTY_NOST, OUTZ_NOST;
	int OUTX_ST, OUTY_ST, OUTZ_ST;
	
	acc_writeReg1(&wire,ACC_CTRL1, 0x2F); //Initialize sensor, turn on sensor
	acc_writeReg1(&wire,ACC_CTRL4, 0x04); //FS = 2g
	acc_writeReg1(&wire,ACC_CTRL5, 0x00); //Disable acc self-test
	delay_ms(200);
	
	do{
		acc_clearREADYbit();
		Status = acc_readReg1(&wire,ACC_STATUS);
	}while((Status&ReadybitMASK) == 0);
	
	if((Status&ReadybitMASK) != 0)
	{
		acc_readXYZ(&OUTX_NOST,&OUTY_NOST,&OUTZ_NOST);
	}
	
	acc_writeReg1(&wire,ACC_CTRL5, 0x04); //Enable acc self-test
	delay_ms(80);
	
	do{
		acc_clearREADYbit();
		Status = acc_readReg1(&wire,ACC_STATUS);
	}while((Status&ReadybitMASK) == 0);
	
	if((Status&ReadybitMASK) != 0){
		acc_readXYZ(&OUTX_ST,&OUTY_ST,&OUTZ_ST);
		gpio_set_pin_level(LED_BUILTIN,true);
	}
	
	int abs_X = abs(OUTX_ST - OUTX_NOST);
	int abs_Y = abs(OUTY_ST - OUTY_NOST);
	int abs_Z = abs(OUTZ_ST - OUTZ_NOST);
	
	if(	   (((abs_X*0.061) <= 1500) && ((abs_X*0.061) >= 70))
	&& (((abs_Y*0.061) <= 1500) && ((abs_Y*0.061) >= 70))
	&& (((abs_Z*0.061) <= 1500) && ((abs_Z*0.061) >= 70))
	)
	{
		return 0;
	}
	else
	{
		return -1;
	}
	acc_writeReg1(&wire,ACC_CTRL5, 0x00); //Disable acc self-test
}
int main(void)
{
	uint8_t Status = 0b00000000;
	int OUTX_NOST, OUTY_NOST, OUTZ_NOST;
	int OUTX_ST, OUTY_ST, OUTZ_ST;
	
	atmel_start_init();
	imu_init(&wire);
	//magnetometer self-test
	mag_writeReg1(&wire,MAG_CTRL_REG1, 0x18);
	mag_writeReg1(&wire,MAG_CTRL_REG2, 0x60);
	mag_writeReg1(&wire,MAG_CTRL_REG3, 0x00);
	mag_writeReg1(&wire,MAG_CTRL_REG5, 0x40);
	delay_ms(20);
	
	do 
	{
		mag_clearREADYbit();
		Status = mag_readReg1(&wire,MAG_STATUS_REG);
		//gpio_toggle_pin_level(LED_BUILTIN);
		//delay_ms(10);
	} while ((Status&ReadybitMASK) == 0);
	if((Status&ReadybitMASK) != 0)
	{
		mag_readXYZ(&OUTX_NOST, &OUTY_NOST, &OUTZ_NOST);
	}
	
	mag_writeReg1(&wire,MAG_CTRL_REG1, 0x19); //Enable mag self-test
	delay_ms(60);
	
	do
	{
		mag_clearREADYbit();
		Status = mag_readReg1(&wire,MAG_STATUS_REG);
		//gpio_toggle_pin_level(LED_BUILTIN);
		//delay_ms(10);
	} while ((Status&ReadybitMASK) == 0);
	if((Status&ReadybitMASK) != 0)
	{
		mag_readXYZ(&OUTX_ST, &OUTY_ST, &OUTZ_ST);
	}
	
	int abs_X = abs(OUTX_ST - OUTX_NOST);
	int abs_Y = abs(OUTY_ST - OUTY_NOST);
	int abs_Z = abs(OUTZ_ST - OUTZ_NOST);
	
	while(	(((abs_X*0.58/1000) >= 1) && ((abs_X*0.58/1000) <= 3))
		 &&	(((abs_Z*0.58/1000) >= 0.1) && ((abs_Z*0.58/1000) <= 1))
		 &&	(((abs_Y*0.58/1000) >= 1) && ((abs_Y*0.58/1000) <= 3))
		)
	{
		gpio_toggle_pin_level(LED_BUILTIN);
		delay_ms(100);
	}
}

