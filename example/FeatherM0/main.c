#include <atmel_start.h>
#include "LSM303.c"
#include "math.h" 

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

