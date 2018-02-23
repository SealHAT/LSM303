#include <atmel_start.h>
 #include "LSM303.c"
 
const uint8_t MASK = 0b11111111;
const uint8_t IamLX= 0b01000001;
const uint8_t IamMAG = 0b00111101;

int main(void)
{
	//gpio_set_pin_level(LED0, true);
	uint8_t NameLX = 0b00000000;
	uint8_t NameMAG = 0b00000000;
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();
	acc_init(&wire);
	NameLX = acc_readReg(ACC_WHO_AM_I);
	mag_init(&wire);
	NameMAG= mag_readReg(MAG_WHO_AM_I);
		
	while((NameMAG == IamMAG)&&(NameLX == IamLX))
	{
		//gpio_set_pin_level(LED0, true);
		delay_ms(100);
		gpio_toggle_pin_level(LED0);
	}
}

