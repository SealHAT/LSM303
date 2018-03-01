#include <atmel_start.h>
#include "LSM303.h"
#include "math.h" 
#include "usb_start.h"

#define STRING_SIZE (64)

int decPart(const float VAL) {
	int retval = (VAL * 100) - ((int)VAL * 100);
	return (retval > 0 ? retval : retval * -1);
}

int main(void)
{
	
	char  output[STRING_SIZE];		/* A string used for output on USB */
	AxesRaw_t xcel;					/* Accelerometer reading */
	float x,y,z;					/* Float axis for string output */
	
	atmel_start_init();
	imu_init(&wire);	
	acc_config(ACC_FS_2g, ACC_BDU_ENABLE, ACC_ENABLE_ALL, ACC_ODR_50_Hz, ACC_INCREMENT);
	
	for(;;) {
		/* Turn on LED if the DTR signal is set (serial terminal open on host) */
		gpio_set_pin_level(LED_BUILTIN, usb_dtr());

		/* Read the light sensor as both a exponent/mantissa and as an integer LUX value */
		if(acc_getStatus() != ACC_NULL_STATUS) {
			xcel  = acc_read();
			
			x = (float)(xcel.xAxis*0.061/1000);
			y = (float)(xcel.yAxis*0.061/1000);
			z = (float)(xcel.zAxis*0.061/1000);
		
			/* Format as a string and output to USB Serial */
			sprintf(output, "ACCEL: x=%d.%dg   y=%d.%dg   z=%d.%dg\n", (int)x, decPart(x), (int)y, decPart(y), (int)z, decPart(z));
			
			if(usb_dtr()) {
				usb_send_buffer((uint8_t*)output, strlen(output));
			}
		}
	}
}

/*
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

uint8_t Status = 0b00000000;
int OUTX_NOST, OUTY_NOST, OUTZ_NOST;
int OUTX_ST, OUTY_ST, OUTZ_ST;
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
*/