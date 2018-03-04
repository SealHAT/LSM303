#include <atmel_start.h>
#include "LSM303.h"
#include "math.h" 
#include "usb_start.h"

#define STRING_SIZE (64)

int decPart(const float VAL, const int SIGFIG) {
	int retval = (VAL * SIGFIG) - ((int)VAL * SIGFIG);
	return (retval > 0 ? retval : retval * -1);
}

int main(void)
{
	char  output[STRING_SIZE];		/* A string used for output on USB */
	AxesRaw_t xcel;					/* Accelerometer reading */
	AxesRaw_t mag;					/* Magnetometer reading */
	int16_t   temp;					/* Magnetometer temperature */
	IMU_STATUS_t newAcc, newMag;	/* Indicate a new sample */
	float x,y,z,c;					/* Float axis for string output */
	
	atmel_start_init();
	lsm303_init(&wire);	
	lsm303_startAcc(ACC_FS_2G, ACC_ODR_50_Hz);
	lsm303_startMag(MAG_MODE_CONTINUOUS, MAG_DO_10_Hz, MAG_TEMP_ENABLE);
	
	for(;;) {
		/* Turn on LED if the DTR signal is set (serial terminal open on host) */
		gpio_set_pin_level(LED_BUILTIN, usb_dtr());

		/* Read and print the Accelerometer if it is ready */
		newAcc = lsm303_statusAcc();
		if(newAcc != NULL_STATUS) {
			xcel  = lsm303_readAcc();			
			
			x = lsm303_getGravity(xcel.xAxis);
			y = lsm303_getGravity(xcel.yAxis);
			z = lsm303_getGravity(xcel.zAxis);
			
			/* Format as a string and output to USB Serial */
			sprintf(output, "ACCEL: x=%d.%dg   y=%d.%dg   z=%d.%dg\n", (int)x, decPart(x, 1000), (int)y, decPart(y, 1000), (int)z, decPart(z, 1000));
			
			if(usb_dtr()) {
				usb_send_buffer((uint8_t*)output, strlen(output));
			}
		}
		
		/* Read and print the Magnetometer if it is ready */
		newMag = lsm303_statusMag();
		if(newMag != NULL_STATUS) {
			mag  = lsm303_readMag();
			temp = lsm303_readTemp();
			
			x = lsm303_getGauss(mag.xAxis);
			y = lsm303_getGauss(mag.yAxis);
			z = lsm303_getGauss(mag.zAxis);
			c = lsm303_getCelcius(temp);
			
			/* Format as a string and output to USB Serial */
			sprintf(output, "MAG: x=%d.%dgauss   y=%d.%dgauss   z=%d.%dgauss    TEMP: %d.%d\n", 
			        (int)x, decPart(x, 1000), (int)y, decPart(y, 1000), (int)z, decPart(z, 1000), (int)c, decPart(c, 100));
			
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