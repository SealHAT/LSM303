#include <atmel_start.h>
#include <stdio.h>
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
	
	atmel_start_init();
	lsm303_init(&wire);
	lsm303_startAcc(ACC_FS_2G, ACC_ODR_50_Hz);
	lsm303_startMag(MAG_MODE_CONTINUOUS, MAG_DO_40_Hz, MAG_TEMP_ENABLE);
	
	for(;;) {
		/* Turn on LED if the DTR signal is set (serial terminal open on host) */
		gpio_set_pin_level(LED_BUILTIN, usb_dtr());

		/* Read and print the Accelerometer if it is ready */
		newAcc = lsm303_statusAcc();
		if(newAcc != NULL_STATUS) {
			xcel  = lsm303_readAcc();
			
			/* Print the data if USB is available */
			if(usb_dtr()) {
				snprintf(output, STRING_SIZE, "XCEL:%d,%d,%d,%d\n", ACC_FS_2G, xcel.xAxis, xcel.yAxis, xcel.zAxis);
				usb_send_buffer((uint8_t*)output, strlen(output));
			}
		}
		
		/* Read and print the Magnetometer if it is ready */
		newMag = lsm303_statusMag();
		if(newMag != NULL_STATUS) {
			mag  = lsm303_readMag();
			temp = lsm303_readTemp();
			
			/* Print the data if USB is available */
			if(usb_dtr()) {
				snprintf(output, STRING_SIZE, "MAG:%d,%d,%d,%d\n", mag.xAxis, mag.yAxis, mag.zAxis, temp);
				usb_send_buffer((uint8_t*)output, strlen(output));
			}
		}
	}
}