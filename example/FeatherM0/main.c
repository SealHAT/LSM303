#include <atmel_start.h>
#include <stdio.h>
#include "LSM303.h"
#include "math.h"
#include "usb_start.h"

#define STRING_SIZE (64)

// float printing function ported from arduino
size_t printFloat(double number, uint8_t digits);

int main(void)
{
	char  output[STRING_SIZE];		/* A string used for output on USB */
	AxesRaw_t xcel;					/* Accelerometer reading */
	AxesRaw_t mag;					/* Magnetometer reading */
	int16_t   temp;					/* Magnetometer temperature */
	IMU_STATUS_t newAcc, newMag;	/* Indicate a new sample */
	float x, y, z;
	
	atmel_start_init();
	lsm303_init(&wire);
	lsm303_startAcc(ACC_FS_2G, ACC_ODR_50_Hz);
//	lsm303_startMag(MAG_MODE_CONTINUOUS, MAG_DO_40_Hz, MAG_TEMP_ENABLE);
	
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
			
			/* Print the data if USB is available */
			if(usb_dtr()) {
				snprintf(output, STRING_SIZE, "%d.%d,%d.%d,%d.%d\n", (int)x, decPart(x), (int)y, decPart(y), (int)z, decPart(z));
				usb_send_buffer((uint8_t*)output, strlen(output));
			}
		}
		
		/* Read and print the Magnetometer if it is ready */
//		newMag = lsm303_statusMag();
//		if(newMag != NULL_STATUS) {
//			mag  = lsm303_readMag();
//			temp = lsm303_readTemp();
//			
//			/* Print the data if USB is available */
//			if(usb_dtr()) {
//				snprintf(output, STRING_SIZE, "MAG:%d,%d,%d,%d\n", mag.xAxis, mag.yAxis, mag.zAxis, temp);
//				usb_send_buffer((uint8_t*)output, strlen(output));
//			}
//		}
	}
}

size_t printFloat(double number, uint8_t digits)
{
	static const int OUTPUT_SIZE = 12;
	char output[OUTPUT_SIZE];
	size_t n = 0;

	if (isnan(number)) {
		snprintf(output, OUTPUT_SIZE, "nan");
	}	
	else if (isinf(number)) {
		snprintf(output, OUTPUT_SIZE, "inf");
	}
	else if (number > 4294967040.0) {
		snprintf(output, OUTPUT_SIZE, "ovf");  // constant determined empirically
	}
	else if (number <-4294967040.0) {
		snprintf(output, OUTPUT_SIZE, "ovf");  // constant determined empirically
	}
	else {
		// Handle negative numbers
		if (number < 0.0) {
			output[n++] = '-';
			number = -number;
		}

		// Round correctly so that print(1.999, 2) prints as "2.00"
		double rounding = 0.5;
		for (uint8_t i = 0; i < digits; i++) {
			rounding /= 10.0;
		}

		number += rounding;

		// Extract the integer part of the number and print it
		unsigned long int_part = (unsigned long)number;
		double remainder = number - (double)int_part;
		n += print(int_part);

		// Print the decimal point, but only if there are digits beyond
		if (digits > 0) {
			n += print(".");
		}

		// Extract digits from the remainder one at a time
		while (digits-- > 0)
		{
			remainder *= 10.0;
			unsigned int toPrint = (unsigned int)remainder;
			n += print(toPrint);
			remainder -= toPrint;
		}
	}
	return n;
}