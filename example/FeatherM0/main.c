#include <atmel_start.h>
#include <stdio.h>
#include "LSM303.h"
#include "analyze.h"
#include "SerialPrint.h"

#define STRING_SIZE (64)

int32_t printAxis(AxesSI_t* reading);

int main(void)
{
	AxesSI_t xcel;					/* Accelerometer reading */
	AxesRaw_t mag;					/* Magnetometer reading */
	int16_t   temp;					/* Magnetometer temperature */
	IMU_STATUS_t newAcc, newMag;	/* Indicate a new sample */
    int32_t err;
	
	atmel_start_init();
	lsm303_init(&wire);
	lsm303_startAcc(ACC_FS_2G, ACC_ODR_200_Hz);
	
	for(;;) {
		/* Turn on LED if the DTR signal is set (serial terminal open on host) */
		gpio_set_pin_level(LED_BUILTIN, usb_dtr());

		/* Read and print the Accelerometer if it is ready */
		newAcc = lsm303_statusAcc();
		if(newAcc != NULL_STATUS) {
            xcel  = lsm303_getGravity();
			/* Print the data if USB is available */
			if(usb_dtr()) {
				err = printAxis(&xcel);
                if(err < 0) {
                    delay_ms(1);
                    usb_write("ERROR!\n", 7);
                } // USB ERROR
			} // USB DTR ON
		} // NEW ACCEL
	} // FOREVER
}

static int ftostr(double number, uint8_t digits, char* buff, const int LEN) {
    uint8_t i;
    size_t n = 0;
    
    // Not a number, special floating point value
    if (isnan(number)) {
        n = snprintf(buff, LEN, "nan");
    }
    // infinity, special floating point value
    else if (isinf(number)) {
        n = snprintf(buff, LEN, "inf");
    }
    // constant determined empirically
    else if (number > 4294967040.0) {
        n = snprintf(buff, LEN, "ovf");
    }
    // constant determined empirically
    else if (number <-4294967040.0) {
        n = snprintf(buff, LEN, "ovf");
    }
    // A valid floating point value
    else {
        // Handle negative numbers
        if (number < 0.0) {
            buff[n++] = '-';
            number = -number;
        }

        // Round correctly so that print(1.999, 2) prints as "2.00"
        double rounding = 0.5;
        for (i = 0; i < digits; i++) {
            rounding /= 10.0;
        }
        number += rounding;

        // Extract the integer part of the number and print it
        unsigned long int_part = (unsigned long)number;
        double remainder = number - (double)int_part;
        n += snprintf(&buff[n], LEN-n, "%ld", int_part);

        // Print the decimal point, but only if there are digits beyond
        if (digits > 0) {
            buff[n] = '.';
            buff[++n] = '\0';
            
            // Extract digits from the remainder one at a time
            while (digits-- > 0) {
                // calculate the current digit
                remainder *= 10.0;
                unsigned int toPrint = (unsigned int)remainder;
                
                // overwrite the last null terminator with the current digit
                n += snprintf(&buff[n], LEN-n, "%d", toPrint);
                
                // shift to the next digit
                remainder -= toPrint;
            }
        }
    } // VALID FLOAT BLOCK

    return n;
}

int32_t printAxis(AxesSI_t* reading) {
    static char output[STRING_SIZE];
    int n = 0;

    n += ftostr(reading->xAxis, 3, &output[n], STRING_SIZE - n);
    output[n++] = ',';
    n += ftostr(reading->yAxis, 3, &output[n], STRING_SIZE - n);
    output[n++] = ',';
    n += ftostr(reading->zAxis, 3, &output[n], STRING_SIZE - n);   
    output[n++] = '\n';
    return usb_write(output, n);
}