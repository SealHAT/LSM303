#include <atmel_start.h>
#include <stdio.h>
#include "LSM303AGR.h"
#include "analyze.h"
#include "SerialPrint.h"

#define STRING_SIZE (64)

int32_t printAxis(AxesSI_t* reading);

int main(void)
{
	AxesSI_t xcel;					/* Accelerometer reading */
	AxesSI_t mag;					/* Magnetometer reading */
	//int16_t   temp;				    /* Magnetometer temperature */
	IMU_STATUS_t newAcc, newMag;	/* Indicate a new sample */
	//int32_t unread_num;
    int32_t err;
	
	atmel_start_init();
	lsm303_init(&wire);
	lsm303_startAcc(AXIS_ENABLE_ALL, ACC_SCALE_2G, ACC_HR_50_HZ);
	lsm303_startMag(MAG_LP_50_HZ);
	lsm303_startFIFO();
	for(;;) {
		while(lsm303_statusFIFOWTM() == 0){}
		while(lsm303_statusFIFOEMPTY() == 0){
			gpio_set_pin_level(LED_BUILTIN, true);
			xcel  = lsm303_getGravity();
			gpio_set_pin_level(LED_BUILTIN, false);
			/* Print the data if USB is available */
			if(usb_dtr()) {
				err = printAxis(&xcel);
				if(err < 0) {
					delay_ms(1);
					usb_write("ERROR!\n", 7);
				} // USB ERROR
			} // USB DTR ON
		}
		
	
// 	/* Read and print the Magnetometer if it is ready */
// 	newMag = lsm303_statusMag();
// 	if(newMag != NULL_STATUS) {
// 		gpio_set_pin_level(LED_BUILTIN, true);
// 		mag  = lsm303_getGauss();
// 		gpio_set_pin_level(LED_BUILTIN, false);
// 		/* Print the data if USB is available */
// 		if(usb_dtr()) {
// 			err = printAxis(&mag);
// 			if(err < 0) {
// 				delay_ms(1);
// 				usb_write("ERROR!\n", 7);
// 			} // USB ERROR
// 		} // USB DTR ON
// 	} // NEW MAG
	
	} // FOREVER
}

/////////////////////////////
// 		/* Read and print the Accelerometer if it is ready */
// 		newAcc = lsm303_statusAcc();
// 		if(newAcc & ZYX_NEW_DATA_AVAILABLE) {
// 			gpio_set_pin_level(LED_BUILTIN, true);
// 			xcel  = lsm303_getGravity();
// 			gpio_set_pin_level(LED_BUILTIN, false);
// 			/* Print the data if USB is available */
// 			if(usb_dtr()) {
// 				err = printAxis(&xcel);
// 				if(err < 0) {
// 					delay_ms(1);
// 					usb_write("ERROR!\n", 7);
// 				} // USB ERROR
// 			} // USB DTR ON
// 		} // NEW ACCEL
/////////////////////////////////////////////////////
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