#include <atmel_start.h>
#include <stdio.h>
#include "LSM303AGR.h"
//#include "analyze.h"
#include "SerialPrint.h"

#define STRING_SIZE         (64)
#define BUFFER_SIZE         (40)
#define INT2_THRESHOLD		(0.5/0.016)//g
#define INT2_DURATION		(0)//N/ODR

int32_t printAxis(AxesSI_t* reading);
int32_t printbuffer(int32_t* buffer);
int32_t printval(uint32_t val);

void I2C_UNBLOCK_BUS(const uint8_t SDA, const uint8_t SCL)
{
	uint32_t i, count;

	// Set pin SDA direction to input, pull off
	gpio_set_pin_direction(SDA, GPIO_DIRECTION_IN);
	gpio_set_pin_pull_mode(SDA, GPIO_PULL_OFF);
	gpio_set_pin_function(SDA, GPIO_PIN_FUNCTION_OFF);

	for(i = 0, count = 0; i < 50; i++) {
		count += gpio_get_pin_level(SDA);
	}

	if(count < 10) {
		// Set pin SCL direction to output, pull off
		gpio_set_pin_direction(SCL, GPIO_DIRECTION_OUT);
		gpio_set_pin_pull_mode(SCL, GPIO_PULL_OFF);
		gpio_set_pin_function(SCL, GPIO_PIN_FUNCTION_OFF);

		for(i = 0; i <= 32; i++) {
			gpio_toggle_pin_level(SCL);
		}
	}

}

int main(void)
{
    AxesRaw_t xcel[BUFFER_SIZE];	    // Accelerometer reading
    AxesRaw_t mag;					    // Magnetometer  reading
    int32_t   err;                      // error code catcher

    bool      ovflw;                    // catch overflows
	uint32_t reg_detect;
//	int32_t count;
// 	int32_t pitch[BUFFER_SIZE];
// 	int32_t roll[BUFFER_SIZE];
// 	AxesSI_t xcel_SI[BUFFER_SIZE];
//uint32_t i;
	

	I2C_UNBLOCK_BUS(IMU_SDA, IMU_SCL);
    atmel_start_init();

    // initialize the I2C for the IMU
    lsm303_init(&I2C_IMU);

    // start the IMU in FIFO mode with the appropriate scale and rate
    lsm303_acc_startFIFO(ACC_SCALE_2G, ACC_HR_50_HZ);

    // start the magnetometer at the given rate
    lsm303_mag_start(MAG_LP_10_HZ);
	
	lsm303_acc_setINT2(ACC_INT2_4D_en, INT2_THRESHOLD, INT2_DURATION);
	lsm303_INT2_Enable4D();

    for(;;) {
		
        if(gpio_get_pin_level(IMU_INT1_XL)) {
			
            err = lsm303_acc_FIFOread(xcel, BUFFER_SIZE, &ovflw);
            if(err < 0) {
                while(1) {;}
            }
			//gpio_set_pin_level(LED_BUILTIN, false);
            if(ovflw){;
            }
            else {
				
            }
        }
		gpio_set_pin_level(LED_BUILTIN, false);
		if(gpio_get_pin_level(IMU_INT2_XL)) {
			gpio_set_pin_level(LED_BUILTIN, true);
			err = lsm303_motion_detect(&reg_detect);
			//delay_ms(50);
			//gpio_set_pin_level(LED_BUILTIN, false);
			//gpio_set_pin_level(LED_BUILTIN, false);
			if(err < 0) {
				while(1) {;}
			}else {
				if(usb_dtr()) {
					err = printval(reg_detect);
 					if(err < 0) {
					usb_write("ERROR!\n", 7);
 					} // USB ERROR
			    }  // USB DTR ON
				
			}
		}
		
		/*
        if(gpio_get_pin_level(IMU_INT_MAG)) {
			//gpio_set_pin_level(LED_BUILTIN, true);
            err = lsm303_mag_rawRead(&mag);
			//gpio_set_pin_level(LED_BUILTIN, false);
            if(err != ERR_NONE) {
                gpio_set_pin_level(LED_BUILTIN, true);
                while(1) {;}
            }
        }*/
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

int32_t printbuffer(int32_t* buffer) {
	static char output[STRING_SIZE];
	int n = 0;

	n += ftostr((buffer[n]), 1, &output[n], STRING_SIZE - n);
	output[n++] = ',';
	//output[n++] = '\n';
	return usb_write(output, n);
}

int32_t printval(uint32_t val) {
	static char output[STRING_SIZE];
	memcpy(output+8, &val, 4);
	int n = 0;

	n += ftostr(val, 1, &output[n], STRING_SIZE - n);
	//output[n++] = ',';
	//output[n++] = '\n';
	return usb_write(output, n);
}

// for(;;) {
// 		
//         if(gpio_get_pin_level(IMU_INT1_XL)) {
// 			//gpio_set_pin_level(LED_BUILTIN, true);
// 			
//             count = lsm303_acc_FIFOread(xcel, BUFFER_SIZE, &ovflw);
// 			
// 			/*for(i = 0; i<count; i++){
// 				xcel_SI[i] = lsm303_acc_getSI(&(xcel[i]));
// 				//pitch[i] = get_pitch(&(xcel_SI[i]));
// 				//roll[i] = get_roll(&(xcel_SI[i]));
// 			}*/
// 			
// 			//gpio_set_pin_level(LED_BUILTIN, false);
//             if(err < 0) {
//                 //gpio_set_pin_level(LED_BUILTIN, false);
//                 while(1) {;}
//             }
// 			//gpio_set_pin_level(LED_BUILTIN, false);
//             if(ovflw){;
//                 //gpio_set_pin_level(LED_BUILTIN, false);
//             }
//             else {
// 				/*/if(usb_dtr()) {
// 					gpio_set_pin_level(LED_BUILTIN, usb_dtr());
// 					
// 					for(i = 0; i<count; i++){
// 						//err = printbuffer(&(pitch[i]));
// 						//err = printbuffer(&(roll[i]));
// 						err = printAxis(&(xcel_SI[i]));
// 						
// 					}
// 					
// 					
// 					
// 					if(err < 0) {
// 						delay_ms(1);
// 						usb_write("ERROR!\n", 7);
// 					} // USB ERROR
// 				} */ // USB DTR ON
//             }
//         }
// 		/*
//         if(gpio_get_pin_level(IMU_INT_MAG)) {
// 			//gpio_set_pin_level(LED_BUILTIN, true);
//             err = lsm303_mag_rawRead(&mag);
// 			//gpio_set_pin_level(LED_BUILTIN, false);
//             if(err != ERR_NONE) {
//                 gpio_set_pin_level(LED_BUILTIN, true);
//                 while(1) {;}
//             }
//         }*/
//     } // FOREVER
