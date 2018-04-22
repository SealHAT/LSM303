#include <atmel_start.h>
#include <stdio.h>
#include "LSM303AGR.h"
#include "analyze.h"
#include "SerialPrint.h"

#define STRING_SIZE         (64)
#define BUFFER_SIZE         (15)

int32_t printAxis(AxesSI_t* reading);

int main(void)
{
    AxesRaw_t xcel[BUFFER_SIZE];	    // Accelerometer reading
    AxesRaw_t mag;					    // Magnetometer  reading
    int32_t   err;                      // error code catcher
    bool      ovflw;                    // catch overflows

    atmel_start_init();

    // initialize the I2C for the IMU
    lsm303_init(&I2C_IMU);

    // start the IMU in FIFO mode with the appropriate scale and rate
    lsm303_acc_startFIFO(ACC_SCALE_2G, ACC_HR_50_HZ);

    // start the magnetometer at the given rate
    lsm303_mag_start(MAG_LP_10_HZ);

    for(;;) {

        if(gpio_get_pin_level(IMU_INT1_XL)) {
            err = lsm303_acc_FIFOread(xcel, BUFFER_SIZE, &ovflw);

            if(err < 0) {
                gpio_set_pin_level(LED_BUILTIN, true);
                while(1) {;}
            }

            if(ovflw){
                gpio_set_pin_level(LED_BUILTIN, true);
            }
            else {
                gpio_set_pin_level(LED_BUILTIN, false);
            }
        }

        if(gpio_get_pin_level(IMU_INT_MAG)) {
            err = lsm303_mag_rawRead(&mag);
            if(err != ERR_NONE) {
                gpio_set_pin_level(LED_BUILTIN, true);
                while(1) {;}
            }
        }
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