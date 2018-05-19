#include <atmel_start.h>
#include <stdio.h>
#include "hal_atomic.h"
#include "LSM303AGR.h"
#include "analyze.h"
#include "SerialPrint.h"

int32_t printAxis(AxesSI_t* reading, const bool motion);

volatile bool    accDataReady;
volatile bool    magDataReady;
volatile bool    motionDetected;
volatile uint8_t detect;                   // to read out motion detection values

static void irq_AccDataReady(void) {
    accDataReady = true;
}

static void irq_MagDataReady(void) {
    magDataReady = true;
}

static void irq_motionDetect(void) {
    motionDetected = true;
    lsm303_acc_motionDetectRead(&detect);
}

int main(void)
{
    static const int BUFFER_SIZE = 32u;
    AxesRaw_t xcel[BUFFER_SIZE];	    // Accelerometer reading
    AxesRaw_t mag;					    // Magnetometer  reading
    int32_t   err;                      // error code catcher
    bool      ovflw;                    // catch overflows

    atmel_start_init();

    // initialize the I2C for the IMU
    lsm303_init(&I2C_IMU);


    ext_irq_register(IMU_INT1_XL, irq_AccDataReady);
    ext_irq_register(IMU_INT2_XL, irq_motionDetect);
    ext_irq_register(IMU_INT_MAG, irq_MagDataReady);

    // start the IMU in FIFO mode with the appropriate scale and rate
    lsm303_acc_startFIFO(ACC_SCALE_2G, ACC_HR_50_HZ);
    lsm303_acc_motionDetectStart(0x16, 100, 0);

    // start the magnetometer at the given rate
//    lsm303_mag_start(MAG_LP_10_HZ);

    for(;;) {

        if(accDataReady) {
            accDataReady = false;
            CRITICAL_SECTION_ENTER();
            err = lsm303_acc_FIFOread(xcel, BUFFER_SIZE, &ovflw);
            CRITICAL_SECTION_LEAVE();

            if(err < 0) {
                gpio_set_pin_level(LED_RED, false);
                while(1) {;}
            }
            
            int i;
            for(i = 0; i < (err/6); i++) {
                AxesSI_t tempAxis = lsm303_acc_getSI(&xcel[i]);
                printAxis(&tempAxis, motionDetected);
            }

            motionDetected = false;
            ovflw = false;

        }

//         if(gpio_get_pin_level(IMU_INT_MAG)) {
//             err = lsm303_mag_rawRead(&mag);
//             if(err != ERR_NONE) {
//                 gpio_set_pin_level(MOD2, true);
//                 while(1) {;}
//             }
//         }
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

#define STRING_SIZE     (64)
int32_t printAxis(AxesSI_t* reading, const bool motion) {
    static char output[STRING_SIZE];
    int n = 0;

    n += ftostr(reading->xAxis, 3, &output[n], STRING_SIZE - n);
    output[n++] = ',';
    n += ftostr(reading->yAxis, 3, &output[n], STRING_SIZE - n);
    output[n++] = ',';
    n += ftostr(reading->zAxis, 3, &output[n], STRING_SIZE - n);
    output[n++] = ',';
    n+= snprintf(&output[n], STRING_SIZE - n, "%d\n", motion);
    return usb_write(output, n);
}