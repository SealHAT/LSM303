/*
 * LSM303.h
 *
 * Created: 2/23/2018 2:58:51 PM
 *  Author: hpan5
 */ 


#ifndef LSM303AGR_H_
#define LSM303AGR_H_

#include <atmel_start.h>	/* where the IO functions live */
#include <stdint.h>
#include <stdbool.h>
#include "math.h"

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

/* Structure to store and return 3-axis measurements */
typedef struct {
	int16_t xAxis;
	int16_t yAxis;
	int16_t zAxis;
} AxesRaw_t;

typedef struct {
	float xAxis;
	float yAxis;
	float zAxis;
} AxesSI_t;

/* Valid modes for the LSM303AGR */
typedef enum {
    ACC_POWER_DOWN      = 0x00,

    ACC_HR_1_HZ         = 0x14,
    ACC_HR_10_HZ        = 0x24,
    ACC_HR_25_HZ        = 0x34,
    ACC_HR_50_HZ        = 0x44,
    ACC_HR_100_HZ       = 0x54,
    ACC_HR_200_HZ       = 0x64,
    ACC_HR_400_HZ       = 0x74,
    ACC_HR_1344_HZ      = 0x94,

    ACC_NORM_1_HZ       = 0x10,
    ACC_NORM_10_HZ      = 0x20,
    ACC_NORM_25_HZ      = 0x30,
    ACC_NORM_50_HZ      = 0x40,
    ACC_NORM_100_HZ     = 0x50,
    ACC_NORM_200_HZ     = 0x60,
    ACC_NORM_400_HZ     = 0x70,
    ACC_NORM_1344_HZ    = 0x90,

    ACC_LP_1_HZ         = 0x18,
    ACC_LP_10_HZ        = 0x28,
    ACC_LP_25_HZ        = 0x38,
    ACC_LP_50_HZ        = 0x48,
    ACC_LP_100_HZ       = 0x58,
    ACC_LP_200_HZ       = 0x68,
    ACC_LP_400_HZ       = 0x78,
    ACC_LP_1620_HZ      = 0x88,
    ACC_LP_5376_HZ      = 0x98,
} ACC_OPMODE_t;

typedef enum {
    ACC_SCALE_2G        = 0x00,
    ACC_SCALE_4G        = 0x10,
    ACC_SCALE_8G        = 0x20,
    ACC_SCALE_16G       = 0x30
} ACC_FULL_SCALE_t;


typedef enum {
    MAG_IDLE            = 0x13,

    MAG_NORM_10_HZ      = 0x00,
    MAG_NORM_20_HZ      = 0x04,
    MAG_NORM_50_HZ      = 0x08,
    MAG_NORM_100_HZ     = 0x0C,

    MAG_LP_10_HZ        = 0x10,
    MAG_LP_20_HZ        = 0x14,
    MAG_LP_50_HZ        = 0x18,
    MAG_LP_100_HZ       = 0x1C
} MAG_OPMODE_t;



/** @brief initialize the lsm303 IMU sensor without starting it
 *
 * @param WIRE [IN] The I2C descriptor to use for the device
 * @return true if successful, system error code otherwise
 */
int32_t lsm303_init(struct i2c_m_sync_desc *const WIRE);

/** @brief Set the rate and range of the accelerometer
 *
 * This function sets the rate and range of the accelerometer. The Rate
 * can be any of the pre-defined rates in ACC_MODE_t, including the ACC_POWER_DOWN
 * state. This is how the accelerometer is turned on and off manually. Once sampling the
 * readings will be ready to read, either by setting up the interrupts with the appropriate
 * functions or by polling the status register.
 *
 * @param RANGE [IN] the full scale range of the accelerometer
 * @param MODE [IN] mode of the accelerometer to set the rate and the resolution
 * @return true if successful, system error code otherwise
 */
int32_t lsm303_startAcc(const ACC_FULL_SCALE_t RANGE, const ACC_OPMODE_t MODE);

/** @brief stop the accelerometer and place it in power down mode
 *
 * This function halts the accelerometer and places it in power down mode, the last
 * used mode will be preserved an can be resumed later.
 * @return true if successful, system error code otherwise
 */
int32_t lsm303_stopAcc();

/** @brief resume the accelerometer operations
 *
 * This function resumes the last used settings. If the accelerometers last mode set
 * was the power down mode then the default settings will be used (all axis at 2Gs in high res mode at 50Hz).
 * @return true if successful, system error code otherwise
 */
int32_t lsm303_resumeAcc();

/** @brief set the rate and enable the magnetometer
 *
 * This function enables the magnetometer at the specified rate. The internal
 * temperature sensor is used to compensate the readings.
 *
 * @param MODE [IN] The mode of the sensor which specifies the rate and the power mode
 * @return true if successful, system error code otherwise
 */
int32_t lsm303_startMag(const MAG_OPMODE_t MODE);

/** @brief stop the magnetometer and place it in power down mode
 *
 * This function halts the accelerometer and places it in power down mode, the last
 * used mode will be preserved an can be resumed later.
 * @return true if successful, system error code otherwise
 */
int32_t lsm303_stopMag();

/** @brief resume the magnetometer with the last used settings
 *
 * This function resumes the last used settings. If the magnetometer last mode set
 * was the power down mode then the default settings will be used (low power at 20 Hz).
 * @return true if successful, system error code otherwise
 */
int32_t lsm303_resumeMag();

/** @brief Get the status of the accelerometer
 *
 * This function checks if there is a new set of data in the accelerometer. 
 * It will return a system error code on failure, Most importantly 
 * ERR_OVERFLOW if data has overflowed. If there is no data it will return false, 
 * if there is data is will return true.
 *
 * @return positive if a new set of data is available, or a system error code.
 */
int32_t ls303_acc_dataready();

/** @brief Get the status of the magnetometer
 *
 * This function checks if there is a new set of data in the magnetometer.
 * it will return a system error code on failure, Most importantly 
 * ERR_OVERFLOW if data has overflowed. If there is no data it will return false, 
 * if there is data is will return true.
 *
 * @return positive if a new set of data is available, or a system error code.
 */
int32_t ls303_mag_dataready();

/** @brief obtains a three-axis accelerometer reading
 *
 * Readings obtained from this function are "raw" and must be transformed into
 * floating point values if actual gravity readings are desired. This can be done
 * with the lsm303_getGravity() function. If there is a communication error the function
 * will return a struct with all values set to 0xFF. This is invalid data.
 *
 * @return the three axis of the accelerometer as the AxesRaw_t struct
 */
AxesRaw_t lsm303_readAcc();

/** @brief obtains a three-axis magnetometer reading
 *
 * Readings obtained from this function are "raw" and must be transformed into
 * floating point values if actual gauss readings are desired. This can be done
 * with the lsm303_getGauss() function.
 *
 * @return the three axis of the magnetometer as the AxesRaw_t struct.
 */
AxesRaw_t lsm303_readMag();

/** @brief obtains the internal temperature of the magnetometer
 *
 * The temperature has 8 digits per degree C, and is 0 at 25C
 * Therefore C = (reading/8)+25
 *
 * @return the temperature as a signed 16-bit integer.
 */
int16_t lsm303_readTemp();

/** @brief transform raw accelerometer readings into Gs
 *
 * @param AXIS [IN] The reading to convert
 * @return The axis x y and z in SI unit (gravity)
 */
AxesSI_t lsm303_getGravity();

/** @brief transform raw magnetometer readings into Gauss
 *
 * @param AXIS [IN] The reading to convert
 * @return The axis x y and z in SI unit (Gauss)
 */
AxesSI_t lsm303_getGauss();

/** @brief transform raw temperature readings into Gauss
 *
 * @param TEMP [IN] the reading to convert
 * @return The temperature in Celcius, as a flaoting point value
 */
inline float lsm303_getCelcius(const int16_t TEMP) { return (TEMP / 8.0) + 25.0;  }

#ifdef __cplusplus
}
#endif // __cplusplus

#endif /* LSM303AGR_H_ */