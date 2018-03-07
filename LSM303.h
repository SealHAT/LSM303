/*
 * LSM303.h
 *
 * Created: 2/23/2018 2:58:51 PM
 *  Author: hpan5
 */ 


#ifndef LSM303_H_
#define LSM303_H_

#include <atmel_start.h>	/* where the IO functions live */
#include <stdint.h>
#include <stdbool.h>
#include "LSM303CTypes.h"

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

/* Status return values from all IMU sensors */
typedef enum {
	NULL_STATUS				= 0x00,
	X_NEW_DATA_AVAILABLE    = 0x01,
	Y_NEW_DATA_AVAILABLE    = 0x02,
	Z_NEW_DATA_AVAILABLE    = 0x04,
	ZYX_NEW_DATA_AVAILABLE  = 0x08,
	X_OVERRUN               = 0x10,
	Y_OVERRUN               = 0x20,
	Z_OVERRUN               = 0x40,
	ZYX_OVERRUN             = 0x80
} IMU_STATUS_t;

/** @brief initialize the lsm303 IMU sensor without starting it
 *
 * @param WIRE [IN] The I2C descriptor to use for the device
 * @return true if successful, false if hardware allocation fails
 */
bool lsm303_init(struct i2c_m_sync_desc *const WIRE);

/** @brief Set the rate and range of the accelerometer
 *
 * This function sets the rate and range of the accelerometer. The Rate
 * can be any of the pre-defined rates in ACC_ODR_t, including the ACC_ODR_POWER_DOWN
 * state. This is how the accelerometer is turned on and off manually. Once sampling the
 * readings will be ready to read, either by setting up the interrupts with the appropriate
 * functions or by polling the status register.
 *
 * @param RANGE [IN] the full scale range of the accelerometer
 * @param RATE [IN] the sample rate of the accelerometer
 * @return true if successful, false if registers are not set correctly
 */
bool lsm303_startAcc(const ACC_FS_t RANGE, const ACC_ODR_t RATE);

/** @brief set the rate and enable the magnetometer
 *
 * This function enables the magnetometer at the specified rate. The internal
 * temperature sensor can also be enabled to help compensate readings.
 * The MODE setting can either one of three modes: 
 *     - MAG_MODE_OFF: disables the magnetometer
 *     - MAG_MODE_SINGLE: Performs a single reading and then automatically returns the mode to the OFF state
 *                        and indicates the data ready state on the DRDY pin.
 *     - MAG_MODE_CONTINUOUS: Continuously reads at the specified ODR rate. The magnetometer has no FIFO.
 *
 * @param MODE [IN] The mode of the sensor, as described above.
 * @param RATE [IN] The rate to sample the sensor, a predefined rate from MAG_DO_t
 * @param TEMPERATURE [IN] Enable or Disable the internal temperature sensor
 * @return
 */
bool lsm303_startMag(const MAG_MODE_t MODE, const MAG_DO_t RATE, const MAG_TEMP_EN_t TEMPERATURE);

/** @brief Get the status of the accelerometer
 *
 * The Sensor status enum defined several flags. It can indicate a data overflow for
 * both individual axis as well as all three axis combined. If neither condition is true
 * the NULL_STATUS value is returned.
 *
 * @return the IMU_STATUS_t flags enumeration
 */
IMU_STATUS_t lsm303_statusAcc();

/** @brief Get the status of the magnetometer
 *
 * The Sensor status enum defined several flags. It can indicate a data overflow for
 * both individual axis as well as all three axis combined. If neither condition is true
 * the NULL_STATUS value is returned.
 *
 * @return the IMU_STATUS_t flags enumeration
 */
IMU_STATUS_t lsm303_statusMag();

/** @brief obtains a three-axis accelerometer reading
 *
 * Readings obtained from this function are "raw" and must be transformed into
 * floating point values if actual gravity readings are desired. This can be done
 * with the lsm303_getGravity() function.
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

#endif /* LSM303_H_ */