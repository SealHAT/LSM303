/*
 * LSM303.h
 *
 * Created: 2/23/2018 2:58:51 PM
 *  Author: hpan5
 */ 
#include "LSM303CTypes.h"

#ifndef LSM303_H_
#define LSM303_H_

#include <atmel_start.h>	/* where the IO functions live */
#include <stdint.h>
#include <stdbool.h>

typedef struct {
	int16_t xAxis;
	int16_t yAxis;
	int16_t zAxis;
} AxesRaw_t;

/** @brief Initializes the LSM303C IMU
 *
 * This function stores the port structure to use for communication with
 * the IMU and initializes it to synchronous mode.
 *
 * @param WIRE [IN] th i2c synchronous structure to use for communication.
 * @return True if successful, false if not. Any error is likely due to I2C
 */
bool imu_init(struct i2c_m_sync_desc *const WIRE);

/** @brief Runs the self test on the accelerometer
 *
 * @return The results of the test
 */
int32_t acc_SelfTest();

/** @brief Configure the operation of the Accelerometer
 *
 * @param RANGE [IN] Full scale range option for the accelerometer
 * @param BLOCK_UPDATE [IN] option to limit updates until the last value is fully read
 * @param AXIS [IN] enable or disable the individual axis, logical OR of axis desired or 0x00 to disable accelerometer
 * @param RATE [IN] The update rate of the accelerometer
 * @return TRUE if the configuration is successful
 */
bool acc_config(const ACC_FS_t RANGE, const ACC_BDU_t BLOCK_UPDATE, const uint8_t AXIS, const ACC_ODR_t RATE);

/** @brief Configure the operation of the Accelerometer
 *
 * Gets the status of the accelerometer - Flags set for new data and data overruns
 *
 * @return the status of the accelerometer as a ACC_STATUS_FLAGS_t type
 */
ACC_STATUS_FLAGS_t acc_getStatus();

/** @brief reads a value from the accelerometer
 *
 * This function reads a value from the accelerometer and returns the
 * three axis values as a struct. The values are signed 16-bit integers.
 *
 * @return A struct of the three axis values, each value is a signed 16-bit integer
 */
AxesRaw_t acc_read();

/** @brief Configure the operation of the magnetometer
 *
 * @param RATE [IN] The update rate of the magnetometer
 * @param SCALE [IN] Full scale range option for the magnetometer
 * @param BLOCK_UPDATE [IN] option to limit updates until the last value is fully read
 * @param PWR_MODE [IN] Operation mode of the x/y axis (power mode)
 * @param PERFORMANCE [IN] Operation mode of the z axis (power mode)
 * @param CONV_MODE [IN] conversion operation mode - continuous, single conversion, or power-down
 * @return TRUE if the configuration is successful
 */
bool mag_config(const MAG_DO_t RATE, const MAG_FS_t SCALE, const MAG_BDU_t BLOCK_UPDATE, const MAG_OMXY_t OMXY, const MAG_OMZ_t OMZ, const MAG_MD_t CONV_MODE);

/** @brief Runs the self test on the magnetometer
 *
 * @return The results of the test
 */
int32_t mag_SelfTest();

/** @brief reads a value from the magnetometer
 *
 * This function reads a value from the accelerometer and returns the
 * three axis values as a struct. The values are signed 16-bit integers.
 *
 * @return A struct of the three axis values, each value is a signed 16-bit integer
 */
AxesRaw_t mag_read();

/** @brief gets the temperature of the IMU sensor
 *
 * This function reads the internal temperature of the IMU for
 * calibrating the readings.
 *
 * @return the temperature reading as a signed 16-bit integer
 */
int16_t imu_readTemp();

/** @brief The IMU temperature sensor value in Celsius
 *
 * @return The IMU internal temperature in Celsius
 */
inline float imu_tempInC(const int16_t RAW_TEMP) { return (RAW_TEMP / 8) + 25 }

#endif /* LSM303_H_ */