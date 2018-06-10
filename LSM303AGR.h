/*
 * LSM303.h
 *
 * Created: 2/23/2018 2:58:51 PM
 *  Author: hpan5
 */


#ifndef LSM303AGR_H_
#define LSM303AGR_H_

#include "hal_i2c_m_sync.h" // device specific IO functions
#include <hal_delay.h>
#include <stdint.h>
#include "LSM303_types.h"

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

/** @brief initialize the lsm303 IMU sensor without starting it
 *
 * @param WIRE [IN] The I2C descriptor to use for the device
 * @return true if successful, system error code otherwise
 */
int32_t lsm303_init(struct i2c_m_sync_desc *const WIRE);

/** @brief Set the rate and range of the accelerometer and start in Bypass Mode
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
int32_t lsm303_acc_startBypass(const ACC_FULL_SCALE_t RANGE, const ACC_OPMODE_t MODE);

/** @brief Set the rate and range of the accelerometer and start the FIFO
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
int32_t lsm303_acc_startFIFO(const ACC_FULL_SCALE_t RANGE, const ACC_OPMODE_t MODE);

/** @brief stop the accelerometer and place it in power down mode
 *
 * This function halts the accelerometer and places it in power down mode, the last
 * used mode will be preserved an can be resumed later.
 * @return true if successful, system error code otherwise
 */
int32_t lsm303_acc_toggle();

/** @brief Set up the motion detection interrupt on the INT2 pin
 *
 * The threshold setting is set in hardware as a 7 bit number where the LSB of the value is
 * dependent on the current scale setting. The LSB represents 16, 32, 62, or 186 milligravities
 * in 2G, 4G, 8G, and 16G modes. This limits the resolution and max value of the thresholds.
 * for example in 8G mode the maximum threshold that can fit in 7 bits is 7874. This function will
 * adjust all input accordingly as long as the parameters given are equal to or less than the current
 * sensitivity.
 *
 * @param sensitivity [IN] some combination of axis to detect from the ACC_MOTION_AXIS_t enumeration.
 * @param threshold [IN] The Threshold of activation. This is used for both the positive
 *                       and negative directions. The value passed in should be in milliGravities
 *                       and the function will return ERR_INVALID_ARG if the threshold is
 *                       greater than or equal to the sensitivity range currently set.
 * @param duration [IN] The number of accelerometer samples where the threshold must be exceeded to
 *                      trigger an interrupt. This results in a duration time of duration/sampleRate.
 *                      The duration must be between 0 and 127. Values over 127 will result in a return
 *                      value of ERR_INVALID_ARG.
 * @return true if successful, system error code otherwise. If an error returns, no registers will have changed.
 */
int32_t lsm303_acc_motionDetectStart(const uint8_t sensitivity, uint16_t threshold, uint8_t duration);

/** @brief reads the motion interrupt register to finds the cause
 *
 * This function check the motion interrupt register on each of the three axis
 * and return the a register state the motions: SWAY, SURGE and HEAVE.
 * @param detect [IN] 8-bit value to store the interrupt flags as described by ACC_MOTION_AXIS_t
 * @return true if successful, system error code otherwise
 */
int32_t lsm303_acc_motionDetectRead(uint8_t* detect);

/** @brief sets up the filter struct for software motion detection
 *
 * @param filter [IN] motion detection struct. Must initialize S to zero before first use.
 * @param threshold [IN] threshold of detection in milligravities
 * @param sensitivity [IN] Axis to detection motion on, combination of enum ACC_MOTION_AXIS_t
 * @param scaler [IN] integer from 1-8, scales the high pass filter. suggested starting value of 1 (50% averaging)
 */
void lsm303_acc_motionDetectSoft_init(MOTION_DETECT_t* filter, const int16_t threshold, const uint8_t sensitivity, const uint8_t scaler);

/** @brief Detects motion in software using a high pass filter and threshold
 *
 * @param filter [IN] motion detection struct. Must initialize S to zero before first use.
 * @param buffer [IN] buffer of raw axis readings
 * @param LEN [IN] length of the data buffer
 * @return a bitwise or of the values in ACC_MOTION_AXIS_t
 */
uint8_t lsm303_motionDetectSoft(MOTION_DETECT_t* filter, AxesRaw_t* buffer, const uint16_t LEN);

/** @brief Get the status of the accelerometer
 *
 * This function checks if there is a new set of data in the accelerometer.
 * It will return a system error code on failure, Most importantly
 * -ERR_OVERFLOW (18) if data has overflowed. If there is no data it will return false,
 * if there is data is will return true.
 *
 * @return positive if a new set of data is available, or a system error code.
 */
int32_t lsm303_acc_dataready();

/** @brief obtains a three-axis accelerometer reading
 *
 * Readings obtained from this function are "raw" and must be transformed into
 * floating point values if actual gravity readings are desired. This can be done
 * with the lsm303_getGravity() function. If there is a communication error the function
 * will return a struct with all values set to 0xFF. This is invalid data.
 *
 * @param rawRead [IN] pointer to a AxesRaw_t struct for loading the data
 * @return 0 if successful, otherwise a system error code
 */
int32_t lsm303_acc_rawRead(AxesRaw_t* rawRead);

/** @brief transform raw accelerometer readings into Gs
 *
 *  The raw data will be shifted to be right aligned once this function is done
 *
 * @param siAccel [IN] pointer to a raw axis struct holding a valid reading from the device
 * @return AxiesSI struct containing the values in Gs, NAN if there is an error.
 */
AxesSI_t lsm303_acc_getSI(AxesRaw_t* rawAccel);

/** @brief convert milliGravities to the raw 16-bit integer value
 *
 * @param milliG [IN] the SI gravity value in milliGravities
 * @return the raw reading equivalent as a right aligned 12-bit value. Based on current Full Scale.
 */
int16_t acc_MgToRaw(int16_t milliG);

/** @brief Check if Watermark had been reached in FIFO buffer
 *
 * @param enabled New enabled state of the FIFO buffer
 * @return true if successful, false otherwise
 */
int32_t lsm303_acc_FIFOWatermark(bool* overflow);

/** @brief Check if there is an sample overrun in FIFO buffer
 *
 * @param enabled New enabled state of the FIFO buffer
 * @return true if successful, false otherwise
 */
int32_t lsm303_acc_FIFOOverrun(void);

/** @brief Check if FIFO buffer is empty
 *
 * @param enabled New enabled state of the FIFO buffer
 * @return true if successful, false otherwise
 */
int32_t lsm303_acc_FIFOEmpty(void);

/** @brief Get the number of unread samples in FIFO buffer
 *
 * @param enabled New enabled state of the FIFO buffer
 * @return true if successful, false otherwise
 */
int32_t lsm303_acc_FIFOCount(void);

/** @brief FIFO multiple read
 *
 * @param buf [OUT] pointer to a buffer of AxesRaw_t structures, must be a multiple of 6 bytes
 * @param LEN [IN] the number of structures in the buffer (number of bytes / 6)
 * @param overrun [OUT] optional flag to indicate FIFO overrun. Pass NULL if not used.
 * @return Number of bytes read if successful, system error or I2C error if failure.
 */
int32_t lsm303_acc_FIFOread(AxesRaw_t* buf, const uint32_t LEN, bool* overrun);

/** @brief set the rate and enable the magnetometer
 *
 * This function enables the magnetometer at the specified rate. The internal
 * temperature sensor is used to compensate the readings.
 *
 * @param MODE [IN] The mode of the sensor which specifies the rate and the power mode
 * @return true if successful, system error code otherwise
 */
int32_t lsm303_mag_start(const MAG_OPMODE_t MODE);

/** @brief stop the magnetometer and place it in power down mode
 *
 * This function halts the accelerometer and places it in power down mode, the last
 * used mode will be preserved an can be resumed later.
 * @return true if successful, system error code otherwise
 */
int32_t lsm303_mag_toggle(void);

/** @brief Get the status of the magnetometer
 *
 * This function checks if there is a new set of data in the magnetometer.
 * it will return a system error code on failure, Most importantly
 * -ERR_OVERFLOW (18) if data has overflowed. If there is no data it will return false,
 * if there is data is will return true.
 *
 * @return positive if a new set of data is available, or a system error code.
 */
int32_t lsm303_mag_dataready(void);

/** @brief obtains a three-axis magnetometer reading
 *
 * Readings obtained from this function are "raw" and must be transformed into
 * floating point values if actual gauss readings are desired. This can be done
 * with the lsm303_mag_getSI() function.
 *
 * @param rawMag [IN] raw axis struct for collecting magnetic data
 * @return 0 if successful otherwise system error code
 */
int32_t lsm303_mag_rawRead(AxesRaw_t* rawMag);

/** @brief transform raw magnetometer readings into Gs
 *
 * @param rawMag [IN] pointer to a raw axis struct holding a valid reading from the device
 * @return AxiesSI struct containing the values in Gauss, NAN if there is an error.
 */
AxesSI_t lsm303_mag_getSI(AxesRaw_t* rawMag);

/** @brief obtains the internal temperature of the magnetometer
 *
 * The temperature has 8 digits per degree C, and is 0 at 25C
 * Therefore C = (reading/8)+25
 *
 * @param temperature [IN] pointer to a 16-bit number to receive the temperature
 * @return system error code, ERR_NONE on success
 */
int32_t lsm303_readTemp(int16_t* temperature);

/** @brief transform raw temperature readings into Gauss
 *
 * @param TEMP [IN] the reading to convert
 * @return The temperature in Celsius, as a floating point value
 */
static inline float lsm303_getCelcius(const int16_t TEMP) { return (TEMP / 8.0) + 25.0;  }

#ifdef __cplusplus
}
#endif // __cplusplus

#endif /* LSM303AGR_H_ */
