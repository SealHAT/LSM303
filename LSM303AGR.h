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

typedef enum {
	ACC_FIFOSRC_WTM         = 0x80, // set high when FIFO contents exceed watermark
	ACC_FIFOSRC_OVRN        = 0x40, // set high when the FIFO is full (32 samples) and the NEXT reading will overwrite the oldest
	ACC_FIFOSRC_EMPTY       = 0x20, // set high when all samples are read and FIFo is empty
	ACC_FIFOSRC_FSS         = 0x1F, // the current number of unread samples
} ACC_FIFO_STATUS_t;

typedef enum {
    AXIS_DISABLE_ALL    = 0x00,
    AXIS_X_ENABLE       = 0x01,
    AXIS_Y_ENABLE       = 0x02,
    AXIS_YX_ENABLE      = 0x03,
    AXIS_Z_ENABLE       = 0x04,
    AXIS_ZX_ENABLE      = 0x05,
    AXIS_ZY_ENABLE      = 0x06,
    AXIS_ENABLE_ALL	    = 0x07
} IMU_AXIS_t;

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
    MAG_IDLE            = 0x03,

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
 * @return true if successful, false if hardware allocation fails
 */
bool lsm303_init(struct i2c_m_sync_desc *const WIRE);

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
 * @return true if successful, false if registers are not set correctly
 */
bool lsm303_startAcc(const IMU_AXIS_t AXIS, const ACC_FULL_SCALE_t RANGE, const ACC_OPMODE_t MODE);

/** @brief stop the accelerometer and place it in power down mode
 *
 * This function halts the accelerometer and places it in power down mode, the last
 * used mode will be preserved an can be resumed later.
 * @return true if successful, false if I2C transmission fails
 */
bool lsm303_stopAcc();

/** @brief Set the rate and range of the accelerometer
 *
 * This function resumes the last used settings. If the accelerometers last mode set
 * was the power down mode then the default settings will be used (all axis at 2Gs in high res mode at 50Hz).
 * @return true if successful, false if registers are not set correctly
 */
bool lsm303_resumeAcc();

bool lsm303_setFIFOenabled(bool enabled);

/** @brief set the rate and enable the magnetometer
 *
 * This function enables the magnetometer at the specified rate. The internal
 * temperature sensor is used to compensate the readings.
 *
 * @param MODE [IN] The mode of the sensor which specifies the rate and the power mode
 * @return true if successful, false otherwise
 */
bool lsm303_startMag(const MAG_OPMODE_t MODE);

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

#endif /* LSM303AGR_H_ */