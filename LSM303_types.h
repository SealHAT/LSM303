/*
 * LSM303_Types.h
 *
 * Created: 09-Jun-18 23:23:53
 *  Author: Ethan
 */

#ifndef LSM303_TYPES_H_
#define LSM303_TYPES_H_

/* Structure to store and return 3-axis measurements */
typedef struct __attribute__((__packed__)){
    int16_t xAxis;
    int16_t yAxis;
    int16_t zAxis;
} AxesRaw_t;

typedef struct __attribute__((__packed__)){
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
    ACC_POWER_MODE_MASK = 0x0F,
    ACC_HIGH_RESOLUTION = 0x04,
    ACC_NORMAL_POWER    = 0x00,
    ACC_LOW_POWER       = 0x08
} ACC_POWER_MODE_t;

typedef enum {
    ACC_RATE_MASK       = 0xF0,
    ACC_1_HZ            = 0x10,
    ACC_10_HZ           = 0x20,
    ACC_25_HZ           = 0x30,
    ACC_50_HZ           = 0x40,
    ACC_100_HZ          = 0x50,
    ACC_200_HZ          = 0x60,
    ACC_400_HZ          = 0x70,
} ACC_SAMPLE_RATE_t;

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

typedef enum {
    MAG_RATE_MASK       = 0x0F,
    MAG_10_HZ           = 0x00,
    MAG_20_HZ           = 0x04,
    MAG_50_HZ           = 0x08,
    MAG_100_HZ          = 0x0C,
} MAG_SAMPLE_RATE_t;

typedef enum {
    MOTION_INT_X_HIGH       = 0x02,
    MOTION_INT_Y_HIGH       = 0x08,
    MOTION_INT_Z_HIGH       = 0x20,
    MOTION_INT_XY_HIGH      = 0x0A,
    MOTION_INT_XZ_HIGH      = 0x22,
    MOTION_INT_YZ_HIGH      = 0x28,
    MOTION_INT_XYZ_HIGH     = 0x2A,

    MOTION_INT_X_LOW        = 0x01,
    MOTION_INT_Y_LOW        = 0x04,
    MOTION_INT_Z_LOW        = 0x10,
    MOTION_INT_XY_LOW       = 0x05,
    MOTION_INT_XZ_LOW       = 0x11,
    MOTION_INT_YZ_LOW       = 0x14,
    MOTION_INT_XYZ_LOW      = 0x15,

    MOTION_INT_MASK         = 0x3F
} ACC_MOTION_AXIS_t;

typedef struct __attribute__((__packed__)){
    AxesRaw_t S;            // memory value, for internal use by algorithm
    int16_t   threshold;    // threshold of motion to detect in milligravity
    int16_t   duration;     // Duration of movement to detect. not used.
    uint8_t   sensitivity;  // axis to check for motion as defined by ACC_MOTION_AXIS_t
    uint8_t   hp;           // must be a value greater than 0
} MOTION_DETECT_t;

#endif /* LSM303_TYPES_H_ */
