/*
 * analyze.h
 *
 * Created: 3/7/2018 1:48:04 PM
 *  Author: hpan5
 */ 
#include "LSM303AGR.h"
#define PI (3.14159265358979323846)

#ifndef ANALYZE_H_
#define ANALYZE_H_

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

/**
 *
 */
int32_t get_pitch(AxesSI_t* Axes);


int32_t get_roll(AxesSI_t* Axes);

/**
 *
 */
int32_t get_yaw(AxesSI_t* Axes);

//float yaw_est(AxesSI_t Axes);

//uint32_t get_odba(AxesSI_t Axes);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif /* ANALYZE_H_ */