/*
 * analyze.h
 *
 * Created: 3/7/2018 1:48:04 PM
 *  Author: hpan5
 */ 
#include "LSM303.h"
#define PI (3.14159265358979323846)

#ifndef ANALYZE_H_
#define ANALYZE_H_

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

/**
 *
 */
float getPitch(AxesSI_t Axes, float roll);

/**
 *
 */
float getRoll(AxesSI_t Axes);

float getYaw(AxesSI_t Mag, float pitch, float roll);

int16_t getDegree(float rawval);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif /* ANALYZE_H_ */