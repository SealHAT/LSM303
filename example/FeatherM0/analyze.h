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

float pitch_est(AxesSI_t Axes);
float row_est(AxesSI_t Axes);

#endif /* ANALYZE_H_ */