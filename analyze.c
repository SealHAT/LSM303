/*
 * analyze.c
 *
 * Created: 3/7/2018 1:47:44 PM
 *  Author: hpan5
 */ 
#include "analyze.h"

float get_pitch(AxesSI_t Axes){
	float pitch = 0;
	/*Apply trigonometry to get the pic*/
	pitch = (atan(Axes.yAxis/sqrt(pow(Axes.zAxis,2)+pow(Axes.xAxis,2))))*(180.0/PI);
	return pitch;
}

float get_roll(AxesSI_t Axes){
	float roll = 0;
	/*Apply trigonometry to get the pic*/
	roll = (atan(-Axes.xAxis/Axes.zAxis))*(180.0/PI);
	return roll;
}

float get_yaw(AxesSI_t Axes){
	float yaw = 0;
	yaw = (atan(Axes.zAxis/sqrt(pow(Axes.xAxis,2)+pow(Axes.yAxis,2))))*(180.0/PI);
	return yaw;
}

uint32_t get_odba(AxesSI_t Axes){
	uint32_t odba = 0;
	odba = sqrt(pow(Axes.xAxis,2) + pow(Axes.yAxis,2) + pow(Axes.zAxis,2));
	return odba;
}
