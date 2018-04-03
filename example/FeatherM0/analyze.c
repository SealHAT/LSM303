/*
 * analyze.c
 *
 * Created: 3/7/2018 1:47:44 PM
 *  Author: hpan5
 */ 
#include "analyze.h"

float getPitch(AxesSI_t Axes, float roll){
	float pitch;
	/*Apply trigonometry to get the pitch*/
	//pitch = (atan(Axes.yAxis/sqrt(pow(Axes.zAxis,2)+pow(Axes.xAxis,2))))*(180.0/PI);
	pitch = atan(Axes.yAxis/(Axes.xAxis*sin(roll) + Axes.zAxis*cos(roll)));
	return pitch;
}

float getRoll(AxesSI_t Axes){
	float roll;
	/*Apply trigonometry to get the roll*/
	roll = (atan(Axes.xAxis/Axes.zAxis));
	return roll;
}

float getYaw(AxesSI_t Mag, float pitch, float roll){
	float resBx, resBy, yaw;
	resBx = (-cos(pitch)*Mag.yAxis) + (sin(roll)*sin(pitch)*Mag.xAxis) + (sin(pitch)*cos(roll)*Mag.zAxis);
	resBy = (cos(roll)*Mag.yAxis) - (sin(roll)*Mag.xAxis);
	yaw = atan(resBx/resBx);
	return yaw;
}

int16_t getDegree(float rawval){
	int16_t val;
	val = rawval*(180.0/PI);
	return val;
}