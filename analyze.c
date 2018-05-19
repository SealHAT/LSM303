/*
 * analyze.c
 *
 * Created: 3/7/2018 1:47:44 PM
 *  Author: hpan5
 */ 
#include "analyze.h"

int32_t get_pitch(AxesSI_t* Axes){
	int32_t pitch = 0;
	/*Apply trigonometry to get the pic*/
	pitch = (atan(-Axes->yAxis/sqrt(pow(Axes->zAxis,2)+pow(Axes->xAxis,2))))*(180.0/PI);
	return pitch;
}

int32_t get_roll(AxesSI_t* Axes){
	int32_t roll = 0;
	/*Apply trigonometry to get the pic*/
	roll = (atan(-Axes->xAxis/Axes->zAxis))*(180.0/PI);
	return roll;
}

int32_t get_yaw(AxesSI_t* Axes){
	int32_t yaw = 0;
	yaw = (atan(Axes->zAxis/sqrt(pow(Axes->xAxis,2)+pow(Axes->yAxis,2))))*(180.0/PI);
	return yaw;
}
/*
int32_t motion_detect(AxesRaw_t* Axes){
	uint8_t reg_detect = 0x00; 
	switch()
	if(Axes->xAxis > 0x132){ //x > 0.3g
		reg_detect |= SWAY; //SWAY bit is enabled
	}else{
		reg_detect &= ~SWAY; //Clear SWAY bit
	}
	
	if(Axes->yAxis > 0x132){ //y > 0.3g
		reg_detect |= SURGE; //SWAY bit is enabled
		}else{
		reg_detect &= ~SURGE; //Clear SWAY bit
	}
	
	if(Axes->zAxis > 0x52E){ //z > 1.3g
		reg_detect |= HEAVE; //SWAY bit is enabled
		}else{
		reg_detect &= ~HEAVE; //Clear SWAY bit
	}
	
	return reg_detect;
}*/

/*
uint32_t get_odba(AxesSI_t Axes){
	uint32_t odba = 0;
	odba = sqrt(pow(Axes.xAxis,2) + pow(Axes.yAxis,2) + pow(Axes.zAxis,2));
	return odba;
}*/
