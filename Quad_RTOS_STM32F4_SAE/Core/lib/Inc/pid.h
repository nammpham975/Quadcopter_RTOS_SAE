/*
 * pid.h
 *
 *  Created on: Oct 31, 2024
 *      Author: DELL
 */

#ifndef LIB_INC_PID_H_
#define LIB_INC_PID_H_

#include"mpu6050.h"

typedef struct{
	float previous_input;

	float p_gain;
	float i_gain;
	float d_gain;

	float proportional;
	float integral;
	float derivative;

	float scale;

	int output;
	int max_range;
}pid_data;

void pid_calculate(float input,float channel,pid_data*pid);

#endif /* LIB_INC_PID_H_ */
