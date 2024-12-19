/*
 * pid.c
 *
 *  Created on: Oct 31, 2024
 *      Author: DELL
 */

#include"pid.h"
#include"math.h"

double pid_dt = 0.005;

void pid_calculate(float input,float channel,pid_data*pid){

	pid->proportional = ((channel - input) * pid->p_gain)/pid->scale;
	pid->integral = pid->integral + ((channel - input) * pid->i_gain * pid_dt)/pid->scale;
	pid->derivative = ((channel - input) - pid->previous_input) * pid->d_gain/(pid->scale * pid_dt);

	if(pid->integral >= pid->max_range/(3.0*pid->scale)){
		pid->integral = pid->max_range/(3.0*pid->scale);
	}
	if(pid->integral <= -1 * pid->max_range/(3.0*pid->scale)){
		pid->integral = -1 *pid->max_range/(3.0*pid->scale);
	}

	pid->output = (pid->proportional + pid->integral + pid->derivative);

	if(pid->output >= pid->max_range/pid->scale){
		pid->output = pid->max_range/pid->scale;
	}
	if(pid->output <= -1 * pid->max_range/pid->scale){
		pid->output = -1 * pid->max_range/pid->scale;
	}

	pid->previous_input = channel - input;
}
