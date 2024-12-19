/*
 * mixer.c
 *
 *  Created on: Nov 1, 2024
 *      Author: DELL
 */
#include"mixer.h"
#include"math.h"

void calc_quad_mixer(quadrotor* uav,uint16_t throttle,int16_t pitch,int16_t roll,int16_t yaw){
	uav->esc_1 = throttle - roll + pitch + yaw;
	uav->esc_2 = throttle + roll - pitch + yaw;
	uav->esc_3 = throttle + roll + pitch - yaw;
	uav->esc_4 = throttle - roll - pitch - yaw;
	if(uav->esc_1 >= 2000){
		uav->esc_1 = 2000;
	}
	if(uav->esc_2 >= 2000){
		uav->esc_2 = 2000;
	}
	if(uav->esc_3 >= 2000){
		uav->esc_3 = 2000;
	}
	if(uav->esc_4 >= 2000){
		uav->esc_4 = 2000;
	}
}

void calc_quad_plus_mixer(quadrotor* uav,uint16_t throttle,int16_t pitch,int16_t roll,int16_t yaw){
	uav->esc_1 = throttle - roll + 0 + yaw;
	uav->esc_2 = throttle + roll + 0 + yaw;
	uav->esc_3 = throttle + 0 + pitch - yaw;
	uav->esc_4 = throttle + 0 - pitch - yaw;
}

void calc_hex_x(hexarotor* uav,uint16_t throttle,int16_t pitch,int16_t roll,int16_t yaw){
	uav->esc_1 = throttle - roll + 0 - yaw;
	uav->esc_2 = throttle + roll + 0 + yaw;
	uav->esc_3 = throttle + 0.5*roll + 0.866*pitch - yaw;
	uav->esc_4 = throttle - 0.5*roll - 0.866*pitch + yaw;
	uav->esc_5 = throttle - 0.5*roll + 0.866*pitch + yaw;
	uav->esc_6 = throttle + 0.5*roll - 0.866*pitch - yaw;
}

void calc_hex_plus(hexarotor* uav,uint16_t throttle,int16_t pitch,int16_t roll,int16_t yaw){
	uav->esc_1 = throttle + 0 + pitch - yaw;
	uav->esc_2 = throttle + 0 - pitch + yaw;
	uav->esc_3 = throttle + 0.866*roll - 0.5*pitch - yaw;
	uav->esc_4 = throttle - 0.866*roll + 0.5*pitch + yaw;
	uav->esc_5 = throttle + 0.866*roll + 0.5*pitch + yaw;
	uav->esc_6 = throttle - 0.866*roll - 0.5*pitch - yaw;
}
