/*
 * mixer.h
 *
 *  Created on: Nov 1, 2024
 *      Author: DELL
 */

#ifndef LIB_INC_MIXER_H_
#define LIB_INC_MIXER_H_

#include"stdint.h"

typedef struct{
	uint16_t esc_1;
	uint16_t esc_2;
	uint16_t esc_3;
	uint16_t esc_4;
}quadrotor;

typedef struct{
	uint16_t esc_1;
	uint16_t esc_2;
	uint16_t esc_3;
	uint16_t esc_4;
	uint16_t esc_5;
	uint16_t esc_6;
}hexarotor;

void calc_quad_mixer(quadrotor* uav,uint16_t throttle,int16_t pitch,int16_t roll,int16_t yaw);
void calc_quad_plus_mixer(quadrotor* uav,uint16_t throttle,int16_t pitch,int16_t roll,int16_t yaw);
void calc_hex_x(hexarotor* uav,uint16_t throttle,int16_t pitch,int16_t roll,int16_t yaw);
void calc_hex_plus(hexarotor* uav,uint16_t throttle,int16_t pitch,int16_t roll,int16_t yaw);

#endif /* LIB_INC_MIXER_H_ */
