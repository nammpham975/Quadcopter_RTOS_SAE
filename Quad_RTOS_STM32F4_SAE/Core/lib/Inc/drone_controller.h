/*
 * drone_controller.h
 *
 *  Created on: Nov 7, 2024
 *      Author: DELL
 */

#ifndef LIB_INC_DRONE_CONTROLLER_H_
#define LIB_INC_DRONE_CONTROLLER_H_

#include"stdint.h"

typedef struct{
	uint16_t throttle;
	float roll;
	float pitch;
	float yaw;
	int yaw_rate;
}drone_controller_t;

float mapDroneController(uint16_t channel_value,uint16_t min_i,uint16_t max_i,float min_o,float max_o);

#endif /* LIB_INC_DRONE_CONTROLLER_H_ */
