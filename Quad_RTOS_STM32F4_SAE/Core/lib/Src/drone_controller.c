/*
 * drone_controller.c
 *
 *  Created on: Nov 7, 2024
 *      Author: DELL
 */
#include"drone_controller.h"

float mapDroneController(uint16_t channel_value,uint16_t min_in,uint16_t max_in,float min_out,float max_out){
	int16_t channel_value_i16 = channel_value;
	int16_t min_i16 = min_in;
	int16_t max_i16 = max_in;
	float channel = channel_value_i16;
	float min = min_i16;
	float max = max_i16;
	float result = (channel - min)*(max_out - min_out + 1)/(max - min + 1) + min_out;
	return result;
}
