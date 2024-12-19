/*
 * compelmentary_filter.c
 *
 *  Created on: Nov 24, 2024
 *      Author: DELL
 */
#include"complementary_filter.h"
#include"math.h"

void filter(cpFilter_t*euler){
	euler->alpha = euler->contantTime/(euler->contantTime + euler->sampleTime);
	euler->HPF = euler->angle + euler->angle_rate * euler->sampleTime;
	euler->angle = euler->HPF*euler->alpha + euler->LPF * (1.0 - euler->alpha);
}
