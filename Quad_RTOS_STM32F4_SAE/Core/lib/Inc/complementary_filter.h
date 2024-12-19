/*
 * complementary_filter.h
 *
 *  Created on: Nov 24, 2024
 *      Author: DELL
 */

#ifndef LIB_INC_COMPLEMENTARY_FILTER_H_
#define LIB_INC_COMPLEMENTARY_FILTER_H_

#include"stdint.h"

typedef struct{
	float angle;
	float angle_rate;
	float HPF;
	float LPF;
	float contantTime;
	float sampleTime;
	float alpha;
}cpFilter_t;

void filter(cpFilter_t*euler);

#endif /* LIB_INC_COMPLEMENTARY_FILTER_H_ */
