/*
 * ppm.h
 *
 *  Created on: Oct 31, 2024
 *      Author: DELL
 */

#ifndef LIB_INC_PPM_H_
#define LIB_INC_PPM_H_

#include"stdint.h"

typedef struct{
	uint16_t channel[11];
	uint8_t channel_count;
	uint16_t meassureTime;
	uint16_t lastMeassure;
}ppm_t;

#endif /* LIB_INC_PPM_H_ */
