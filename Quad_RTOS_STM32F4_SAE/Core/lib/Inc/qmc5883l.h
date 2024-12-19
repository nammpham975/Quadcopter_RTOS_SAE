/*
 * qmc5883l.h
 *
 *  Created on: Nov 5, 2024
 *      Author: DELL
 */

#ifndef LIB_INC_QMC5883L_H_
#define LIB_INC_QMC5883L_H_

#include "stdint.h"

typedef struct{
	int16_t mx_raw;
	int16_t my_raw;
	int16_t mz_raw;
	float yaw_raw;
	float yaw;
	float yaw_setpoint;
}qmc5883l_angle_t;

int QMC5883L_Init();
void QMC5883L_Read(qmc5883l_angle_t *sensor);

#endif /* LIB_INC_QMC5883L_H_ */
