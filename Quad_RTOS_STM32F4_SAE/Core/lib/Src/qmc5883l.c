/*
 * qmc5883l.c
 *
 *  Created on: Nov 5, 2024
 *      Author: DELL
 */
#include"qmc5883l.h"
#include"math.h"
#include"stm32f4xx_hal.h"

extern I2C_HandleTypeDef hi2c1;

#define rad_to_deg 180/3.145926;

#define QMC5883L_DEFAULT_ADDRESS 0x1A
#define QMC5883L_X_LSB_OUT 0x00
#define QMC5883L_X_MSB_OUT 0x01
#define QMC5883L_Y_LSB_OUT 0x02
#define QMC5883L_Y_MSB_OUT 0x03
#define QMC5883L_Z_LSB_OUT 0x04
#define QMC5883L_Z_MSB_OUT 0x05
#define QMC5883L_STATUS_REG	0x06
#define QMC5883L_TEMP_LOW	0x07
#define QMC5883L_TEMP_HIGH	0x08
#define QMC5883L_CONTROL_REG_1	0x09
#define QMC5883L_CONTROL_REG_2	0x0A
#define QMC5883L_PERIOD_REG	0x0B
#define QMC5883L_ID_REG	0x0D

int QMC5883L_Init(){
	uint8_t data_write = 0x01;
	uint8_t check;

	HAL_I2C_Mem_Write(&hi2c1, QMC5883L_DEFAULT_ADDRESS, QMC5883L_PERIOD_REG, 1, &data_write, 1, 100);
	data_write = 0x0C | 0x01;
	HAL_I2C_Mem_Write(&hi2c1, QMC5883L_DEFAULT_ADDRESS, QMC5883L_CONTROL_REG_1, 1, &data_write, 1, 100);

	HAL_I2C_Mem_Read(&hi2c1, QMC5883L_DEFAULT_ADDRESS, QMC5883L_STATUS_REG, 1, &check, 1, 10);
	if((check & 0x01) == 1){
		return 1;
	}
	return 0;
}

void QMC5883L_Read(qmc5883l_angle_t *sensor){
	uint8_t data_read[6];
	uint8_t check;
	HAL_I2C_Mem_Read(&hi2c1, QMC5883L_DEFAULT_ADDRESS, QMC5883L_X_LSB_OUT, 1, data_read, 6, 10);
	sensor->mx_raw = (data_read[1] << 8)|data_read[0];
	sensor->my_raw = (data_read[3] << 8)|data_read[2];
	sensor->mz_raw = (data_read[5] << 8)|data_read[4];
	sensor->yaw_raw = atan2f(sensor->my_raw,sensor->mx_raw)*rad_to_deg;
	if(sensor->yaw_raw > 0){
		sensor->yaw = sensor->yaw_raw;
	}
	else{
		sensor->yaw = 360 + sensor->yaw_raw;
	}
}
