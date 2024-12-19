/*
 * bmp180.h
 *
 *  Created on: Oct 31, 2024
 *      Author: DELL
 */

#ifndef LIB_INC_BMP180_H_
#define LIB_INC_BMP180_H_

#include"stdint.h"

typedef struct {
	short AC1;
	short AC2;
	short AC3;
	unsigned short AC4;
	unsigned short AC5;
	unsigned short AC6;
	short B1;
	short B2;
	short MB;
	short MC;
	short MD;

	long X1;
	long X2;
	long X3;
	long B3;
	unsigned long B4;
	long B5;
	long B6;
	unsigned long B7;

	uint32_t UT;
	uint32_t UP;
	uint8_t oss;
}bmp180_calibration_t;

typedef struct{
	int32_t Pressure;
	int32_t Temperature;
	float altitude;
}bmp180_data_t;

void BMP180_Read_Calibration_Data(bmp180_calibration_t *data);
void BMP180_Read_UTemp(bmp180_calibration_t *data);
void BMP180_Read_UPress(bmp180_calibration_t *data,bmp180_data_t *atm);
void BMP180_Calculate_RTemp(bmp180_calibration_t *data,bmp180_data_t *atm);
void BMP180_Calculate_RPress(bmp180_calibration_t *data,bmp180_data_t *atm);
void BMP180_Calculate_Altitude(bmp180_data_t *atm);

#endif /* LIB_INC_BMP180_H_ */
