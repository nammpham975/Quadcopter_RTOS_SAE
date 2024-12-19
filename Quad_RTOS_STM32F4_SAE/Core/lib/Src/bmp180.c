/*
 * bmp180.c
 *
 *  Created on: Oct 31, 2024
 *      Author: DELL
 */

#include"bmp180.h"
#include"stm32f4xx_hal.h"
#include"math.h"

extern I2C_HandleTypeDef hi2c1;

#define BMP180_ADDR                  0xEE
// BMP180 registers
#define BMP180_PROM_START_ADDR          0xAA // E2PROM calibration data start register
#define BMP180_PROM_DATA_LEN            22   // E2PROM length
#define BMP180_CHIP_ID_REG              0xD0 // Chip ID
#define BMP180_VERSION_REG              0xD1 // Version
#define BMP180_CTRL_MEAS_REG            0xF4 // Measurements control (OSS[7.6], SCO[5], CTL[4.0]
#define BMP180_ADC_OUT_MSB_REG          0xF6 // ADC out MSB  [7:0]
#define BMP180_ADC_OUT_LSB_REG          0xF7 // ADC out LSB  [7:0]
#define BMP180_ADC_OUT_XLSB_REG         0xF8 // ADC out XLSB [7:3]
#define BMP180_SOFT_RESET_REG           0xE0 // Soft reset control

// BMP180 control values
#define BMP180_T_MEASURE                0x2E // temperature measurement
#define BMP180_P0_MEASURE               0x34 // pressure measurement (OSS=0, 4.5ms)
#define BMP180_P1_MEASURE               0x74 // pressure measurement (OSS=1, 7.5ms)
#define BMP180_P2_MEASURE               0xB4 // pressure measurement (OSS=2, 13.5ms)
#define BMP180_P3_MEASURE               0xF4 // pressure measurement (OSS=3, 25.5ms)
#define BMP180_PARAM_MG                 3038
#define BMP180_PARAM_MH                -7357
#define BMP180_PARAM_MI                 3791
#define atmPress 101325 					 //Pa

void BMP180_Read_Calibration_Data(bmp180_calibration_t *data){
	uint8_t Callib_Data[22] = {0};
	HAL_I2C_Mem_Read(&hi2c1, BMP180_ADDR, BMP180_PROM_START_ADDR , 1, Callib_Data,22, 1000);

	data->AC1 = ((Callib_Data[0] << 8) | Callib_Data[1]);
	data->AC2 = ((Callib_Data[2] << 8) | Callib_Data[3]);
	data->AC3 = ((Callib_Data[4] << 8) | Callib_Data[5]);
	data->AC4 = ((Callib_Data[6] << 8) | Callib_Data[7]);
	data->AC5 = ((Callib_Data[8] << 8) | Callib_Data[9]);
	data->AC6 = ((Callib_Data[10] << 8) | Callib_Data[11]);
	data->B1 = ((Callib_Data[12] << 8) | Callib_Data[13]);
	data->B2 = ((Callib_Data[14] << 8) | Callib_Data[15]);
	data->MB = ((Callib_Data[16] << 8) | Callib_Data[17]);
	data->MC = ((Callib_Data[18] << 8) | Callib_Data[19]);
	data->MD = ((Callib_Data[20] << 8) | Callib_Data[21]);

	data->oss = 0;
}

void BMP180_Read_UTemp(bmp180_calibration_t *data){
	uint8_t dataToWrite = BMP180_T_MEASURE;
	uint8_t Temp_RAW[2] = {0};
	HAL_I2C_Mem_Write(&hi2c1, BMP180_ADDR, BMP180_CTRL_MEAS_REG, 1, &dataToWrite, 1, 1000);
	HAL_I2C_Mem_Read(&hi2c1, BMP180_ADDR, BMP180_ADC_OUT_MSB_REG, 1, Temp_RAW, 2, 1000);
	data->UT = ((Temp_RAW[0]<<8) + Temp_RAW[1]);
}

void BMP180_Read_UPress(bmp180_calibration_t *data,bmp180_data_t *atm){
	uint8_t datatowrite = BMP180_P0_MEASURE + (data->oss<<6);
	uint8_t Press_RAW[3] = {0,0,0};
	uint8_t cmd;
	HAL_I2C_Mem_Write(&hi2c1, BMP180_ADDR, BMP180_CTRL_MEAS_REG, 1, &datatowrite, 1, 1000);
	switch (data->oss) {
	case (0):
		cmd= BMP180_P0_MEASURE;
		break;
	case (1):
		cmd= BMP180_P1_MEASURE;
		break;
	case (2):
		cmd= BMP180_P2_MEASURE;
		break;
	case (3):
		cmd= BMP180_P3_MEASURE;
		break;
	}
	HAL_I2C_Mem_Write(&hi2c1, BMP180_ADDR, BMP180_CTRL_MEAS_REG, 8, &cmd, 1, 1000);
	HAL_I2C_Mem_Read(&hi2c1, BMP180_ADDR, BMP180_ADC_OUT_MSB_REG, 1, Press_RAW, 3, 1000);
	data->UP = (((Press_RAW[0]<<16)+(Press_RAW[1]<<8)+Press_RAW[2]) >> (8-data->oss));
}

void BMP180_Calculate_RTemp(bmp180_calibration_t *data,bmp180_data_t *atm){
	data->X1 = ((data->UT - data->AC6) * (data->AC5/(pow(2,15))));
	data->X2 = ((data->MC*(pow(2,11)))/(data->X1 + data->MD));
	data->B5 = data->X1 + data->X2;
	atm->Temperature = ((data->B5 + 8)/(pow(2,4)))/10.0;
}

void BMP180_Calculate_RPress(bmp180_calibration_t *data,bmp180_data_t *atm){
	//data->X1 = ((data->UT - data->AC6) * (data->AC5/(pow(2,15))));
	//data->X2 = ((data->MC * (pow(2,11))) / (data->X1 + data->MD));
	//data->B5 = data->X1 + data->X2;
	data->B6 = data->B5 - 4000;
	data->X1 = (data->B2 * (data->B6 * data->B6/(pow(2,12))))/(pow(2,11));
	data->X2 = data->AC2 * data->B6/(pow(2,11));
	data->X3 = data->X1 + data->X2;
	data->B3 = (((data->AC1*4 + data->X3)<< data->oss) + 2)/4;
	data->X1 = data->AC3 * data->B6/pow(2,13);
	data->X2 = (data->B1 * (data->B6 * data->B6/(pow(2,12))))/(pow(2,16));
	data->X3 = ((data->X1 + data->X2) + 2)/pow(2,2);
	data->B4 = data->AC4 * (uint32_t)(data->X3 + 32768)/(pow(2,15));
	data->B7 = ((uint32_t)data->UP - data->B3)*(50000>>data->oss);
	if(data->B7 < 0x80000000){
		atm->Pressure = (data->B7 * 2)/data->B4;
	}
	else{
		atm->Pressure = (data->B7/data->B4) * 2;
	}
	data->X1 = (atm->Pressure/(pow(2,8)))*(atm->Pressure/(pow(2,8)));
	data->X1 = (data->X1*3038)/(pow(2,16));
	data->X2 = (-7357*atm->Pressure)/(pow(2,16));
	atm->Pressure += (data->X1 + data->X2 + 3791)/(pow(2,4));
}

void BMP180_Calculate_Altitude(bmp180_data_t *atm){
	atm->altitude = 44330 * (1 - (pow((atm->Pressure/(float)atmPress),0.19029495718)));
}
