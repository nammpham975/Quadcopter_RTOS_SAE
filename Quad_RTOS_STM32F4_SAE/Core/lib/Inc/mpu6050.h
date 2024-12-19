/*
 * mpu6050.h
 *
 *  Created on: Oct 27, 2024
 *      Author: DELL
 */

#ifndef LIB_INC_MPU6050_H_
#define LIB_INC_MPU6050_H_

#include"stdint.h"

typedef struct{
	int16_t ax_raw;
	int16_t ay_raw;
	int16_t az_raw;

	float ax;
	float ay;
	float az;

	int16_t gx_raw;
	int16_t gy_raw;
	int16_t gz_raw;

	float gx;
	float gy;
	float gz;

	float gx_cal;
	float gy_cal;
	float gz_cal;

	int16_t temp_raw;

	float temp;
}mpu6050_raw_t;

typedef struct{
	float roll_accel;
	float pitch_accel;
	float yaw_accel;

	float roll_gyro;
	float pitch_gyro;
	float yaw_gyro;

	float roll;
	float pitch;
	float yaw;
}mpu6050_angle_t;

int MPU6050_Init();
void MPU6050_Calibrate(mpu6050_raw_t *raw);
void MPU6050_Read_Data(mpu6050_raw_t *raw);
void MPU6050_Read_Angle(mpu6050_raw_t *raw,mpu6050_angle_t *angle);

#endif /* LIB_INC_MPU6050_H_ */
