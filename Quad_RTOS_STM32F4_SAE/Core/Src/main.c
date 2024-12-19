/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include"FreeRTOS.h"
#include"task.h"
#include"mpu6050.h"
#include"bmp180.h"
#include"qmc5883l.h"
#include"ppm.h"
#include"pid.h"
#include"drone_controller.h"
#include"mixer.h"
#include"complementary_filter.h"

#include"checksum.h"
#include"common/mavlink.h"
#include"common/mavlink_msg_attitude.h"
#include"common/mavlink_msg_manual_setpoint.h"
#include"common/mavlink_msg_servo_output_raw.h"
#include"ardupilotmega/mavlink_msg_pid_tuning.h"
#include"minimal/mavlink_msg_heartbeat.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define system_id  0x01
#define imu_id 0x1E
#define pid_id 0xC2
#define servo_id 0x24
#define setpoint_id 0x51
#define hearbeat_id 0x02

#define calibrate 1

#define rad_to_deg 180/3.145926

#define DWT_CTRL (*(volatile uint32_t*)0xE0001000)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
DMA_HandleTypeDef hdma_tim2_up_ch3;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
mpu6050_angle_t angle_imu;
mpu6050_raw_t raw_imu;

bmp180_calibration_t raw_pressure;
bmp180_data_t pressure;

qmc5883l_angle_t magnet_angle;

ppm_t ppm_signal;

mavlink_message_t msg;
mavlink_attitude_t attitude_message;
mavlink_servo_output_raw_t servo_message;
mavlink_manual_setpoint_t setpoint_message;
mavlink_pid_tuning_t pid_message;
mavlink_heartbeat_t heart_message;
uint16_t msg_length;

int msg_select = 0;

drone_controller_t drone_control;

pid_data pid_roll;
pid_data pid_pitch;
pid_data pid_yaw;

cpFilter_t roll_filter;
cpFilter_t pitch_filter;

quadrotor Quadcopter;

int mpu_status = 0;
int qmc_status = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void control_handler(void*parameter){
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	for(;;){
		drone_control.throttle = ppm_signal.channel[3];
		drone_control.roll = mapDroneController(ppm_signal.channel[1], 1000, 2000, -25, 25);
		drone_control.pitch = mapDroneController(ppm_signal.channel[2], 1000, 2000, -25, 25);
		drone_control.yaw_rate = mapDroneController(ppm_signal.channel[4], 1000, 2000, -1, 1);
		drone_control.yaw = drone_control.yaw_rate;

		setpoint_message.thrust = drone_control.throttle;
		setpoint_message.roll = drone_control.roll;
		setpoint_message.pitch = drone_control.pitch;
		setpoint_message.yaw = drone_control.yaw;

		vTaskDelayUntil(&xLastWakeTime,pdMS_TO_TICKS(5));
	}
}

static void pwm_handler(void*parameter){
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	for(;;){
		pid_calculate(angle_imu.roll, drone_control.roll, &pid_roll);
		pid_calculate(angle_imu.pitch, drone_control.pitch, &pid_pitch);

		magnet_angle.yaw_setpoint += drone_control.yaw/5;
		if(magnet_angle.yaw_setpoint >= 360){
			magnet_angle.yaw_setpoint -= 360;
		}
		else if(magnet_angle.yaw_setpoint < 0){
			magnet_angle.yaw_setpoint += 360;
		}

		pid_calculate(magnet_angle.yaw, magnet_angle.yaw_setpoint, &pid_yaw);

		pid_message.P = pid_roll.proportional;
		pid_message.I = pid_roll.integral;
		pid_message.D = pid_roll.derivative;

		calc_quad_mixer(&Quadcopter, drone_control.throttle, pid_pitch.output, pid_roll.output, pid_yaw.output);

		servo_message.servo1_raw = Quadcopter.esc_1;
		servo_message.servo2_raw = Quadcopter.esc_2;
		servo_message.servo3_raw = Quadcopter.esc_3;
		servo_message.servo4_raw = Quadcopter.esc_4;
		if(ppm_signal.channel[5] > 1400){
			htim4.Instance->CCR1 = 800;
			htim4.Instance->CCR2 = 800;
			htim4.Instance->CCR3 = 800;
			htim4.Instance->CCR4 = 800;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
		}
		else{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
			if(drone_control.throttle >= 1200){		// 1550
				htim4.Instance->CCR1 = Quadcopter.esc_1;
				htim4.Instance->CCR2 = Quadcopter.esc_2;
				htim4.Instance->CCR3 = Quadcopter.esc_3;
				htim4.Instance->CCR4 = Quadcopter.esc_4 + 30;

				//htim4.Instance->CCR1 = 0;
				//htim4.Instance->CCR2 = 0;
				//htim4.Instance->CCR3 = 0;
				//htim4.Instance->CCR4 = 0;
			}
			else {
				htim4.Instance->CCR1 = drone_control.throttle;			// * 0.858 + 63
				htim4.Instance->CCR2 = drone_control.throttle;			// * 0.928 + 24
				htim4.Instance->CCR3 = drone_control.throttle;
				htim4.Instance->CCR4 = drone_control.throttle;

				//htim4.Instance->CCR1 = 0;
				//htim4.Instance->CCR2 = 0;
				//htim4.Instance->CCR3 = 0;
				//htim4.Instance->CCR4 = 0;
			}
		}
		vTaskDelayUntil(&xLastWakeTime,pdMS_TO_TICKS(5));
	}
}

static void mpu6050_handler(void*parameter){
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	for(;;){
		MPU6050_Read_Data(&raw_imu);
		MPU6050_Read_Angle(&raw_imu, &angle_imu);
		vTaskDelayUntil(&xLastWakeTime,pdMS_TO_TICKS(5));
	}
}

static void complementary_handler(void*parameter){
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	for(;;){
		roll_filter.angle_rate = angle_imu.roll_gyro;
		roll_filter.LPF = angle_imu.roll_accel;
		pitch_filter.angle_rate = angle_imu.pitch_gyro;
		pitch_filter.LPF = angle_imu.pitch_accel;
		filter(&roll_filter);
		filter(&pitch_filter);
		angle_imu.roll = roll_filter.angle;
		angle_imu.pitch = pitch_filter.angle;

		attitude_message.roll = angle_imu.roll;
		attitude_message.pitch = angle_imu.pitch;
		attitude_message.rollspeed = angle_imu.roll_gyro;
		attitude_message.pitchspeed = angle_imu.pitch_gyro;
		vTaskDelayUntil(&xLastWakeTime,pdMS_TO_TICKS(5));
	}
}

static void mavlink_handler(void* parameter){
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	for(;;){
		if(msg_select == 0){
			msg_length = mavlink_msg_heartbeat_encode(system_id, hearbeat_id, &msg, &heart_message);
			msg_select++;
		}
		else if(msg_select == 1){
			msg_length = mavlink_msg_attitude_encode(system_id, imu_id, &msg, &attitude_message);
			msg_select++;
		}
		else if(msg_select == 2){
			msg_length = mavlink_msg_servo_output_raw_encode(system_id, servo_id, &msg, &servo_message);
			msg_select++;
		}
		else if(msg_select == 3){
			msg_length = mavlink_msg_manual_setpoint_encode(system_id, setpoint_id, &msg, &setpoint_message);
			msg_select++;
		}
		else if(msg_select == 4){
			msg_length = mavlink_msg_pid_tuning_encode(system_id, pid_id, &msg, &pid_message);
			msg_select = 0;
		}
		HAL_UART_Transmit_DMA(&huart2, &msg, msg_length);

		vTaskDelayUntil(&xLastWakeTime,pdMS_TO_TICKS(10));
	}
}

static void qmc5883l_handler(void* parameter){
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	for(;;){
		QMC5883L_Read(&magnet_angle);
		attitude_message.yaw = magnet_angle.yaw;
		vTaskDelayUntil(&xLastWakeTime,pdMS_TO_TICKS(5));
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_0){
		ppm_signal.meassureTime = __HAL_TIM_GET_COUNTER(&htim2)*10;
		__HAL_TIM_SET_COUNTER(&htim2,0);
		ppm_signal.channel_count += 1;
		if(ppm_signal.meassureTime > 3000){
			ppm_signal.channel_count = 0;
		}
		ppm_signal.channel[ppm_signal.channel_count] = ppm_signal.meassureTime;
	}
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C2_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	mpu_status = MPU6050_Init();
	qmc_status = QMC5883L_Init();

	HAL_TIM_Base_Start(&htim2);

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

#if calibrate
	/*
	htim4.Instance->CCR1 = 2000;
	htim4.Instance->CCR2 = 2000;
	htim4.Instance->CCR3 = 2000;
	htim4.Instance->CCR4 = 2000;
	HAL_Delay(2000);
	htim4.Instance->CCR1 = 1000;
	htim4.Instance->CCR2 = 1000;
	htim4.Instance->CCR3 = 1000;
	htim4.Instance->CCR4 = 1000;
	HAL_Delay(1000);
	 */
	HAL_Delay(1000);
	if(mpu_status == 1){
		for(int i = 0 ; i < 2 ; i++){
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
			HAL_Delay(500);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
			HAL_Delay(500);
		}
	}
	else if(mpu_status == 0){
		for(int i = 0 ; i < 2 ; i++){
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
			HAL_Delay(250);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
			HAL_Delay(250);
		}
	}
	HAL_Delay(1000);

	if(qmc_status == 1){
		for(int i = 0 ; i < 3 ; i++){
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
			HAL_Delay(250);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
			HAL_Delay(250);
		}
	}
	else if(qmc_status == 0){
		for(int i = 0 ; i < 3 ; i++){
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
			HAL_Delay(250);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
			HAL_Delay(250);
		}
	}
	MPU6050_Calibrate(&raw_imu);
	QMC5883L_Read(&magnet_angle);
	magnet_angle.yaw_setpoint = magnet_angle.yaw;

#endif
	heart_message.type = 2;
	heart_message.autopilot = 17;
	heart_message.base_mode = 2;
	heart_message.system_status = 3;

	pid_roll.p_gain = 10.9 ;
	pid_roll.i_gain = 0.5;
	pid_roll.d_gain = 0.9;
	pid_roll.max_range = 500;
	pid_roll.scale = 5;

	pid_pitch.p_gain = 10;			//10
	pid_pitch.i_gain = 0.5;			//0.5
	pid_pitch.d_gain = 0.9;			//0.9
	pid_pitch.max_range = 500;
	pid_pitch.scale = 5;

	pid_yaw.p_gain = 15.7;			//14.5
	pid_yaw.i_gain = 0.2;
	pid_yaw.d_gain = 0.8;			//0.8
	pid_yaw.max_range= 500;
	pid_yaw.scale = 5;

	roll_filter.contantTime = 0.495;
	roll_filter.sampleTime = 0.005;

	pitch_filter.contantTime = 0.495;
	pitch_filter.sampleTime = 0.005;

	DWT_CTRL |= (1 << 0);

	SEGGER_SYSVIEW_Conf();
	SEGGER_SYSVIEW_Start();

	xTaskCreate(mpu6050_handler, "mpu6050_task", 100, "mpu6050_task", 4, NULL);
	xTaskCreate(complementary_handler, "filter_task", 100, "compelmentary_filter_task", 4, NULL);
	xTaskCreate(pwm_handler, "pwm_task", 200, "pwm_task", 4, NULL);
	xTaskCreate(qmc5883l_handler, "qmc5883l_task", 100, "qmc5883l_task", 3, NULL);
	xTaskCreate(control_handler,"control_task",100,"control_task",3,NULL);
	xTaskCreate(mavlink_handler, "mavlink_task", 200, "mavlink_task", 2, NULL);

	vTaskStartScheduler();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 250-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 25-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 5000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 57600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
