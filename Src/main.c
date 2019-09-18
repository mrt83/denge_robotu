/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "definitions.h"
#include "timer.h"
#include "mpu6050.h"
#include "interpolation.h"
#include <stdio.h>
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void programInit(void);
void programTask(void);
void workingLed(void);
void i2c_mpu6050_Task(void);
void pidTask(void);
void uartTask(void);
void motorControl(void);
void pwm_proces();
void pwm_proces_abs();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t syc=0;
uint8_t data[7]="MERHABA";
uint8_t rec_data[3];
uint8_t sayi=5;
uint16_t zaman=100;
uint8_t buffer[100],n=0,ilk_deneme=0;
uint8_t tek_sayi[4];
char str[50];
float sagMotorPwmDutyValue = 0.0;
float solMotorPwmDutyValue = 0.0;
PID motorPID;

extern uint8_t dizi[];

float A_data[6];
float temp_data[2];
float G_data[6];
float Xa=0,Ya=0,Za=0;
float temp=0;
float Xg=0,Yg=0,Zg=0;
float xa_derece=0,yg_derece=0;
float duty_izleme=0;

float aci_kalib;

int8_t yon=0;

extern uint8_t syc;






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
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
  HAL_TIM_Base_Start_IT(&htim1);
  //HAL_UART_Receive_IT(&huart1,(uint8_t *)rx_buffer,10);
  //__HAL_UART_ENABLE_IT(&huart1,UART_IT_RXNE);
  HAL_Delay(150);
  programInit();
  uint32_t sayac;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)  //*****************************************************************************************************
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  startTimeScanFlags();
	  programTask();

	/* if(scan2Msec)
	 {
		 HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	 }*/

	  clearTimeScanFlags();
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 35800;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, motor_sag_geri_Pin|motor_sag_ileri_Pin|motor_sol_ileri_Pin|motor_sol_geri_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : motor_sag_geri_Pin motor_sag_ileri_Pin motor_sol_ileri_Pin motor_sol_geri_Pin */
  GPIO_InitStruct.Pin = motor_sag_geri_Pin|motor_sag_ileri_Pin|motor_sol_ileri_Pin|motor_sol_geri_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */




void programInit(void)
{

	aci_kalib=5;
	xAxisValue_eski=0;
	yAxisValue_eski=0;
	zAxisValue_eski=0;
	yGyroValue_eski=0;
	zGyroValue_eski=0;

    MPU6050_Init(MPU6050_DEV_1_ADDRESS);
    motorPID.EPSILON = 0;  //hata_payi
    motorPID.DT = 0.01;		 //zaman sabiti
    motorPID.KP = 1;
    motorPID.KI = 1;
    motorPID.KD = 0;
    motorPID.PID_ERROR_LIMIT = 60.0;
    motorPID.INTEGRAL_INITIAL_VALUE = 0.0;
    motorPID.INTEGRAL_MIN_VAL = 0.0;
	motorPID.INTEGRAL_MAX_VAL = 80.0;
	motorPID.MAX_OUTPUT = 100.0;
	pid_reset(&motorPID);
}

void programTask(void)
{
	//workingLed();
	//motorControl();
	MPU6050_Task(MPU6050_DEV_1_ADDRESS);
	///pwm_proces();
	pidTask();
	//pwm_proces_abs();
	uartTask();



	//        	U_8 dev_address = MPU6050_DEV_1_ADDRESS;
	//        	U_16 reg_address = 0x3B; //Read X axis(LSB);
	//        	U_8 timeout = 200;
	//        	U_16 readDataSequence[16];
	//
	//			result = I2CRead(dev_address, reg_address, (U_8*)&readDataSequence, sizeof(readDataSequence), timeout;
//
//	U_8 writeData[2];
//	writeData[0] = 0x0F;
//	writeData[1] = 0xF8;
//	I2CWrite(AD7997_0_ADDR_AGND, 0x02, (U_8*)&writeData, sizeof(writeData), 200, FALSE);
//	I2CWrite(AD7997_0_ADDR_VDD,  0x02, (U_8*)&writeData, sizeof(writeData), 200, FALSE);
}

void i2c_mpu6050_Task(void)
{

	  // A_data[0]=mpu6050_read(0x3B); //Read X axis(LSB)
	  // A_data[1]=mpu6050_read(0x3C); //Read X axis(MSB)
	  // A_data[2]=mpu6050_read(0x3D); //Read Y axis(LSB)
	  // A_data[3]=mpu6050_read(0x3E); //Read Y axis(MSB)
	  // A_data[4]=mpu6050_read(0x3F); //Read Z axis(LSB)
	  // A_data[5]=mpu6050_read(0x40); //Read Z axis(MSB)

	  // temp_data[0]=mpu6050_read(0x41);
	  // temp_data[1]=mpu6050_read(0x42);

	  // G_data[0]=mpu6050_read(0x43); //Read X axis(LSB)
	  // G_data[1]=mpu6050_read(0x44); //Read X axis(MSB)
	  // G_data[2]=mpu6050_read(0x45); //Read Y axis(LSB)
	  // G_data[3]=mpu6050_read(0x46); //Read Y axis(MSB)
	  // G_data[4]=mpu6050_read(0x47); //Read Z axis(LSB)
	  // G_data[5]=mpu6050_read(0x48); //Read Z axis(MSB)

	  // Xa=make16(A_data[0],A_data[1]);

	  // Ya=make16(A_data[2],A_data[3]);
	  // Za=make16(A_data[4],A_data[5]);

	   //temp=make16(temp_data[0],temp_data[1]);
	   //temp=temp/(int16)340 + (int16)36.53;

	  // Xg=make16(G_data[0],G_data[1]);
	  // Yg=make16(G_data[2],G_data[3]);
	  // Zg=make16(G_data[4],G_data[5]);

	 //  float Heading = atan2((signed int16)Ya,(signed int16)Xa)* 180 / pi + 180;

	 // sprintf(buffer,"xa =%Ld  ya =%Ld  za =%Ld  xg = %Ld yg = %Ld  zg = %Ld temp=%Ld \t\r",Xa,Ya,Za,Xg,Yg,Zg,temp);
	 //   fprintf(RS232,"%s",buffer);

}


void pwm_proces_abs()
{
	static float pwm_min_val;

	        sagMotorPwmDutyValue = motorPID.output;
			solMotorPwmDutyValue = motorPID.output;

			if (pitch >0)
			{
				HAL_GPIO_WritePin(GPIOB, SAG_MOTOR_DIR_BACK_PIN, SET);
				HAL_GPIO_WritePin(GPIOB, SAG_MOTOR_DIR_FORWARD_PIN, RESET);

			}
			else if (pitch<0)
			{

				HAL_GPIO_WritePin(GPIOB, SAG_MOTOR_DIR_BACK_PIN, RESET);
				HAL_GPIO_WritePin(GPIOB, SAG_MOTOR_DIR_FORWARD_PIN, SET);
			}
			else
			{
				HAL_GPIO_WritePin(GPIOB, SAG_MOTOR_DIR_BACK_PIN, RESET);
				HAL_GPIO_WritePin(GPIOB, SAG_MOTOR_DIR_FORWARD_PIN, RESET);
			}

			if (pitch > 0)
			{
				HAL_GPIO_WritePin(GPIOB, SOL_MOTOR_DIR_BACK_PIN, SET);
				HAL_GPIO_WritePin(GPIOB, SOL_MOTOR_DIR_FORWARD_PIN, RESET);

			}
			else if(pitch < 0)
			{

				HAL_GPIO_WritePin(GPIOB, SOL_MOTOR_DIR_BACK_PIN, RESET);
				HAL_GPIO_WritePin(GPIOB, SOL_MOTOR_DIR_FORWARD_PIN, SET);
			}

	        else
	        {
		       HAL_GPIO_WritePin(GPIOB, SAG_MOTOR_DIR_BACK_PIN, RESET);
		       HAL_GPIO_WritePin(GPIOB, SAG_MOTOR_DIR_FORWARD_PIN, RESET);
	        }


			if (pitch > 60 || pitch < -60)
			{

				htim2.Instance->CCR1 = 0;
				htim2.Instance->CCR2 = 0;
			}

			else if (pitch < aci_kalib && pitch > -1 * aci_kalib)
			{
				htim2.Instance->CCR1 = (uint16_t) 0;
				htim2.Instance->CCR2 = (uint16_t) 0;

				/*HAL_GPIO_WritePin(GPIOB, SOL_MOTOR_DIR_BACK_PIN, SET);
				 HAL_GPIO_WritePin(GPIOB, SOL_MOTOR_DIR_FORWARD_PIN, SET);

				 HAL_GPIO_WritePin(GPIOB, SAG_MOTOR_DIR_BACK_PIN, SET);
				 HAL_GPIO_WritePin(GPIOB, SAG_MOTOR_DIR_FORWARD_PIN, SET);*/
			}

			else
			{

				float tempPwmValue1 = limitedInterpolation(sagMotorPwmDutyValue, 0,
						100, 20000, 60000);
				float tempPwmValue2 = limitedInterpolation(solMotorPwmDutyValue, 0,
						100, 20000, 60000);
				duty_izleme = tempPwmValue1;

				htim2.Instance->CCR1 = (uint16_t) (tempPwmValue1);
				htim2.Instance->CCR2 = (uint16_t) (tempPwmValue2);
			}
}

void pwm_proces()
{
	static float pwm_min_val;

	        sagMotorPwmDutyValue = motorPID.output;
			solMotorPwmDutyValue = motorPID.output;

			if (sagMotorPwmDutyValue < 0) {
				sagMotorPwmDutyValue = -1 * sagMotorPwmDutyValue;
				HAL_GPIO_WritePin(GPIOB, SAG_MOTOR_DIR_BACK_PIN, SET);
				HAL_GPIO_WritePin(GPIOB, SAG_MOTOR_DIR_FORWARD_PIN, RESET);
				pwm_min_val = 30000;
			}
			else if (sagMotorPwmDutyValue>0)
			{
				pwm_min_val = 20000;
				HAL_GPIO_WritePin(GPIOB, SAG_MOTOR_DIR_BACK_PIN, RESET);
				HAL_GPIO_WritePin(GPIOB, SAG_MOTOR_DIR_FORWARD_PIN, SET);
			}
			else
			{
				HAL_GPIO_WritePin(GPIOB, SAG_MOTOR_DIR_BACK_PIN, RESET);
				HAL_GPIO_WritePin(GPIOB, SAG_MOTOR_DIR_FORWARD_PIN, RESET);
			}

			if (solMotorPwmDutyValue < 0) {
				solMotorPwmDutyValue = -1 * solMotorPwmDutyValue;
				HAL_GPIO_WritePin(GPIOB, SOL_MOTOR_DIR_BACK_PIN, SET);
				HAL_GPIO_WritePin(GPIOB, SOL_MOTOR_DIR_FORWARD_PIN, RESET);
				pwm_min_val = 30000;
			}
			else if(solMotorPwmDutyValue > 0)
			{
				pwm_min_val = 20000;
				HAL_GPIO_WritePin(GPIOB, SOL_MOTOR_DIR_BACK_PIN, RESET);
				HAL_GPIO_WritePin(GPIOB, SOL_MOTOR_DIR_FORWARD_PIN, SET);
			}

			else
					{
						HAL_GPIO_WritePin(GPIOB, SAG_MOTOR_DIR_BACK_PIN, RESET);
									HAL_GPIO_WritePin(GPIOB, SAG_MOTOR_DIR_FORWARD_PIN, RESET);
					}


			if (pitch > 60 || pitch < -60)
			{

				htim2.Instance->CCR1 = 0;
				htim2.Instance->CCR2 = 0;
			}

			else if (pitch < aci_kalib && pitch > -1 * aci_kalib)
			{
				htim2.Instance->CCR1 = (uint16_t) 0;
				htim2.Instance->CCR2 = (uint16_t) 0;

				/*HAL_GPIO_WritePin(GPIOB, SOL_MOTOR_DIR_BACK_PIN, SET);
				 HAL_GPIO_WritePin(GPIOB, SOL_MOTOR_DIR_FORWARD_PIN, SET);

				 HAL_GPIO_WritePin(GPIOB, SAG_MOTOR_DIR_BACK_PIN, SET);
				 HAL_GPIO_WritePin(GPIOB, SAG_MOTOR_DIR_FORWARD_PIN, SET);*/
			}

			else
			{

				float tempPwmValue1 = limitedInterpolation(sagMotorPwmDutyValue, 0,
						100, pwm_min_val, 60000);
				float tempPwmValue2 = limitedInterpolation(solMotorPwmDutyValue, 0,
						100, pwm_min_val, 60000);
				duty_izleme = tempPwmValue1;

				htim2.Instance->CCR1 = (uint16_t) (tempPwmValue1);
				htim2.Instance->CCR2 = (uint16_t) (tempPwmValue2);
			}
}

void pidTask(void)  //PID işlemi burada yapılıyor.
{

	static float pwm_min_val=20000.0;
	if (scan10Msec)
	{

		if(ilk_deneme==0)
		{
			if(pitch!=0)
				return;
			else
			{
				ilk_deneme=1;
			}

		}

		static float pitch_abs;
		if(pitch<0)
		{
			pitch_abs=-1*pitch;
		}
		else
			pitch_abs=pitch;


		motorPID.setValue = 0;	//olması gereken.
		motorPID.feedbackValue = pitch_abs;
		pid_control(&motorPID, positiveLimiting);
		if(motorPID.error<aci_kalib  && motorPID.error > -1 * aci_kalib)
		{
			pid_reset(&motorPID);

			pitch=0;

		}

		 sagMotorPwmDutyValue = motorPID.output;
					solMotorPwmDutyValue = motorPID.output;

//					if(yon==0)
//					{
//
//					}
//					else
//					{
//
//					}


					if (pitch >0)
					{
						HAL_GPIO_WritePin(GPIOB, SAG_MOTOR_DIR_BACK_PIN, SET);
						HAL_GPIO_WritePin(GPIOB, SAG_MOTOR_DIR_FORWARD_PIN, RESET);
						HAL_GPIO_WritePin(GPIOB, SOL_MOTOR_DIR_BACK_PIN, SET);
						HAL_GPIO_WritePin(GPIOB, SOL_MOTOR_DIR_FORWARD_PIN, RESET);
						//pwm_min_val=20000.0;

						yon=90;

					}
					else if (pitch<0)
					{

						HAL_GPIO_WritePin(GPIOB, SAG_MOTOR_DIR_BACK_PIN, RESET);
						HAL_GPIO_WritePin(GPIOB, SAG_MOTOR_DIR_FORWARD_PIN, SET);
						HAL_GPIO_WritePin(GPIOB, SOL_MOTOR_DIR_BACK_PIN, RESET);
						HAL_GPIO_WritePin(GPIOB, SOL_MOTOR_DIR_FORWARD_PIN, SET);
						//pwm_min_val=50000;
						yon=0;
					}
					else
					{
						HAL_GPIO_WritePin(GPIOB, SAG_MOTOR_DIR_BACK_PIN, RESET);
						HAL_GPIO_WritePin(GPIOB, SAG_MOTOR_DIR_FORWARD_PIN, RESET);
						HAL_GPIO_WritePin(GPIOB, SOL_MOTOR_DIR_BACK_PIN, RESET);
						HAL_GPIO_WritePin(GPIOB, SOL_MOTOR_DIR_FORWARD_PIN, RESET);
						yon=0;
					}


					if (pitch > 60 || pitch < -60)
					{

						htim2.Instance->CCR1 = 0;
						htim2.Instance->CCR2 = 0;
					}

					else if (pitch < aci_kalib && pitch > -1 * aci_kalib)
					{
						htim2.Instance->CCR1 = (uint16_t) 0;
						htim2.Instance->CCR2 = (uint16_t) 0;
						yon=45;

						/*HAL_GPIO_WritePin(GPIOB, SOL_MOTOR_DIR_BACK_PIN, SET);
						 HAL_GPIO_WritePin(GPIOB, SOL_MOTOR_DIR_FORWARD_PIN, SET);

						 HAL_GPIO_WritePin(GPIOB, SAG_MOTOR_DIR_BACK_PIN, SET);
						 HAL_GPIO_WritePin(GPIOB, SAG_MOTOR_DIR_FORWARD_PIN, SET);*/
					}

					else
					{

						float tempPwmValue1 = limitedInterpolation(sagMotorPwmDutyValue, 0,
								100, 20000, 60000);
						float tempPwmValue2 = limitedInterpolation(solMotorPwmDutyValue, 0,
								100, 20000, 60000);
						duty_izleme = tempPwmValue1;

						htim2.Instance->CCR1 = (uint16_t) (tempPwmValue1);
						htim2.Instance->CCR2 = (uint16_t) (tempPwmValue2);
					}
	}

}
void uartTask(void)
{
	if (scan1Sec)
		{
		HAL_UART_Receive_IT(&huart1,(uint8_t *)rx_buffer,7);
		if(syc>=7)    /// DESİMAL 48     ASCILL 0
		{
			if (rx_buffer[0] == 'S' && rx_buffer[6] == 'B')
			{
				if (rx_buffer[1] == 'K' && rx_buffer[2] == 'P')
				{
					 motorPID.KP= (float)((rx_buffer[3]-48)*10+(rx_buffer[4]-48)+(float)((float)(rx_buffer[5]-48)/10));
					/* n = sprintf((char *) buffer, "%.2f \n",motorPID.KP);
					 HAL_UART_Transmit(&huart1,buffer,n,10000);*/

				}
				if (rx_buffer[1] == 'K' && rx_buffer[2] == 'I')
				{
					motorPID.KI= (float)((rx_buffer[3]-48)*10+(rx_buffer[4]-48)+(float)((float)(rx_buffer[5]-48)/10));
				}
				if (rx_buffer[1] == 'K' && rx_buffer[2] == 'D')
				{
					motorPID.KD= (float)((rx_buffer[3]-48)*10+(rx_buffer[4]-48)+(float)((float)(rx_buffer[5]-48)/10));
				}
				if (rx_buffer[1] == 'A' && rx_buffer[2] == 'K')
				{
					aci_kalib= (float) ((rx_buffer[3] - 48) * 10 + (rx_buffer[4] - 48) + (float) ((float) (rx_buffer[5] - 48) / 10));
				}

		   }
		syc=0;
			// __HAL_UART_ENABLE_IT(&huart1,UART_IT_RXNE);
		}
		}

		if (scan2Msec)
			{

		//  n= sprintf((char *)buffer," %.2f   %.2f   %.2f  %.2f  %.2f  %.2f  %.2f\n",x_ang,y_ang,xAxisValue,yAxisValue,zAxisValue,pitch);  // lazım olan y_ang

		//n = sprintf((char *) buffer, "%.2f   %.2f   %.2f   %.2f   %.2f   %.2f \n",motorPID.KP,motorPID.KI,motorPID.KD,pitch,motorPID.error,duty_izleme);

	   // n = sprintf((char *) buffer, "%.2f  %.2f \n",pitch,duty_izleme);
		//	n = sprintf((char *) buffer, "%d",(int8_t)pitch);

			tek_sayi[0]=205;
			tek_sayi[3]=230;
			tek_sayi[1]=(uint8_t)(pitch+90);
			tek_sayi[2]=(uint8_t)(yon);
		HAL_UART_Transmit(&huart1,tek_sayi,4,10000);


	}
}
void motorControl(void)
{
	if(scan100Msec)
	{
		//
	}
}
void workingLed(void)
{
	if (scan1Sec)
	{
		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
