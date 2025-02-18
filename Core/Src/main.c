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
#include "string.h"
#include "stdio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define MaxRpm 120
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint32_t adcValue[2];
volatile GPIO_PinState lastButtonState=GPIO_PIN_SET;
volatile uint8_t joystickEnabled=0;
uint8_t rxDataF=0;
uint8_t rxDataB=0;
uint8_t obstacleFront=0;
uint8_t obstacleBack=0;
uint32_t currentDutyCycleLeft=0;
uint32_t currentDutyCycleRight=0;
uint32_t dutyCycleIncrement=50;
uint32_t targetDutyCycleLeft=0;
uint32_t targetDutyCycleRight=0;
uint32_t targetRPMLeft=0;
uint32_t targetRPMRight=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
 * void updateDutyCycle(uint32_t targetDutyCycleLeft,uint32_t targetDutyCycleRight)
{
	//gradually adjust the left motor duty cycle
	if(currentDutyCycleLeft<targetDutyCycleLeft){
		currentDutyCycleLeft+=dutyCycleIncrement;
		if(currentDutyCycleLeft>999)
			currentDutyCycleLeft=999;
	}

	else if(currentDutyCycleLeft>targetDutyCycleLeft){
		if(currentDutyCycleLeft<dutyCycleIncrement)
			currentDutyCycleLeft=0;
		else{
			currentDutyCycleLeft-=dutyCycleIncrement;
		}
	}

	if(currentDutyCycleRight<targetDutyCycleRight)
	{
		currentDutyCycleRight+=dutyCycleIncrement;
		if(currentDutyCycleRight>999)
			currentDutyCycleRight=999;
	}
	else if(currentDutyCycleRight>targetDutyCycleRight)
	{
		if(currentDutyCycleRight<dutyCycleIncrement)
			currentDutyCycleRight=0;
		else{
			currentDutyCycleRight-=dutyCycleIncrement;
		}
	}
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,currentDutyCycleLeft);
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,currentDutyCycleRight);


}
 *
 * */

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
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_IT(&hadc1);//joystick
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);//pwm
  HAL_TIM_Base_Start_IT(&htim4);//rpm
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);//motor driver
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  GPIO_PinState currentButtonState=HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0);
	  if(currentButtonState==GPIO_PIN_RESET&&lastButtonState==GPIO_PIN_SET)
	  {
		  joystickEnabled=!joystickEnabled;
	  }
	  lastButtonState=currentButtonState;
	  if(joystickEnabled){
		  HAL_ADC_Start(&hadc1);//pc0-x value
		  if(HAL_ADC_PollForConversion(&hadc1,1000)==HAL_OK){
			  adcValue[0]=HAL_ADC_GetValue(&hadc1);
		  }
		  HAL_ADC_Start(&hadc1);//pc1-yvalue
		  if(HAL_ADC_PollForConversion(&hadc1,1000)==HAL_OK){
			  adcValue[1]=HAL_ADC_GetValue(&hadc1);
		  }

	  }
	  else { // ADC timeout 또는 조이스틱 비활성화 시 모터 정지
	            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0);
	            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 0);
	            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 0);
	            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
	            continue; // 다음 루프로 이동
	        }

//gpio by raspberry pi

	     GPIO_PinState pin2_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10);//get B10
	  GPIO_PinState pin10_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);//get B4


		  if (adcValue[0] < 3000) {
		      // when forward

			  if (pin2_state == GPIO_PIN_SET) {
				  rxDataF = 1;
				  rxDataB = 0;
			  } else {
				  rxDataF = 0;
				  rxDataB = 0;
			  }
		  }
		  else if (adcValue[0] >3200) {
			  // when backward

			  if (pin10_state == GPIO_PIN_SET) {
				  rxDataB = 1;
				  rxDataF = 0;
			  } else {
				  rxDataB = 0;
				  rxDataF = 0;
			  }
		  } else {
			  // not backward or forward
			  rxDataF = 0;
			  rxDataB = 0;
		  }




//motor
		  if ((rxDataF==1 && adcValue[0] < 3000)||(rxDataB==1&&adcValue[0]>3200))
		  {
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0);
			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 0);
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 0);
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
		  }//block joystick when camera send 1

		  else{//when there is no obstacle

			  if (adcValue[1] < 2900) {
				  if (adcValue[0] < 3000) {
					  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 1);//left
					  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 0);
					  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 1);//right
					  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
					  //  targetDutyCycleLeft = clamp((3000 - adcValue[0]) * 999 / 3000, 0, 999);
					  //targetDutyCycleRight = clamp((3000 - adcValue[0]) * 999 / 3000, 0, 999);
					  //__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,999);
					  //__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,999);
				  }
				  if (adcValue[0] > 3200) {
					  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0);
					  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 1);
					  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 0);
					  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
					  //__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,999);
					  //__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,999);
					  //    targetDutyCycleLeft = clamp((adcValue[0] - 3300) * 999 / (4095 - 3300), 0, 999);
					  //  targetDutyCycleRight = clamp((adcValue[0] - 3300) * 999 / (4095 - 3300), 0, 999);
				  }
				  if(adcValue[0]<=3200&&adcValue[0]>=3000){
					  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 1);
					  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 0);
					  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 0);
					  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
//					  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,999);

					  //  targetDutyCycleLeft = 0;
					  //targetDutyCycleRight = clamp((2900 - adcValue[0]) * 999 / 2900, 0, 999);
				  }
			  }
			  if (adcValue[1] > 3200) {
				  if (adcValue[0] < 3000) {
					  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 1);
					  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 0);
					  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 1);
					  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
					//  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,999);
					  //__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,999);
					  //  targetDutyCycleLeft = clamp((3000 - adcValue[0]) * 999 / 3000, 0, 999);
					  //targetDutyCycleRight = clamp((3000 - adcValue[0]) * 999 / 3000, 0, 999);
				  }
				  if (adcValue[0] > 3200) {
					  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0);
					  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 1);
					  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 0);
					  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
					//  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,999);
					  //__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,999);
					  //    targetDutyCycleLeft = clamp((adcValue[0] - 3300) * 999 / (4095 - 3300), 0, 999);
					  //  targetDutyCycleRight = clamp((adcValue[0] - 3300) * 999 / (4095 - 3300), 0, 999);
				  }
				  if(adcValue[0]<=3200&&adcValue[0]>=3000){
					  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0);
					  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 0);
					  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 1);
					  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);

					 // __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,999);
					  //        targetDutyCycleLeft = clamp((3100 - adcValue[0]) * 999 / 3100, 0, 999);
					  //      targetDutyCycleRight = 0;
				  }
			  }
			  if(adcValue[1]<=3200&&adcValue[1]>=2900) {
				  //   targetDutyCycleLeft = 0;
				  // targetDutyCycleRight = 0;
				  if (adcValue[0] < 3000) {
					  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 1);
					  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 0);
					  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 1);
					  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
				//	  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,999);
					//  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,999);
					  //      targetDutyCycleLeft = clamp((3000 - adcValue[0]) * 999 / 3000, 0, 999);
					  //    targetDutyCycleRight = clamp((3000 - adcValue[0]) * 999 / 3000, 0, 999);
				  }
				  if (adcValue[0] > 3200) {
					  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0);
					  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 1);
					  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 0);
					  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
				//	  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,999);
					//  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,999);
					  //        targetDutyCycleLeft = clamp((adcValue[0] - 3300) * 999 / (4095 - 3300), 0, 999);
					  //      targetDutyCycleRight = clamp((adcValue[0] - 3300) * 999 / (4095 - 3300), 0, 999);
				  }
				  if(adcValue[0]<=3200&&adcValue[0]>=3000){
					  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0);
					  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 0);
					  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 0);
					  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);

					  //    targetDutyCycleLeft = 0;
					  //      targetDutyCycleRight = 0;
				  }
			  }



		  }

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = ENABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA6 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
