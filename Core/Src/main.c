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
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MAX_FREQ_H 20000
#define MAX_FREQ_V 500
#define MAX_FREQ_STEP_H 2000
#define SMALL_FREQ_STEP_H 600
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;
DMA_HandleTypeDef hdma_tim16_ch1_up;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t rcv_buf[64];
GPIO_PinState curDirState_v = GPIO_PIN_RESET;
GPIO_PinState curDirState_h = GPIO_PIN_RESET;
bool dirChanged_h = false;
// store last frequency set for horizontal and vertical rotation:
// prevents motor from stalling due to frequency changing too fast
uint32_t prev_freq_h = 0;
uint32_t prev_freq_v = 0;
int prev_leftJoystickX = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM16_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM15_Init(void);
/* USER CODE BEGIN PFP */
void SetPWMFrequency(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t frequency);
void pullTrigger(TIM_HandleTypeDef *htim, uint32_t channel);
void parse_cmd(char *buf);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  uint32_t raw;
  char msg[100];
  uint16_t count = 0;
  uint32_t freq = 0;

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
  MX_USART2_UART_Init();
  MX_TIM16_Init();
  MX_ADC1_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */
//  HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(USART3_IRQn);

  // pwm for vertical rotation stepper
  TIM16->CCR1 = 100;
  HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
  // pwm for trigger pull stepper
  TIM3->CCR1 = 50;
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  TIM15->CCR1 = 100;
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);

//  HAL_GPIO_TogglePin (GPIOC, GPIO_PIN_6);

//  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
//  HAL_GPIO_WritePin(GPIOA, 7, GPIO_PIN_RESET);
//  HAL_Delay(5000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	raw = HAL_ADC_GetValue(&hadc1);

//	HAL_GPIO_WritePin(GPIOB, 0, GPIO_PIN_RESET);


	sprintf(msg, "%hu hello\r\n", raw);
//	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	memset(rcv_buf, 0, sizeof(rcv_buf));

	HAL_UART_Receive(&huart3, rcv_buf, 11, 100);
	rcv_buf[11] = '\0';
	parse_cmd((char *)rcv_buf);


//	HAL_UART_Receive_IT(&huart3, rcv_buf, sizeof(rcv_buf));
	sprintf(msg, "%s \r\n", (char *)rcv_buf);
//	HAL_UART_Transmit(&huart2, msg, strlen((char *)msg), HAL_MAX_DELAY);

//	if (count>2000) {
//		SetPWMFrequency(&htim16, TIM_CHANNEL_1, 17000);
//		msg = "17000\r\n";
//		HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
//	}


//	if (count>5000 && count<=8000) {
//		SetPWMFrequency(&htim16, TIM_CHANNEL_1, 3000);
//		sprintf(msg, "12000\r\n");
//		HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
//	}
//	else if (count>8000) {
//		SetPWMFrequency(&htim16, TIM_CHANNEL_1, 2000);
//		sprintf(msg, "7000\r\n");
//		HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
//	}


//	else if (count>11000) {
//		SetPWMFrequency(&htim16, TIM_CHANNEL_1, 700);
//		msg = "1700";
//		HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
//	}

//	freq = ((uint32_t)raw*3000)/4095;

//	sprintf(msg, "%u\r\n", freq);
//	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
//
//	SetPWMFrequency(&htim16, TIM_CHANNEL_1, freq);
	count++;
//	HAL_Delay(10);
	HAL_Delay(1);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.Timing = 0x10909CEC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  htim3.Init.Prescaler = 800-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 80-1;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 200-1;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */
  HAL_TIM_MspPostInit(&htim15);

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 80-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 200-1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */
  HAL_TIM_MspPostInit(&htim16);

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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA7 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//    if (huart->Instance == USART3) {
//        // Handle received data
//        HAL_UART_Transmit(&huart2, rcv_buf, sizeof(rcv_buf), HAL_MAX_DELAY);
//
//        // Re-enable UART receive interrupt
//        HAL_UART_Receive_IT(&huart3, rcv_buf, sizeof(rcv_buf));
//    }
//}



void SetPWMFrequency(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t frequency)
{
    uint32_t timer_clock = HAL_RCC_GetPCLK1Freq(); // Get the timer clock frequency
//    uint32_t prescaler = 0;
    uint32_t period = 0;

    if (htim == &htim15) {
    	if (dirChanged_h) {
        	if (frequency + prev_freq_h >= MAX_FREQ_STEP_H - 500) { // used -400
        		if (prev_freq_h < SMALL_FREQ_STEP_H) {
        			if (curDirState_h == GPIO_PIN_RESET) {
        				curDirState_h = GPIO_PIN_SET;
        			} else {
        				curDirState_h = GPIO_PIN_RESET;
        			}
        			HAL_GPIO_TogglePin (GPIOB, GPIO_PIN_0);
        			dirChanged_h = false;
        			frequency = SMALL_FREQ_STEP_H - prev_freq_h;
        		} else {
        			frequency = prev_freq_h - SMALL_FREQ_STEP_H;
        		}

        	} else {
    			if (curDirState_h == GPIO_PIN_RESET) {
    				curDirState_h = GPIO_PIN_SET;
    			} else {
    				curDirState_h = GPIO_PIN_RESET;
    			}
    			HAL_GPIO_TogglePin (GPIOB, GPIO_PIN_0);
    			dirChanged_h = false;


        	}
    	} else if (frequency >= prev_freq_h + MAX_FREQ_STEP_H) {
//        	frequency = (frequency + prev_freq_h)/2;
        	// 2500, 500 works well for 20000 Hz
    			// also 1000, 700
        	// 2000, 100-150 works well for 25000 Hz
        	frequency = prev_freq_h + SMALL_FREQ_STEP_H;
        }
        else if (prev_freq_h >= MAX_FREQ_STEP_H && frequency <= prev_freq_h - MAX_FREQ_STEP_H) {
        	frequency = prev_freq_h - SMALL_FREQ_STEP_H;
        }
        prev_freq_h = frequency;
    }


    // Calculate prescaler and period
//    for (prescaler = 0; prescaler <= 0xFFFF; prescaler++) {
//        period = (timer_clock / (frequency * (prescaler + 1))) - 1;
//        if (period <= 0xFFFF) {
//            break;
//        }
//    }
    if (frequency==0) HAL_TIM_PWM_Stop(htim, channel);
    else {
    	period = (timer_clock) / (frequency * (79 + 1)) - 1;
        htim->Instance->ARR = period;
//        TIM16->CCR1 = period/2;
        htim->Instance->CCR1 = period/2;

        // Update registers
        HAL_TIM_PWM_Start(htim, channel);
    }


    // Update the timer settings
//    htim->Instance->PSC = prescaler;

    char msg_buf[100];
//    sprintf(msg_buf, "current frequency: %lu\r\n", frequency);
//    HAL_UART_Transmit(&huart2, (uint8_t*)msg_buf, strlen(msg_buf), HAL_MAX_DELAY);
    // NOTE: NOT SURE IF THIS DELAY SHOULD BE IN HERE YET
//    HAL_Delay(10);
//    HAL_Delay(150);

}

void pullTrigger(TIM_HandleTypeDef *htim, uint32_t channel)
{

	TIM3->CCR1 = 250;
	HAL_TIM_PWM_Start(htim, channel);
	HAL_Delay(1000);
	TIM3->CCR1 = 50;
	HAL_TIM_PWM_Start(htim, channel);

    char msg_buf[100];
    sprintf(msg_buf, "Trigger pulled");
    HAL_UART_Transmit(&huart2, (uint8_t*)msg_buf, strlen(msg_buf), HAL_MAX_DELAY);
}


void parse_cmd(char * in_buf) {
	/*
	 Debugging notes:
	 For some reason, it's getting a leftJoyStickY value of 0 irreqularly. Probably
	 has something to do with the way the buffer is being tokenized or the buffer
	 that's being received. I debug the value of the buffer next to verify this, then
	 fix the problem.

	 Might need to zero buffer out

	 Kind of difficult to debug with the debugger because it messes with the data
	 transmission timing.

	 Direction shift is also not working for some reason, so I need to figure that out.
	 */
	if (strlen(in_buf)<5) return;
	char buf[12];
	strcpy(buf, in_buf);
//	if (buf[10]!=47) {
//		for(int i=0; i<5; i++) {
//			if (buf[i]==47) return;
//		}
//	} else {
//		for (int i=0; i<5; i++) {
//			if (buf[i]==47) return;
//		}
//	}
	char msg_buf[100];
	sprintf(msg_buf, "buf: %s\r\n", buf);
	HAL_UART_Transmit(&huart2, (uint8_t*)msg_buf, strlen(msg_buf), HAL_MAX_DELAY);



	char *token = strtok(buf, ",");
	int leftJoystickX = atoi(token);
	if (strlen(token)>1 && leftJoystickX==0) return;
//	if (strlen(token)>1 && leftJoystickX==0) return;
//	sprintf(msg_buf, "token: %s\r\n", token);
//	HAL_UART_Transmit(&huart2, (uint8_t*)msg_buf, strlen(msg_buf), HAL_MAX_DELAY);
//	HAL_UART_Transmit(&huart2, (uint8_t*)token, strlen(token), HAL_MAX_DELAY);
	token = strtok(NULL, ",");
//	HAL_UART_Transmit(&huart2, (uint8_t*)token, strlen(token), HAL_MAX_DELAY);
	int leftJoystickY = atoi(token);
	if (strlen(token)>1 && leftJoystickY==0) return;
//	sprintf(msg_buf, "left joystick y: %d\r\n", leftJoystickY);
//	HAL_UART_Transmit(&huart2, (uint8_t*)msg_buf, strlen(msg_buf), HAL_MAX_DELAY);
	token = strtok(NULL, "/");
	int triggerPulled = atoi(token);
	if (token==NULL) return;
	sprintf(msg_buf, "token: %s\r\n", token);
	HAL_UART_Transmit(&huart2, (uint8_t*)msg_buf, strlen(msg_buf), HAL_MAX_DELAY);
//	HAL_UART_Transmit(&huart2, (uint8_t*)token, strlen(token), HAL_MAX_DELAY);

//	HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
//	GPIO_PinState pinState = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);

	if (curDirState_v == GPIO_PIN_SET && leftJoystickY<0) {
	    // Pin is high
		HAL_GPIO_TogglePin (GPIOC, GPIO_PIN_6);
		curDirState_v = GPIO_PIN_RESET;
	} else if (curDirState_v == GPIO_PIN_RESET && leftJoystickY>0){
	    // Pin is low
		HAL_GPIO_TogglePin (GPIOC, GPIO_PIN_6);
		curDirState_v = GPIO_PIN_SET;
	}

	if (curDirState_h == GPIO_PIN_SET && leftJoystickX<0) {
		// Pin is high
//		HAL_GPIO_TogglePin (GPIOB, GPIO_PIN_0);
//		curDirState_h = GPIO_PIN_RESET;
		dirChanged_h = true;
	} else if (curDirState_h == GPIO_PIN_RESET && leftJoystickX>0){
		// Pin is low
//		HAL_GPIO_TogglePin (GPIOB, GPIO_PIN_0);
//		curDirState_h = GPIO_PIN_SET;
		dirChanged_h = true;
	}

//	if (leftJoystickX >= prev_leftJoystickX + 15) {
//		leftJoystickX = prev_leftJoystickX + 5;
//		prev_leftJoystickX = leftJoystickX;
//	} else if (leftJoystickX <= prev_leftJoystickX - 15) {
//		leftJoystickX = prev_leftJoystickX - 5;
//		prev_leftJoystickX = leftJoystickX;
//	}



	if (leftJoystickY<0) {
		leftJoystickY = -leftJoystickY;
	}
	if (leftJoystickX<0) {
		leftJoystickX = -leftJoystickX;
	}

	uint32_t frequency_v = ((uint32_t)(leftJoystickY)*MAX_FREQ_V)/128; // note: max is 3000
	uint32_t frequency_h = ((uint32_t) leftJoystickX*MAX_FREQ_H)/128;
	if (triggerPulled==1) pullTrigger(&htim3, TIM_CHANNEL_1);
//	if (frequency==0) frequency = 1;
	sprintf(msg_buf, "freq: %lu\r\n", frequency_v);
//	HAL_UART_Transmit(&huart2, (uint8_t*)msg_buf, strlen(msg_buf), HAL_MAX_DELAY);
	SetPWMFrequency(&htim16, TIM_CHANNEL_1, frequency_v);
	SetPWMFrequency(&htim15, TIM_CHANNEL_1, frequency_h);


//	int cmds_idx = 0;
//	int last_buf_idx = 0;
//	char *cmds[10];
//	for (int i=0; i<strlen(buf); i++) {
//		if (buf[i]=='\0') break;
//		if (buf[i] == 32) {
//			strncpy(cmds[cmds_idx], buf+last_buf_idx, i-last_buf_idx);
//			cmds[cmds_idx][i-last_buf_idx] = '\0';
//			cmds_idx++;
//			last_buf_idx = i;
//		}
//	}
//	char msg_buf[100];
//	sprintf(msg_buf, "second cmd: %s", cmds[1]);
//	HAL_UART_Transmit(&huart2, (uint8_t*)msg_buf, 20, HAL_MAX_DELAY);
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
