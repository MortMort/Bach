/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "MAF.h"
#include "PLL.h"
#include "our_library.h"

#include "CONSTANTS.h"
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
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

DAC_HandleTypeDef hdac;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */


// ADC & DAC DMA buffer
uint16_t readStart;

// Ring buffer
int16_t ringBuf[RING_BUF_LEN][RING_BUF_SIZE];
int16_t ringBufData[RING_BUF_SIZE];

uint16_t adcRingBuf1[ADC_RING_BUF_SIZE];
uint16_t adcRingBuf2[ADC_RING_BUF_SIZE];
uint16_t adcRingBuf3[ADC_RING_BUF_SIZE];


uint8_t	ringBufTrigger2 = 0;	// Triggers the ring buffer from the gpio input
uint8_t ringBufFlag	= 0;		// Goes high when the ring buffer is done filling the buffer after trigger
uint8_t ringBufPrintDone = 0;	// Goes high when printing of ring buffer is done, so usart isn't called again

// Execution time, time taking
int16_t timingArray[10];
int16_t timer_temp;


uint16_t var_dac;		// Dac output variable
float var_dac_f;			// Temporary float for dac output

uint16_t adcValue1, adcValue2, adcValue3;

float phaseA, phaseB, phaseC, angleDq;

// Declared in interrupt normally
float alpha1, beta1, Vq, Vd, VqMaf, VdMaf, alpha2, beta2, cosGrid, sinGrid;
float phaseError, anglePllComp, anglePll;

// T_st = 0.02 * 6:
//float ki = 2938.8889;
//float kp = 106.0408611;
//float kPhi = 0.010;

// T_st = 0.02 * 1:
float ki = 105800;
float kp = 1465.1;
float kPhi = 0.0095;



// File opening pointer
//FILE *fpt;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM10_Init(void);
static void MX_DAC_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

// USART DMA implementation: Interrupt definition
//void DMATransferComplete(DMA_HandleTypeDef *hdma);

uint8_t print_ring_buf(uint16_t bufferSize, int16_t circularBuffer[][RING_BUF_SIZE], uint16_t readStart);
uint8_t print_ring_buf_v2(uint16_t bufferSize, int16_t circularBuffer[][RING_BUF_SIZE], uint16_t readStart);
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


  //	huart2.Instance->CR3 |= USART_CR3_DMAT;
  //	adcReading0 = adcBuf[0];
  //	sprintf(msg_2, "Adc reading: %u\r\n", adcReading0);	// Update message for usart print
  //	HAL_DMA_Start_IT(&hdma_usart2_tx, (uint32_t)msg_2,
  //						(uint32_t)&huart2.Instance


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
  MX_ADC1_Init();
  MX_TIM10_Init();
  MX_DAC_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */


  // ADC DMA implementation//  HAL_ADC_Start_DMA(&hadc1, (int32_t*)adcValue16, sizeof(adcValue16)); // Start DMA
//  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcBuf, sizeof(adcBuf)/sizeof(uint16_t)); // Start DMA//  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcValue32, 1); // Start DMA (currently unused

  // USART DMA interrupt
//  HAL_DMA_RegisterCallback(&hdma_usart2_tx, HAL_DMA_XFER_CPLT_CB_ID, &DMATransferComplete);

  // DAC DMA
//  HAL_DAC_Start_DMA(&hdac, DAC1_CHANNEL_1, (uint32_t*)&adcBuf[0], sizeof(adcReading0), DAC_ALIGN_12B_R);//  HAL_DAC_Start_DMA(&hdac, DAC1_CHANNEL_1, (uint32_t*)testArray, sizeof(testArray)/sizeof(uint16_t), DAC_ALIGN_12B_R);

  // Timer interrupt start
  HAL_TIM_Base_Start_IT(&htim10);
  HAL_TIM_Base_Start(&htim1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	// External trigger in (from waveforms)
	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3)){
		ringBufTrigger2 = 1;
	}

	// Reset from button
	if (!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)) {
		ringBufFlag = 0;
		ringBufTrigger2 = 0;
		ringBufPrintDone = 0;
	}


	if (ringBufFlag && !ringBufPrintDone) {
		ringBufPrintDone = print_ring_buf_v2(RING_BUF_LEN, ringBuf, readStart);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

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
  htim1.Init.Prescaler = 18-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65536-1;
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
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 180-1;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 500-1;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

// USART DMA interrupt: Transfer of string to USART complete
//void DMATransferComplete(DMA_HandleTypeDef *hdma)
//{
//  huart2.Instance->CR3 &= ~USART_CR3_DMAT;
//  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//}

// ADC DMA interrupt: Half of buffer is full
//void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
//{
//  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
//}

// ADC DMA interrupt: The whole buffer is full
//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
//{
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
//}

//void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef hdac)
//{
//	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
//}

// Timer 10 (TIM10) interrupt:
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim10)
  {
    static uint16_t count; 			// Counter for sine output
//    static uint16_t var_dac;		// Dac output variable
//    static float var_dac_f;			// Temporary float for dac output

    // PLL variables start
    // Variables declared globally for easier debugging.
    //    static float angleDq, alpha1, beta1, Vq, Vd, alpha2, beta2, cosGrid, sinGrid;
    // PLL variables end

	// Set pin: Start timer
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);

    // ADCs
    timer_temp = __HAL_TIM_GET_COUNTER(&htim1);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);

    // ADC 1
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    adcValue1 = HAL_ADC_GetValue(&hadc1);

    // ADC 2
    HAL_ADC_Start(&hadc2);
    HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
    adcValue2 = HAL_ADC_GetValue(&hadc2);

	// ADC 3
    HAL_ADC_Start(&hadc3);
    HAL_ADC_PollForConversion(&hadc3, HAL_MAX_DELAY);

    adcValue3 = HAL_ADC_GetValue(&hadc3);
    timingArray[0] = __HAL_TIM_GET_COUNTER(&htim1) - timer_temp;




    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);


    timer_temp = __HAL_TIM_GET_COUNTER(&htim1);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);

    phaseA = (float)adcValue1/(0xFFF+1)*3.3f - 1.65f;
    phaseB = (float)adcValue2/(0xFFF+1)*3.3f - 1.65f;
    phaseC = (float)adcValue3/(0xFFF+1)*3.3f - 1.65f;

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
    timingArray[1] = __HAL_TIM_GET_COUNTER(&htim1) - timer_temp;

    // PLL Start
    //--------------------------------------------------------------------------------------------
    angleDq = angleDq + T_SAMPLE*F_RAD;
    if (angleDq > TWO_PI)
    {
    	angleDq = angleDq - TWO_PI;
    }


    	// Create simulation three phase
    //    phaseA = sinf(angleDq);
    //	phaseB = sinf(angleDq-RAD_120);
    //	phaseC = sinf(angleDq+RAD_120);

	// abc -> alpha beta
	timer_temp = __HAL_TIM_GET_COUNTER(&htim1);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);

    abc_to_alphabeta(phaseA, phaseB, phaseC, &alpha1, &beta1);

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
    timingArray[2] = __HAL_TIM_GET_COUNTER(&htim1) - timer_temp;

    // alpha beta -> DQ
    timer_temp = __HAL_TIM_GET_COUNTER(&htim1);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);

    alphabeta_to_dq(alpha1, beta1, angleDq, &Vd, &Vq);

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
    timingArray[3] = __HAL_TIM_GET_COUNTER(&htim1) - timer_temp;

    // MAF
    timer_temp = __HAL_TIM_GET_COUNTER(&htim1);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);

	maf(&Vd, &Vq, &VdMaf, &VqMaf);

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
	timingArray[4] = __HAL_TIM_GET_COUNTER(&htim1) - timer_temp;

	// DQ -> alpha beta
	timer_temp = __HAL_TIM_GET_COUNTER(&htim1);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);

    dq_to_alphabeta(VdMaf, VqMaf, angleDq, &alpha2, &beta2);

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
	timingArray[5] = __HAL_TIM_GET_COUNTER(&htim1) - timer_temp;


	// sinGrid & cosGrid
	timer_temp = __HAL_TIM_GET_COUNTER(&htim1);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);

    cos_sin_grid(alpha2, beta2, &cosGrid, &sinGrid);

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
    timingArray[6] = __HAL_TIM_GET_COUNTER(&htim1) - timer_temp;

    // Phase detector
    timer_temp = __HAL_TIM_GET_COUNTER(&htim1);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);

    phaseError = phase_detector(cosGrid, sinGrid, anglePllComp);

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
    timingArray[7] = __HAL_TIM_GET_COUNTER(&htim1) - timer_temp;

    // PI-regulator
    timer_temp = __HAL_TIM_GET_COUNTER(&htim1);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);

    pi_regulator(phaseError, F_RAD, ki, kp, kPhi, T_SAMPLE, &anglePll, &anglePllComp);

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
    timingArray[8] = __HAL_TIM_GET_COUNTER(&htim1) - timer_temp;


    //--------------------------------------------------------------------------------------------
    // PLL End
//    static float temp = 0.1;
//    temp = temp + 0.1f;
//    if (temp > 3.2f) {
//    	temp = 0.1;
//    }
    // DAC
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
    var_dac_f = (anglePll*0.5f + 0.05f) * 4096.0f/3.3f;	// +1 for offset for negative values, /3.3 for scaling
//    static float var;
//    var = temp;
//    var_dac_f = (var * 4096.0f/3.3f) +  var*(-0.0078) + 45.783;		// +1 for offset for negative values, /3.3 for scaling
//    var_dac_f = (var * 4096.0f/3.3f);		// +1 for offset for negative values, /3.3 for scaling
    var_dac = (uint16_t)var_dac_f; 			// Convert from float to uint16_t

    HAL_DAC_Start(&hdac, DAC_CHANNEL_1); 	// Start the DAC
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, var_dac); // Set dac to digital value
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);

    // Ring buffer
    /*
    ringBufData[0] 	= ((float)	phaseA 			* (float)RING_BUF_SCALING);
    ringBufData[1] 	= ((float)	phaseB			* (float)RING_BUF_SCALING);
    ringBufData[2] 	= ((float)	phaseC 			* (float)RING_BUF_SCALING);
    ringBufData[3] 	= ((float)	VdMaf 			* (float)RING_BUF_SCALING);
    ringBufData[4] 	= ((float)	VqMaf 			* (float)RING_BUF_SCALING);
    ringBufData[5] = ((float)	cosGrid			* (float)RING_BUF_SCALING);
    ringBufData[6] = ((float)	sinGrid 		* (float)RING_BUF_SCALING);
    ringBufData[7] = ((float)	phaseError 		* (float)RING_BUF_SCALING);
    ringBufData[8] = ((float)	anglePll 		* (float)RING_BUF_SCALING);
    ringBufData[9] = ((float)	angleDq			* (float)RING_BUF_SCALING);
	*/

    // Ring buffer
    ringBufData[0] 	= ((float)	phaseA 			* (float)RING_BUF_SCALING);
    ringBufData[1] 	= ((float)	phaseB			* (float)RING_BUF_SCALING);
    ringBufData[2] 		= ((float)	phaseC 			* (float)RING_BUF_SCALING);
    ringBufData[3] 	= ((float)	alpha1 			* (float)RING_BUF_SCALING);
    ringBufData[4] 	= ((float)	beta1 			* (float)RING_BUF_SCALING);
    ringBufData[5] 	= ((float)	Vd 				* (float)RING_BUF_SCALING);
    ringBufData[6] 	= ((float)	Vq				* (float)RING_BUF_SCALING);
    ringBufData[7] 	= ((float)	VdMaf 			* (float)RING_BUF_SCALING);
    ringBufData[8] 	= ((float)	VqMaf 			* (float)RING_BUF_SCALING);
    ringBufData[9] 	= ((float)	alpha2 			* (float)RING_BUF_SCALING);
    ringBufData[10] = ((float)	beta2			* (float)RING_BUF_SCALING);
    ringBufData[11] = ((float)	cosGrid			* (float)RING_BUF_SCALING);
    ringBufData[12] = ((float)	sinGrid 		* (float)RING_BUF_SCALING);
    ringBufData[13] = ((float)	phaseError 		* (float)RING_BUF_SCALING);
    ringBufData[14] = ((float)	anglePll 		* (float)RING_BUF_SCALING);
    ringBufData[15] = ((float)	anglePllComp	* (float)RING_BUF_SCALING);
    ringBufData[16] = ((float)	angleDq			* (float)RING_BUF_SCALING);

    // Timing:
//    ringBufData[17] = timingArray[0];
//    ringBufData[18] = timingArray[1];
//    ringBufData[19] = timingArray[2];
//    ringBufData[20] = timingArray[3];
//    ringBufData[21] = timingArray[4];
//    ringBufData[22] = timingArray[5];
//    ringBufData[23] = timingArray[6];
//    ringBufData[24] = timingArray[7];
//    ringBufData[25] = timingArray[8];

    ringBufFlag = circular_buffer(RING_BUF_LEN, ringBuf, ringBufData, ringBufTrigger2, RING_BUF_SPLIT, &readStart);



    // Count up interrupt count
	if (count < (RING_BUF_LEN*RING_BUF_SPLIT))
	{
	  count++;
	}
	else
	{
	  count = 0;
	}


	// Reset pin: Stop timer
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
  }
}


//  Function    :   print_ring_buf
//  Description :   prints the ring buffer values
//  Parameters  :   uint16_t bufferSize: pointer to an int to store the number
//                  uint16_t circularBuffer: Pointer to circular buffer array
//                  uint16_t readStart: starting index of the circular buffer
//  Returns     :	none
uint8_t print_ring_buf(uint16_t bufferSize, int16_t circularBuffer[][RING_BUF_SIZE], uint16_t readStart) {
    static uint16_t readIndex   =   0;
    static uint8_t init         =   0;

    static char msg[250];	// Initialize string to be written to USART

    // Initialize readIndex to readStart
    if (!init) {
        readIndex = readStart;
        init = 1;
    }

	sprintf(msg, "phaseA, phaseB, phaseC, alpha1, beta1, Vd, Vq, VdMaf, VqMaf, alpha2, beta2, cosGrid, sinGrid, phaseError, anglePll, anglePllComp, angleDq, t_adc, t_3p_sin, t_abc_ab, t_ab_dq, t_maf, t_dq_ab, t_sin_cos, t_phase_d, t_pi_regulator \r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);


    for (int i = 0; i < bufferSize; i++)
    {
        //printf("Buffervalue at index [%d] = %d\n", readIndex, circularBuffer[readIndex]);

//    	sprintf(msg, "%d, %d, %d, %d\r\n", circularBuffer[readIndex][0], circularBuffer[readIndex][1],
//									circularBuffer[readIndex][2], circularBuffer[readIndex][3]);	// Update message for usart print
    	sprintf(msg, "%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\r\n",
											circularBuffer[readIndex][0], 	circularBuffer[readIndex][1],
											circularBuffer[readIndex][2], 	circularBuffer[readIndex][3],
											circularBuffer[readIndex][4], 	circularBuffer[readIndex][5],
											circularBuffer[readIndex][6], 	circularBuffer[readIndex][7],
											circularBuffer[readIndex][8], 	circularBuffer[readIndex][9],
											circularBuffer[readIndex][10], 	circularBuffer[readIndex][11],
											circularBuffer[readIndex][12], 	circularBuffer[readIndex][13],
											circularBuffer[readIndex][14], 	circularBuffer[readIndex][15],
											circularBuffer[readIndex][16], circularBuffer[readIndex][17],
											circularBuffer[readIndex][18], circularBuffer[readIndex][19],
											circularBuffer[readIndex][20], circularBuffer[readIndex][21],
											circularBuffer[readIndex][22], circularBuffer[readIndex][23],
											circularBuffer[readIndex][24], circularBuffer[readIndex][25]);	// Update message for usart print

    	// sprintf(msg, "[%d] = %d\r\n", readIndex, circularBuffer[readIndex]);	// Update message for usart print


    	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    	/*
    	huart2.Instance->CR3 |= USART_CR3_DMAT;
    	HAL_DMA_Start_IT(&hdma_usart2_tx, (uint32_t)msg,
    							(uint32_t)&huart2.Instance->DR, strlen(msg));
		*/

        readIndex++;
        if (readIndex > bufferSize) {
            readIndex = 0;
        }
    }
    return 1;

}

//  Function    :   print_ring_buf
//  Description :   prints the ring buffer values
//  Parameters  :   uint16_t bufferSize: The amount of sampling points that the circularBuffer can contain
//                  uint16_t circularBuffer: Pointer to circular buffer array
//                  uint16_t readStart: starting index of the circular buffer
//  Returns     :	none
uint8_t print_ring_buf_v2(uint16_t bufferSize, int16_t circularBuffer[][RING_BUF_SIZE], uint16_t readStart) {
    static uint16_t readIndex   =   0;
    static uint8_t init         =   0;
    static uint16_t pos			=	0;		// Track position of array

    static char msg[250];	// Initialize string to be written to USART

    // Initialize readIndex to readStart
    if (!init) {
        readIndex = readStart;
        init = 1;
    }

//	sprintf(msg, "phaseA, phaseB, phaseC, alpha1, beta1, Vd, Vq, VdMaf, VqMaf, alpha2, beta2, cosGrid, sinGrid, phaseError, anglePll, anglePllComp, angleDq, t_adc, t_3p_sin, t_abc_ab, t_ab_dq, t_maf, t_dq_ab, t_sin_cos, t_phase_d, t_pi_regulator \r\n");
    sprintf(msg, "phaseA, phaseB, phaseC, alpha1, beta1, Vd, Vq, VdMaf, VqMaf, alpha2, beta2, cosGrid, sinGrid, phaseError, anglePll, anglePllComp, angleDq\r\n");
//    sprintf(msg, "phaseA, phaseB, phaseC, VdMaf, VqMaf, cosGrid, sinGrid, phaseError, anglePll, angleDq\r\n");

	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);


    for (int i = 0; i < bufferSize; i++)
    {
        //printf("Buffervalue at index [%d] = %d\n", readIndex, circularBuffer[readIndex]);

//    	sprintf(msg, "%d, %d, %d, %d\r\n", circularBuffer[readIndex][0], circularBuffer[readIndex][1],
//									circularBuffer[readIndex][2], circularBuffer[readIndex][3]);	// Update message for usart print
    	for (int n = 0; n < (RING_BUF_SIZE); n++) {

    		if (n < RING_BUF_SIZE-1) {
    			pos += sprintf(&msg[pos], "%d, ", circularBuffer[readIndex][n]);
    		}
    		else {
    			pos += sprintf(&msg[pos], "%d\r\n", circularBuffer[readIndex][n]);
    		}
    	}

    	/*
    	sprintf(msg, "%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\r\n",
											circularBuffer[readIndex][0], 	circularBuffer[readIndex][1],
											circularBuffer[readIndex][2], 	circularBuffer[readIndex][3],
											circularBuffer[readIndex][4], 	circularBuffer[readIndex][5],
											circularBuffer[readIndex][6], 	circularBuffer[readIndex][7],
											circularBuffer[readIndex][8], 	circularBuffer[readIndex][9],
											circularBuffer[readIndex][10], 	circularBuffer[readIndex][11],
											circularBuffer[readIndex][12], 	circularBuffer[readIndex][13],
											circularBuffer[readIndex][14], 	circularBuffer[readIndex][15],
											circularBuffer[readIndex][16], circularBuffer[readIndex][17],
											circularBuffer[readIndex][18], circularBuffer[readIndex][19],
											circularBuffer[readIndex][20], circularBuffer[readIndex][21],
											circularBuffer[readIndex][22], circularBuffer[readIndex][23],
											circularBuffer[readIndex][24], circularBuffer[readIndex][25]);	// Update message for usart print

    	// sprintf(msg, "[%d] = %d\r\n", readIndex, circularBuffer[readIndex]);	// Update message for usart print
		*/

    	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    	/*
    	huart2.Instance->CR3 |= USART_CR3_DMAT;
    	HAL_DMA_Start_IT(&hdma_usart2_tx, (uint32_t)msg,
    							(uint32_t)&huart2.Instance->DR, strlen(msg));
		*/
    	pos = 0;	// Reset pos
        readIndex++;
        if (readIndex > (bufferSize-1)) {
            readIndex = 0;
        }
    }
    return 1;

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
