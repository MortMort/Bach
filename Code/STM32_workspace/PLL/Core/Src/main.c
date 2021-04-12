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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


#define adcBuf_LEN 		10									// Size of ADC buffer (unused)
#define RING_BUF_LEN 	2000								// Size of ring buffer
#define PI 				(3.1415926535897)
#define TWO_PI 			(2.0*PI)
#define F_RAD 			(50.0f*3.1415926535897f)
#define F_SAMPLE 		1000
#define T_SAMPLE 		0.001f
#define T_SINE 			1.0f								// [s] sine time
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac;

TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */


// ADC & DAC DMA buffer
uint16_t adcBuf[adcBuf_LEN];
uint16_t readStart;

uint16_t ringBuf[RING_BUF_LEN];
uint16_t adcValue1;
uint16_t adcValue2;

float phaseA, phaseB, phaseC, angleDq;

// Declared in interrupt normally
float alpha1, beta1, Vq, Vd, alpha2, beta2, cosGrid, sinGrid;
float phaseError, anglePllComp, anglePll;

float ki = 2938.8889;
float kp = 106.0408611;
float kPhi = 0.010;

// Simulation sine arrays
float sine1[(int)(F_SAMPLE*T_SINE)];
float sine2[(int)(F_SAMPLE*T_SINE)];

uint8_t ringBufTrigger = 0;
uint8_t ringBufFlag	= 0;
uint8_t ringBufPrintDone = 0;

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
/* USER CODE BEGIN PFP */

// USART DMA implementation: Interrupt definition
//void DMATransferComplete(DMA_HandleTypeDef *hdma);

void sine_phaseA();
void sine_phaseB();
uint8_t printRingBuf(uint16_t bufferSize, uint16_t *circularBuffer, uint16_t readStart);
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
  // Sine creator test:
  sine_phaseA();
  sine_phaseB();


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
  /* USER CODE BEGIN 2 */


  // ADC DMA implementation//  HAL_ADC_Start_DMA(&hadc1, (int32_t*)adcValue16, sizeof(adcValue16)); // Start DMA
//  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcBuf, sizeof(adcBuf)/sizeof(uint16_t)); // Start DMA//  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcValue32, 1); // Start DMA (currently unused

  // USART DMA interrupt
//  HAL_DMA_RegisterCallback(&hdma_usart2_tx, HAL_DMA_XFER_CPLT_CB_ID, &DMATransferComplete);

  // DAC DMA
//  HAL_DAC_Start_DMA(&hdac, DAC1_CHANNEL_1, (uint32_t*)&adcBuf[0], sizeof(adcReading0), DAC_ALIGN_12B_R);//  HAL_DAC_Start_DMA(&hdac, DAC1_CHANNEL_1, (uint32_t*)testArray, sizeof(testArray)/sizeof(uint16_t), DAC_ALIGN_12B_R);

  // Timer interrupt start
  HAL_TIM_Base_Start_IT(&htim10);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	if (ringBufFlag && !ringBufPrintDone) {
		ringBufPrintDone = printRingBuf(RING_BUF_LEN, ringBuf, readStart);
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
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
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
  hadc2.Init.ContinuousConvMode = ENABLE;
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
  htim10.Init.Period = 1000-1;
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
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_8, GPIO_PIN_RESET);

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
    static uint16_t var_dac;		// Dac output variable
    static float dac_temp;			// Temporary float for dac output

    // PLL variables start
    // Variables declared globally for easier debugging.
    //    static float angleDq, alpha1, beta1, Vq, Vd, alpha2, beta2, cosGrid, sinGrid;
    // PLL variables end

	// Set pin: Start timer
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);

    // ADC 1
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    adcValue1 = HAL_ADC_GetValue(&hadc1);

    // ADC 2
    HAL_ADC_Start(&hadc2);
    HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
    adcValue2 = HAL_ADC_GetValue(&hadc2);


//    phaseA = (float)adcValue1/(0xFFF+1);
//    phaseB = (float)adcValue2/(0xFFF+1);
//    phaseC = two_to_three_phase(&phaseA, &phaseB);

    phaseA = sine1[count];
    phaseB = sine2[count];
    phaseC = two_to_three_phase(&phaseA, &phaseB);


	// USART DMA implementation
//	huart2.Instance->CR3 |= USART_CR3_DMAT;
//	adcReading0 = adcBuf[0];
//	sprintf(msg_2, "Adc reading: %u\r\n", adcReading0);	// Update message for usart print
//	HAL_DMA_Start_IT(&hdma_usart2_tx, (uint32_t)msg_2,
//						(uint32_t)&huart2.Instance->DR, strlen(msg_2));

    // Ring buffer trigger test
    if (count == 450) {
    	ringBufTrigger = 1;
    }

    // PLL StartT_SAMPLE
    //--------------------------------------------------------------------------------------------
    angleDq = angleDq + T_SAMPLE*F_RAD;
    if (angleDq > TWO_PI)
    {
    	angleDq = angleDq - TWO_PI;
    }


    alpha1 = abc_to_alpha(phaseA, phaseB, phaseC);
    beta1 = abc_to_beta(phaseA, phaseB, phaseC);

    Vd = alphabeta_to_d(alpha1, beta1, angleDq);
    Vq = alphabeta_to_q(alpha1, beta1, angleDq);

    alpha2 = dq_to_alpha(Vd, Vq, angleDq);
    beta2 = dq_to_beta(Vd, Vq, angleDq);

    cosGrid = cos_grid(alpha2, beta2);
    sinGrid = sin_grid(alpha2, beta2);

    phaseError = phase_detector(cosGrid, sinGrid, anglePllComp);

    anglePll = pi_regulator(phaseError, F_RAD, ki, kp, kPhi, T_SAMPLE);
    anglePllComp = pi_regulator_comp(phaseError, F_RAD, ki, kp, kPhi, T_SAMPLE);
    //--------------------------------------------------------------------------------------------
    // PLL End

    // DAC
    dac_temp = (phaseA + 1) * 4096.0/4.0; // +1 for offset, /4 for scaling
    var_dac = (uint16_t)dac_temp; //
    HAL_DAC_Start(&hdac, DAC_CHANNEL_1); 	// Start the DAC
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, var_dac); // Set dac to digital value

    // Ring buffer
    ringBufFlag = circular_buffer(RING_BUF_LEN, ringBuf, &count, ringBufTrigger, 0.25, &readStart);




    // Count up to size of sine array
	if (count < (F_SAMPLE*T_SINE))
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

void sine_phaseA()
{
	// Create 50 Hz sine with 0 phase shift
	uint32_t var;
	for (var = 0; var < (F_SAMPLE*T_SINE); ++var)
	{
		sine1[var] = sinf(50.0 * (float)var * 1/F_SAMPLE * TWO_PI);
	}
}

void sine_phaseB()
{
	// Create 50 Hz sine with 120 deg phase shift
	uint32_t var;
	for (var = 0; var < (F_SAMPLE*T_SINE); ++var)
	{
		sine2[var] = sinf(50.0 * (float)var * 1/F_SAMPLE * TWO_PI - 120 * PI/180);
	}
}



//  Function    :   printRingBuf
//  Description :   prints the ring buffer values
//  Parameters  :   uint16_t bufferSize: pointer to an int to store the number
//                  uint16_t *circularBuffer: Pointer to circular buffer array
//                  uint16_t readStart: starting index of the circular buffer
//  Returns     :	none
uint8_t printRingBuf(uint16_t bufferSize, uint16_t *circularBuffer, uint16_t readStart) {
    static uint16_t readIndex   =   0;
    static uint8_t init         =   0;

    static char msg[20];	// Initialize string to be written to USART

    // Initialize readIndex to readStart
    if (!init) {
        readIndex = readStart;
        init = 1;
    }

    for (int i = 0; i < bufferSize; i++)
    {
        //printf("Buffervalue at index [%d] = %d\n", readIndex, circularBuffer[readIndex]);

    	sprintf(msg, "[%d] = %d\r\n", readIndex, circularBuffer[readIndex]);	// Update message for usart print


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
