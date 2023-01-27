/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "MAX31865_lib.h"
#include "MAX31865_lib2.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
bool task2=0;
int i=0;
int a[1]={3};
int adc_val1=0;
uint8_t str[10]="";
int adc_val2=0;
uint8_t str2[10]="";
int adc_val3=0;
uint8_t str3[10]="";
int adc_val4=0;
uint8_t str4[10]="";

bool durum=0;
uint8_t Sensor1;
uint8_t Sensor2;
uint8_t Sensor3;
uint8_t Sensor4;
uint8_t Sensor5;
uint8_t Sensor6;
uint8_t Sensor7;

uint16_t result3=0;

char temperature[10];
char temperatureHopper[10];
float PT100_Temperature = 0.0f;
float PT100_TemperatureHopper = 0.0f;
float temp=0;
float temp1=0;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
osThreadId Task2Handle;
osThreadId Task3Handle;
osThreadId Task4Handle;
osThreadId Task5Handle;
osThreadId Task6Handle;
osThreadId Task7Handle;
osThreadId Task8Handle;
osThreadId Task9Handle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI3_Init(void);
static void MX_ADC1_Init(void);
void StartDefaultTask(void const * argument);
void Task2_Init(void const * argument);
void Task3_Init(void const * argument);
void Task4_Init(void const * argument);
void Task5_Init(void const * argument);
void Task6_Init(void const * argument);
void Task7_Init(void const * argument);
void Task8_Init(void const * argument);
void Task9_Init(void const * argument);

/* USER CODE BEGIN PFP */

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
	MX_SPI1_Init();
	MX_SPI3_Init();
	MX_ADC1_Init();
	/* USER CODE BEGIN 2 */
	MAX31865_Init(3); // num of wires
	MAX31865_2_Init(3); // num of wires

	/* USER CODE END 2 */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* definition and creation of defaultTask */
	osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
	defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

	/* definition and creation of Task2 */
	osThreadDef(Task2, Task2_Init, osPriorityNormal, 0, 128);
	Task2Handle = osThreadCreate(osThread(Task2), NULL);

	/* definition and creation of Task3 */
	osThreadDef(Task3, Task3_Init, osPriorityNormal, 0, 128);
	Task3Handle = osThreadCreate(osThread(Task3), NULL);

	/* definition and creation of Task4 */
	osThreadDef(Task4, Task4_Init, osPriorityNormal, 0, 128);
	Task4Handle = osThreadCreate(osThread(Task4), NULL);

	/* definition and creation of Task5 */
	osThreadDef(Task5, Task5_Init, osPriorityNormal, 0, 128);
	Task5Handle = osThreadCreate(osThread(Task5), NULL);

	/* definition and creation of Task6 */
	osThreadDef(Task6, Task6_Init, osPriorityNormal, 0, 128);
	Task6Handle = osThreadCreate(osThread(Task6), NULL);

	/* definition and creation of Task7 */
	osThreadDef(Task7, Task7_Init, osPriorityNormal, 0, 128);
	Task7Handle = osThreadCreate(osThread(Task7), NULL);

	/* definition and creation of Task8 */
	osThreadDef(Task8, Task8_Init, osPriorityNormal, 0, 128);
	Task8Handle = osThreadCreate(osThread(Task8), NULL);

	/* definition and creation of Task9 */
	osThreadDef(Task9, Task9_Init, osPriorityIdle, 0, 128);
	Task9Handle = osThreadCreate(osThread(Task9), NULL);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
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
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL_NONE;
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
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure the Systick interrupt time
	 */
	__HAL_RCC_PLLI2S_ENABLE();
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
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_10;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void)
{

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief SPI3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI3_Init(void)
{

	/* USER CODE BEGIN SPI3_Init 0 */

	/* USER CODE END SPI3_Init 0 */

	/* USER CODE BEGIN SPI3_Init 1 */

	/* USER CODE END SPI3_Init 1 */
	/* SPI3 parameter configuration*/
	hspi3.Instance = SPI3;
	hspi3.Init.Mode = SPI_MODE_MASTER;
	hspi3.Init.Direction = SPI_DIRECTION_2LINES;
	hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
	hspi3.Init.NSS = SPI_NSS_SOFT;
	hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
	hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi3.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi3) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN SPI3_Init 2 */

	/* USER CODE END SPI3_Init 2 */

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
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, SPI1_CS_Pin|m1_Pin|m2_Pin|m3_Pin
			|m4_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : Sensor1_Pin Sensor7_Pin Sensor6_Pin Sensor5_Pin */
	GPIO_InitStruct.Pin = Sensor1_Pin|Sensor7_Pin|Sensor6_Pin|Sensor5_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : Sensor4_Pin Sensor3_Pin Sensor2_Pin */
	GPIO_InitStruct.Pin = Sensor4_Pin|Sensor3_Pin|Sensor2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : SPI1_CS_Pin m1_Pin m2_Pin m3_Pin
                           m4_Pin */
	GPIO_InitStruct.Pin = SPI1_CS_Pin|m1_Pin|m2_Pin|m3_Pin
			|m4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : SPI3_CS1_Pin */
	GPIO_InitStruct.Pin = SPI3_CS1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(SPI3_CS1_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	for(;;)
	{
		i++;
		HAL_Delay(1000);
		if(i==3){
			task2=1;
		}
		osDelay(50);
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Task2_Init */
/**
 * @brief Function implementing the Task2 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Task2_Init */
void Task2_Init(void const * argument)
{
	/* USER CODE BEGIN Task2_Init */
	/* Infinite loop */
	for(;;)
	{
		if(task2==1){
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
			HAL_Delay(1000);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
			HAL_Delay(1000);
		}
		osDelay(25);
	}
	/* USER CODE END Task2_Init */
}

/* USER CODE BEGIN Header_Task3_Init */
/**
 * @brief Function implementing the Task3 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Task3_Init */
void Task3_Init(void const * argument)
{
	/* USER CODE BEGIN Task3_Init */
	/* Infinite loop */
	for(;;)
	{
		ADC_ChannelConfTypeDef sConfig = {3};

		sConfig.Channel = ADC_CHANNEL_10;
		sConfig.Rank = ADC_REGULAR_RANK_1;
		sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1,100);
		adc_val1=(float)HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);
		sprintf(str,"%d",adc_val1);
		HAL_UART_Transmit(&huart2,str,sizeof(str),500);
		//		  ADC_ChannelConfTypeDef sConfig = {1};

		sConfig.Channel = ADC_CHANNEL_11;
		sConfig.Rank = ADC_REGULAR_RANK_1;
		sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1,100);
		adc_val2=(float)HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);
		sprintf(str2,"%d",adc_val2);
		HAL_UART_Transmit(&huart2,str2,sizeof(str2),500);

		sConfig.Channel = ADC_CHANNEL_12;
		sConfig.Rank = ADC_REGULAR_RANK_1;
		sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1,100);
		adc_val3=(float)HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);
		sprintf(str3,"%d",adc_val2);
		HAL_UART_Transmit(&huart2,str3,sizeof(str3),500);

		sConfig.Channel = ADC_CHANNEL_13;
		sConfig.Rank = ADC_REGULAR_RANK_1;
		sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1,100);
		adc_val4=(float)HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);
		sprintf(str4,"%d",adc_val4);
		HAL_UART_Transmit(&huart2,str4,sizeof(str4),500);

		osDelay(200);
	}
	/* USER CODE END Task3_Init */
}

/* USER CODE BEGIN Header_Task4_Init */
/**
 * @brief Function implementing the Task4 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Task4_Init */
void Task4_Init(void const * argument)
{
	/* USER CODE BEGIN Task4_Init */
	/* Infinite loop */
	for(;;)
	{
		Sensor1 = HAL_GPIO_ReadPin(GPIOB,Sensor1_Pin);
		Sensor2 = HAL_GPIO_ReadPin(GPIOC,Sensor2_Pin);
		Sensor3 = HAL_GPIO_ReadPin(GPIOC,Sensor3_Pin);
		Sensor4 = HAL_GPIO_ReadPin(GPIOC,Sensor4_Pin);
		Sensor5 = HAL_GPIO_ReadPin(GPIOB,Sensor5_Pin);
		Sensor6 = HAL_GPIO_ReadPin(GPIOB,Sensor6_Pin);
		Sensor7 = HAL_GPIO_ReadPin(GPIOB,Sensor7_Pin);
		osDelay(100);
	}
	/* USER CODE END Task4_Init */
}

/* USER CODE BEGIN Header_Task5_Init */
/**
 * @brief Function implementing the Task5 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Task5_Init */
void Task5_Init(void const * argument)
{
	/* USER CODE BEGIN Task5_Init */
	/* Infinite loop */
	for(;;)
	{
		osDelay(1);
	}
	/* USER CODE END Task5_Init */
}

/* USER CODE BEGIN Header_Task6_Init */
/**
 * @brief Function implementing the Task6 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Task6_Init */
void Task6_Init(void const * argument)
{
	/* USER CODE BEGIN Task6_Init */
	/* Infinite loop */
	for(;;)
	{
		PT100_TemperatureHopper = MAX31865_Get_Temperature2();
		if (PT100_TemperatureHopper >= 0) {
			PT100_TemperatureHopper = PT100_TemperatureHopper + 0.05;
			sprintf(temperatureHopper, "+%d.%d", (uint16_t) (PT100_TemperatureHopper),((uint16_t) (PT100_TemperatureHopper * 100)- ((uint16_t) PT100_TemperatureHopper) * 100) / 10);
			temp1 = (float)atof(temperatureHopper);
		}
		else {
			PT100_TemperatureHopper = -PT100_TemperatureHopper + 0.05;
			sprintf(temperatureHopper, "-%d.%d", (uint16_t) (PT100_TemperatureHopper),((uint16_t) (PT100_TemperatureHopper * 100)- ((uint16_t) PT100_TemperatureHopper) * 100) / 10);
			temp1 = (float)atof(temperatureHopper);
		}
		osDelay(300);
	}
	/* USER CODE END Task6_Init */
}

/* USER CODE BEGIN Header_Task7_Init */
/**
 * @brief Function implementing the Task7 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Task7_Init */
void Task7_Init(void const * argument)
{
	/* USER CODE BEGIN Task7_Init */
	/* Infinite loop */
	for(;;)
	{
		PT100_Temperature = MAX31865_Get_Temperature();
		if (PT100_Temperature >= 0) {
			PT100_Temperature = PT100_Temperature + 0.05;
			sprintf(temperature, "%d.%d", (uint16_t) (PT100_Temperature),((uint16_t) (PT100_Temperature * 100)- ((uint16_t) PT100_Temperature) * 100) / 10);
			temp = (float)atof(temperature);
		}
		else {
			PT100_Temperature = -PT100_Temperature + 0.05;
			sprintf(temperature, "-%d.%d", (uint16_t) (PT100_Temperature),((uint16_t) (PT100_Temperature * 100)- ((uint16_t) PT100_Temperature) * 100) / 10);
			temp = (float)atof(temperature);
		}

		osDelay(320);

	}
	/* USER CODE END Task7_Init */
}

/* USER CODE BEGIN Header_Task8_Init */
/**
 * @brief Function implementing the Task8 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Task8_Init */
void Task8_Init(void const * argument)
{
	/* USER CODE BEGIN Task8_Init */
	/* Infinite loop */
	for(;;)
	{

		if(durum==1){
			HAL_GPIO_WritePin(GPIOA, m1_Pin|m2_Pin|m3_Pin
					|m4_Pin, GPIO_PIN_SET);
		}
		else{
			HAL_GPIO_WritePin(GPIOA, m1_Pin|m2_Pin|m3_Pin
					|m4_Pin, GPIO_PIN_RESET);
		}
		osDelay(75);
	}
	/* USER CODE END Task8_Init */
}

/* USER CODE BEGIN Header_Task9_Init */
/**
 * @brief Function implementing the Task9 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Task9_Init */
void Task9_Init(void const * argument)
{
	/* USER CODE BEGIN Task9_Init */
	/* Infinite loop */
	for(;;)
	{
		osDelay(1);
	}
	/* USER CODE END Task9_Init */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM1 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM1) {
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
