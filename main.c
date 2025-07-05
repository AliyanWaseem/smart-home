/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

// for ultrasonic
#define TRIG_PIN GPIO_PIN_2
#define TRIG_PORT GPIOA
#define ECHO_PIN GPIO_PIN_3
#define ECHO_PORT GPIOA

// for temprature sensor
#define DHT11_PIN GPIO_PIN_8
#define DHT11_PORT GPIOA

// for buttons
#define DEBOUNCE_DELAY 50  // in milliseconds
#define RELAY_PORT GPIOB
#define RELAY_1 GPIO_PIN_0
#define RELAY_2 GPIO_PIN_1
#define RELAY_3 GPIO_PIN_2
#define RELAY_4 GPIO_PIN_10

// for esp
#define UART_RX_BUFFER_SIZE  40

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

// variables for water level ultrasonic sensor
uint32_t pMillis; // timeout
uint32_t Value1 = 0;
uint32_t Value2 = 0;
uint16_t Distance  = 0;  // cm
uint16_t New_Water_level = 0; // water level in percentage
uint16_t Last_Water_level = 0; // water level in percentage

// variables for temprature sensor
uint8_t RHI, RHD, TCI, TCD, Presence, SUM;
uint32_t pMillisec, cMillisec; // timeouts
float New_tCelsius = 0;
float Last_tCelsius = 0;
float RH = 0;

int debug=0;
int debug2=0;
int count=0;

// variables for ADC & PWM
uint16_t readValue;
int speed;

// variables for buttons
uint32_t lastDebounceTime_PA11 = 0;
uint32_t lastDebounceTime_PA12 = 0;
uint32_t lastDebounceTime_PA15 = 0;
uint32_t lastDebounceTime_PB3  = 0;

// variables for esp uart
uint8_t UART1_RxBuffer[UART_RX_BUFFER_SIZE] = {0};
char msg[30];
uint16_t RxDataLen = 0;
uint8_t recieved_data[UART_RX_BUFFER_SIZE] = {0};
bool recieved = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void CopyData();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void SendLabelValueToESP(const char* label, int value)
{
	memset(msg, 0, sizeof(msg)); // clear the msg
    snprintf(msg, sizeof(msg), "%s%d\r\n", label, value);  // Format: LABEL:xx.xx
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    debug=7;
}

void delay_us(uint16_t us)
{
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    while (__HAL_TIM_GET_COUNTER(&htim1) < us);
}

void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStructPrivate = {0};
    GPIO_InitStructPrivate.Pin = GPIO_Pin;
    GPIO_InitStructPrivate.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructPrivate.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStructPrivate.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStructPrivate); // set the pin as output
}

void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStructPrivate = {0};
	GPIO_InitStructPrivate.Pin = GPIO_Pin;
	GPIO_InitStructPrivate.Mode = GPIO_MODE_INPUT;
	GPIO_InitStructPrivate.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStructPrivate); // set the pin as input
}
void DHT11_Start(void)
{
    // Set pin as output and pull low for 18ms
    Set_Pin_Output(DHT11_PORT, DHT11_PIN);
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_RESET);
    HAL_Delay(20); // low for greater than 18ms start signal from MCU
    // Pull high and switch to input
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_SET);
    delay_us(30);
    Set_Pin_Input(DHT11_PORT, DHT11_PIN);
}

uint8_t Check_Response(void)
{
    uint8_t Response = 0;
    delay_us(40);
    if (!HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN))
    {
        delay_us(80);
        if (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)) Response = 1;
    }
    pMillisec = HAL_GetTick();
    cMillisec = HAL_GetTick();
    while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)) && pMillisec + 2 > cMillisec) // Wait for line to go low
    {
       cMillisec = HAL_GetTick();
    }
    return Response;
}

uint8_t DHT11_Read(void)
{
	uint8_t a,b;
	for (a=0;a<8;a++)
	{
	   pMillisec = HAL_GetTick();
	   cMillisec = HAL_GetTick();
	   while (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)) && pMillisec + 2 > cMillisec)
	   {  // wait for the pin to go high
	     cMillisec = HAL_GetTick();
	   }
	   delay_us (40);   // wait for 40 us
	   if (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)))   // if the pin is low
	   {
	      b&= ~(1<<(7-a));
	   }
	   else
	   {
   	      b|= (1<<(7-a));
	   }
	   pMillisec = HAL_GetTick();
	   cMillisec = HAL_GetTick();
	   while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)) && pMillisec + 2 > cMillisec)
	   {  // wait for the pin to go low
	     cMillisec = HAL_GetTick();
	   }
	}
	return b;
}

void DHT_data()
{
	DHT11_Start();
	if (Check_Response())
	{
		RHI = DHT11_Read();
		RHD = DHT11_Read();
		TCI = DHT11_Read();
		TCD = DHT11_Read();
		SUM = DHT11_Read();

		if ((RHI + RHD + TCI + TCD) == SUM)
		{
			New_tCelsius = (float)TCI + (float)(TCD / 10.0);
		    RH = (float)RHI + (float)(RHD/10.0);
		}
		else
		{
		    //tCelsius = 0; // Reset if checksum fails
		    RH = 0;
		}
		if (abs((int)New_tCelsius - (int)Last_tCelsius) >= 1)
		{
			debug=15;
			SendLabelValueToESP("TEMP:",(int)New_tCelsius);
			SendLabelValueToESP("HUM:",(int)RH);
			Last_tCelsius = New_tCelsius;
		}
	}

}

void Hcsr04_data()
{
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	delay_us(10); // wait for 10 us
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low
    pMillis = HAL_GetTick(); // used this to avoid infinite while loop  (for timeout)
    // wait for the echo pin to go high
    while (!(HAL_GPIO_ReadPin (ECHO_PORT, ECHO_PIN)) && pMillis + 10 >  HAL_GetTick());
    Value1 = __HAL_TIM_GET_COUNTER (&htim1);
    pMillis = HAL_GetTick(); // used this to avoid infinite while loop (for timeout)
    while ((HAL_GPIO_ReadPin (ECHO_PORT, ECHO_PIN)) && pMillis + 50 > HAL_GetTick());
    Value2 = __HAL_TIM_GET_COUNTER (&htim1);
    Distance = (Value2-Value1)* 0.034/2; // for inches conversion
    New_Water_level = (uint16_t)((20 - Distance) * 100.0 / 20.0);
    if (abs(New_Water_level - Last_Water_level) >= 1)
    {
    	debug=5;
    	SendLabelValueToESP("DIST:",(int)New_Water_level);
    	if(New_Water_level>=90) // stop the motor if running
    	{
    		// pump is connected to relay 3
    	  HAL_GPIO_WritePin(RELAY_PORT, RELAY_3, GPIO_PIN_SET);  // Active LOW
    	  SendLabelValueToESP("PUMP",4); // integer can be any value dont care
    	}
    	Last_Water_level = New_Water_level;
    }


}

void ADC_PWM()
{
	HAL_ADC_Start(&hadc1);  // Start a single conversion
	HAL_ADC_PollForConversion(&hadc1,1000);
	readValue = HAL_ADC_GetValue(&hadc1);
	// Map 0–4095 to 0–49 (ARR = 49)
	speed = (readValue * 49) / 4095;
	// Update PWM duty cycle
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed);
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
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start(&hadc1);  //PA7
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); //PA6 TIM3 CH1
  HAL_TIM_Base_Start(&htim1);
  HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low
  HAL_UARTEx_ReceiveToIdle_IT(&huart1, UART1_RxBuffer, UART_RX_BUFFER_SIZE);
  uint32_t lastSendTimeHCSR04 = 0;
  uint32_t lastSendTimeDHT11 = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // for checking data after some time reduce frequently checking data
	  	  uint32_t nowHCSR04 = HAL_GetTick();
	  	  uint32_t nowDHT11 = HAL_GetTick();

	  	  if (nowHCSR04 - lastSendTimeHCSR04 >= 1000) // every 2 seconds
	  	  {
	  	      Hcsr04_data();
	  	      lastSendTimeHCSR04 = nowHCSR04;
	  	  }
	  	  if (nowDHT11 - lastSendTimeDHT11 >= 2000) // every 5 seconds
	  	  {
	  	      DHT_data();
	  	      lastSendTimeDHT11 = nowDHT11;
	  	  }
	  	  ADC_PWM();

	  	  if(recieved)
	  	  {
	  		recieved = false; // Reset flag
	  	  	// Example command parsing
	  	  	if (strncmp((char*)recieved_data, "RELAY1_ON", 9) == 0)
	  	  	{
	  	  	     HAL_GPIO_WritePin(RELAY_PORT, RELAY_1, GPIO_PIN_RESET); // Turn on water pump
	  	  	     debug=5;
	  	  	}
	  	  	else if (strncmp((char*)recieved_data, "RELAY1_OFF", 10) == 0)
	  	  	{
	  	  	     HAL_GPIO_WritePin(RELAY_PORT, RELAY_1, GPIO_PIN_SET);   // Turn off water pump
	  	  	}
	  	  	else if (strncmp((char*)recieved_data, "RELAY2_ON", 9) == 0)
	  	  	{
	  	  	 	 HAL_GPIO_WritePin(RELAY_PORT, RELAY_2, GPIO_PIN_RESET);   // Turn on LED
	  	  	}
	  	  	else if (strncmp((char*)recieved_data, "RELAY2_OFF", 10) == 0)
	  	    {
	  	  		 HAL_GPIO_WritePin(RELAY_PORT, RELAY_2, GPIO_PIN_SET);   // Turn off LED
	  	    }
	  	  	else if (strncmp((char*)recieved_data, "RELAY3_ON", 9) == 0)
	  	  	{
	  	  	 	 HAL_GPIO_WritePin(RELAY_PORT, RELAY_3, GPIO_PIN_RESET);   // Turn on bulb
	  	  	}
	  	  	else if (strncmp((char*)recieved_data, "RELAY3_OFF", 10) == 0)
	  	  	{
	  	  		 HAL_GPIO_WritePin(RELAY_PORT, RELAY_3, GPIO_PIN_SET);   // Turn off bulb
	  	  	}
	  	  	else if (strncmp((char*)recieved_data, "RELAY4_ON", 9) == 0)
	  	  	{
	  	  	 	 HAL_GPIO_WritePin(RELAY_PORT, RELAY_4, GPIO_PIN_RESET);   // Turn on fan
	  	  	}
	  	  	else if (strncmp((char*)recieved_data, "RELAY4_OFF", 10) == 0)
	  	  	{
	  	  		 HAL_GPIO_WritePin(RELAY_PORT, RELAY_4, GPIO_PIN_SET);   // Turn off fan
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  sConfig.Channel = ADC_CHANNEL_7;
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
  htim1.Init.Prescaler = 83;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 49;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart1.Init.BaudRate = 9600;
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA2 PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA11 PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  uint32_t currentTime = HAL_GetTick();
  // buttons for toggeling state
  debug2=10;
  if (GPIO_Pin == GPIO_PIN_11 && (currentTime - lastDebounceTime_PA11) > DEBOUNCE_DELAY)
  {
	  debug2=11;
    lastDebounceTime_PA11 = currentTime;
    HAL_GPIO_TogglePin(RELAY_PORT, RELAY_1);
    SendLabelValueToESP("BTN",1);
  }
  else if (GPIO_Pin == GPIO_PIN_12 && (currentTime - lastDebounceTime_PA12) > DEBOUNCE_DELAY)
  {
	  debug=12;
    lastDebounceTime_PA12 = currentTime;
    HAL_GPIO_TogglePin(RELAY_PORT, RELAY_2);
    SendLabelValueToESP("BTN",2);
  }
  else if (GPIO_Pin == GPIO_PIN_15 && (currentTime - lastDebounceTime_PA15) > DEBOUNCE_DELAY)
  {
	  debug=15;
    lastDebounceTime_PA15 = currentTime;
    HAL_GPIO_TogglePin(RELAY_PORT, RELAY_3);
    SendLabelValueToESP("BTN",3);
  }
  else if (GPIO_Pin == GPIO_PIN_3 && (currentTime - lastDebounceTime_PB3) > DEBOUNCE_DELAY)
  {
    lastDebounceTime_PB3 = currentTime;
    HAL_GPIO_TogglePin(RELAY_PORT, RELAY_4);
    SendLabelValueToESP("BTN",4);
  }
}

void CopyData()
{
	for(int i=0;i<RxDataLen;i++)
	{
		recieved_data[i]=UART1_RxBuffer[i];
	}
	debug=2;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    RxDataLen = Size;
    recieved=true;
    CopyData(); // copy data to other buffer before processing and receiving next data
    // Clear buffer
    debug=1;
    memset(UART1_RxBuffer, 0, UART_RX_BUFFER_SIZE);
    HAL_UARTEx_ReceiveToIdle_IT(&huart1, UART1_RxBuffer, UART_RX_BUFFER_SIZE);
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
