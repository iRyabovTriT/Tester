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
RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim14;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM14_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t StartTest = 0;
GPIO_TypeDef* PortLed[] = {GPIOB, GPIOB, GPIOB, GPIOB, GPIOB, GPIOB, GPIOD, GPIOC, GPIOC, GPIOC, GPIOA};
uint16_t PinLed[] = {LED1_Pin, LED2_Pin, LED3_Pin, LED4_Pin, LED5_Pin, LED6_Pin, LED7_Pin, LED8_Pin, LED9_Pin, LED10_Pin, LED11_Pin};

static uint8_t ErrorTest = 0;

static uint16_t ResPinTest[11] = {0,0,0,0,0,0,0,0,0,0,0};
const uint16_t Cabel_Pin[] = {CABEL1_O_Pin, CABEL2_O_Pin, CABEL3_O_Pin, CABEL4_O_Pin, CABEL5_O_Pin, CABEL6_O_Pin, CABEL7_O_Pin, CABEL8_O_Pin, CABEL9_O_Pin, CABEL10_O_Pin, CABEL11_O_Pin};
static int count = 0;
static int countCabelInter = 0;
static int countCabelInterIndivid = 0; // для 2 и 11 пина
static int arrayCount = 0;
static int ShortCut = 0;
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
  MX_RTC_Init();
  MX_TIM6_Init();
  MX_TIM14_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if(arrayCount < 11)
	  {
		  HAL_GPIO_WritePin(GetPort(arrayCount), Cabel_Pin[arrayCount], GPIO_PIN_SET);
		  //HAL_GPIO_TogglePin(RED_LED_GPIO_Port, RED_LED_Pin);

		  HAL_Delay(150);

		  HAL_GPIO_WritePin(GetPort(arrayCount), Cabel_Pin[arrayCount], GPIO_PIN_RESET);

		  /*
		   * CASE A
		   * Замыкание на 1-5, 8 - 11 пине
		   *
		   * CASE B
		   * Замыкание на 6, 7 пине
		   * */
		  /*
		  switch(count)
		  {
		  case 5:
			  if(countCabelInterIndivid > 2)
			  {
				  HAL_GPIO_WritePin(PortLed[arrayCount], PinLed[arrayCount], GPIO_PIN_SET);
				  ErrorTest = 1;
				  ShortCut = 1;
			  }
			  break;
		  case 6:
			  if(countCabelInterIndivid > 2)
			  {
				  HAL_GPIO_WritePin(PortLed[arrayCount], PinLed[arrayCount], GPIO_PIN_SET);
				  ErrorTest = 1;
				  ShortCut = 1;
			  }
			  break;
		  default:
			  if(countCabelInter > 1)
			  {
				  HAL_GPIO_WritePin(PortLed[arrayCount], PinLed[arrayCount], GPIO_PIN_SET);
				  ErrorTest = 1;
				  ShortCut = 1;
			  }
			  break;
		  }
	*/
		  count++;
		  arrayCount++;
		  countCabelInter = 0;
		  countCabelInterIndivid = 0;
	  }

	  if(arrayCount == 11)
	  {
		  for(int i = 0; i < 11; i++)
		  {
			  if(ResPinTest[i] == 0)
			  {
				  ErrorTest = 1;
			  }
		  }
		  if(ErrorTest)
		  {
			  arrayCount++;

			  HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);
			  if(ShortCut == 1)
			  {
				  for(int i = 0; i < 3; i++)
				  {
				  		HAL_GPIO_WritePin(SPEAKER_GPIO_Port, SPEAKER_Pin, GPIO_PIN_SET);
				  		HAL_Delay(250);
				  		HAL_GPIO_WritePin(SPEAKER_GPIO_Port, SPEAKER_Pin, GPIO_PIN_RESET);
				  		HAL_Delay(250);
				  }
			  }
			  else
			  {
				  for(int i = 0; i < 3; i++)
				  {
				  	HAL_GPIO_WritePin(SPEAKER_GPIO_Port, SPEAKER_Pin, GPIO_PIN_SET);
				  	if(i == 2)
				  	{
				  		HAL_Delay(1000);
				  	}else
				  	{
				  		HAL_Delay(250);
				  	}

				  	HAL_GPIO_WritePin(SPEAKER_GPIO_Port, SPEAKER_Pin, GPIO_PIN_RESET);
				  	HAL_Delay(250);
				  }
			  }

		  }
		  else
		  {
			  HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);
		  }

	  }

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* EXTI0_1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
  /* EXTI2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);
  /* EXTI4_15_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
  /* TIM6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM6_IRQn);
  /* RCC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(RCC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(RCC_IRQn);
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 359;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 10000;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 0;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 65535;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GREEN_LED_Pin|RED_LED_Pin|LED7_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SPEAKER_Pin|CABEL9_O_Pin|CABEL10_O_Pin|CABEL8_O_Pin
                          |CABEL6_O_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, CABEL11_O_Pin|CABEL2_O_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CABEL4_O_Pin|CABEL5_O_Pin|CABEL1_O_Pin|CABEL3_O_Pin
                          |CABEL7_O_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED3_Pin|LED1_Pin|LED5_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED2_Pin|LED9_Pin|LED10_Pin|LED8_Pin
                          |LED6_Pin|LED11_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : CABEL1_I_Pin CABEL3_I_Pin CABEL7_I_Pin */
  GPIO_InitStruct.Pin = CABEL1_I_Pin|CABEL3_I_Pin|CABEL7_I_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : GREEN_LED_Pin RED_LED_Pin */
  GPIO_InitStruct.Pin = GREEN_LED_Pin|RED_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SPEAKER_Pin CABEL9_O_Pin CABEL10_O_Pin CABEL8_O_Pin
                           CABEL6_O_Pin */
  GPIO_InitStruct.Pin = SPEAKER_Pin|CABEL9_O_Pin|CABEL10_O_Pin|CABEL8_O_Pin
                          |CABEL6_O_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CABEL11_O_Pin CABEL2_O_Pin */
  GPIO_InitStruct.Pin = CABEL11_O_Pin|CABEL2_O_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : CABEL4_O_Pin CABEL5_O_Pin CABEL1_O_Pin CABEL3_O_Pin
                           CABEL7_O_Pin */
  GPIO_InitStruct.Pin = CABEL4_O_Pin|CABEL5_O_Pin|CABEL1_O_Pin|CABEL3_O_Pin
                          |CABEL7_O_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : CABEL5_I_Pin CABEL4_I_Pin CABEL2_I_Pin */
  GPIO_InitStruct.Pin = CABEL5_I_Pin|CABEL4_I_Pin|CABEL2_I_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : CABEL9_I_Pin CABEL10_I_Pin CABEL8_I_Pin */
  GPIO_InitStruct.Pin = CABEL9_I_Pin|CABEL10_I_Pin|CABEL8_I_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CABEL6_I_Pin CABEL11_I_Pin */
  GPIO_InitStruct.Pin = CABEL6_I_Pin|CABEL11_I_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : LED7_Pin */
  GPIO_InitStruct.Pin = LED7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED7_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED3_Pin LED1_Pin LED5_Pin */
  GPIO_InitStruct.Pin = LED3_Pin|LED1_Pin|LED5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LED4_Pin */
  GPIO_InitStruct.Pin = LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED2_Pin LED9_Pin LED10_Pin LED8_Pin
                           LED6_Pin LED11_Pin */
  GPIO_InitStruct.Pin = LED2_Pin|LED9_Pin|LED10_Pin|LED8_Pin
                          |LED6_Pin|LED11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
#define STOP 0
#define START 1
#define ERROR 0
#define SUCCESS 1

static uint8_t TestState = STOP;
static uint8_t TestRes = ERROR;

void TestCase(void)
{
	if(TestState == START)
	{

	}else return;
}

static uint8_t wait = 0;


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);

  switch(GPIO_Pin)
  {
  	  case BUTTON_Pin:
  		  break;
  	  case CABEL1_I_Pin:
  		  countCabelInter++;
  		  if(count == 0 && countCabelInter == 1)
  		  {
  			  ResPinTest[0] = 1;
  			  LedOn(LED1_GPIO_Port, LED1_Pin);
  		  }
  		  break;
  	  case CABEL2_I_Pin:
  		  countCabelInter++;
  		  if(count == 1 && countCabelInter == 1)
  		  {
  			  ResPinTest[1] = 1;
  			  LedOn(LED2_GPIO_Port, LED2_Pin);
  		  }
  		  break;
  	  case CABEL3_I_Pin:
  		  countCabelInter++;
  		  if(count == 2 && countCabelInter == 1)
  		  {
  			  ResPinTest[2] = 1;
  			  LedOn(LED3_GPIO_Port, LED3_Pin);
  		  }
  	  	  break;
  	  case CABEL4_I_Pin:
  		  countCabelInter++;
  		  if(count == 3 && countCabelInter == 1)
  		  {
  			  ResPinTest[3] = 1;
  			  LedOn(LED4_GPIO_Port, LED4_Pin);
  		  }
  		  break;
  	  case CABEL5_I_Pin:
  		  countCabelInter++;
  		  if(count == 4 && countCabelInter == 1)
  		  {
  			  ResPinTest[4] = 1;
  			  LedOn(LED5_GPIO_Port, LED5_Pin);
  		  }
  		  break;
  	  case CABEL6_I_Pin:
  		  countCabelInter++;
  		  if(count == 5 && (countCabelInter == 1 || countCabelInter == 2))
  		  {
  			  LedOn(LED6_GPIO_Port, LED6_Pin);
  			  ResPinTest[5] = 1;
  		  }
  		  break;
  	  case CABEL7_I_Pin:
  		  countCabelInter++;
  		  if(count == 6 && (countCabelInter == 1 || countCabelInter == 2))
  		  {
  			  ResPinTest[6] = 1;
  			  LedOn(LED7_GPIO_Port, LED7_Pin);
  		  }
  		  break;
  	  case CABEL8_I_Pin:
  		  countCabelInter++;
  		  if(count == 7 && countCabelInter == 1)
  		  {
  			  ResPinTest[7] = 1;
  			  LedOn(LED8_GPIO_Port, LED8_Pin);
  		  }
  		  break;
  	  case CABEL9_I_Pin:
  		  countCabelInter++;
  		  if(count == 8 && countCabelInter == 1)
  		  {
  			  ResPinTest[8] = 1;
  			  LedOn(LED9_GPIO_Port, LED9_Pin);
  		  }
  		  break;
  	  case CABEL10_I_Pin:
  		  countCabelInter++;
  		  if(count == 9 && countCabelInter == 1)
  		  {
  			  ResPinTest[9] = 1;
  			  LedOn(LED10_GPIO_Port, LED10_Pin);
  		  }
  		  break;
  	  case CABEL11_I_Pin:
  		  countCabelInter++;
  		  if(count == 10 && countCabelInter == 1)
  		  {
  			  ResPinTest[10] = 1;
  			  LedOn(LED11_GPIO_Port, LED11_Pin);
  		  }
  		  break;
  	  default:
  		  TestRes = ERROR;
  		  break;
  }

}



void Delay(uint32_t value)
{
	while(value > 0)
	{
		value--;
	}
}

void CabelTestStart()
{
	StartTest = 0;

	for(int i = 0; i < 11; i++)
	{
		wait = 1;
		HAL_GPIO_WritePin(GetPort(i), Cabel_Pin[i], GPIO_PIN_SET);
		Delay(0xffffff);
		HAL_GPIO_WritePin(GetPort(i), Cabel_Pin[i], GPIO_PIN_RESET);
		if(TestRes == SUCCESS)
		{
			//ResPinTest[i] = SUCCESS;
		}
		TestRes = ERROR;
	}

	for(int i = 0; i < 11; i++)
	{
		if(ResPinTest[i] == ERROR)
		{
			ErrorTest = 1;
		}
	}
	if(ErrorTest == 1)
	{
		SignalError();
		ErrorTest = 0;
	}else
	{
		LedOn(GREEN_LED_GPIO_Port, GREEN_LED_Pin);
	}
	StartTest = 0;
	ErrorTest = 0;
	for(int i = 0; i < 11; i++)
	{
	  	ResPinTest[i] = ERROR;
	}

}



GPIO_TypeDef* GetPort(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin)
	{
	case 0:
		return CABEL1_O_GPIO_Port;
		break;
	case 1:
		return CABEL2_O_GPIO_Port;
		break;
	case 2:
		return CABEL3_O_GPIO_Port;
		break;
	case 3:
		return CABEL4_O_GPIO_Port;
		break;
	case 4:
		return CABEL5_O_GPIO_Port;
		break;
	case 5:
		return CABEL6_O_GPIO_Port;
		break;
	case 6:
		return CABEL7_O_GPIO_Port;
		break;
	case 7:
		return CABEL8_O_GPIO_Port;
		break;
	case 8:
		return CABEL9_O_GPIO_Port;
		break;
	case 9:
		return CABEL10_O_GPIO_Port;
		break;
	case 10:
		return CABEL11_O_GPIO_Port;
		break;
	default:
		return GPIOA;
		break;
	}
}

void LedOn(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
}

void LedOff(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
}

void LedOffAll(void)
{
	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, RED_LED_Pin|LED11_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(GPIOA, SPEAKER_Pin|CABEL1_O_Pin|CABEL2_O_Pin
		                         |CABEL3_O_Pin|CABEL4_O_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(GPIOA, GREEN_LED_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, LED10_Pin|LED9_Pin|LED8_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(GPIOC, CABEL5_O_Pin|CABEL6_O_Pin, GPIO_PIN_RESET);
	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, LED6_Pin|LED5_Pin|LED4_Pin|LED3_Pin|LED2_Pin|LED1_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(GPIOB, CABEL7_O_Pin|CABEL8_O_Pin|CABEL9_O_Pin|CABEL10_O_Pin
		                         |CABEL11_O_Pin, GPIO_PIN_RESET);
	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, GPIO_PIN_SET);
}

void SignalError(void)
{
	HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(SPEAKER_GPIO_Port, SPEAKER_Pin, GPIO_PIN_SET);

	Delay(0xffff);


	HAL_GPIO_WritePin(SPEAKER_GPIO_Port, SPEAKER_Pin, GPIO_PIN_RESET);

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
  //__disable_irq();
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
