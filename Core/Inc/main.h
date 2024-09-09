/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void TestCase(void);
void LedOn(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void LedOff(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void LedOffAll(void);
void CabelTestStart();
GPIO_TypeDef* GetPort(uint16_t GPIO_Pin);
void SignalError(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CABEL9_I_Pin GPIO_PIN_0
#define CABEL9_I_GPIO_Port GPIOC
#define CABEL9_I_EXTI_IRQn EXTI0_1_IRQn
#define CABEL10_I_Pin GPIO_PIN_1
#define CABEL10_I_GPIO_Port GPIOC
#define CABEL10_I_EXTI_IRQn EXTI0_1_IRQn
#define CABEL11_I_Pin GPIO_PIN_2
#define CABEL11_I_GPIO_Port GPIOC
#define CABEL11_I_EXTI_IRQn EXTI2_3_IRQn
#define GREEN_LED_Pin GPIO_PIN_1
#define GREEN_LED_GPIO_Port GPIOA
#define SPEAKER_Pin GPIO_PIN_2
#define SPEAKER_GPIO_Port GPIOA
#define RED_LED_Pin GPIO_PIN_3
#define RED_LED_GPIO_Port GPIOA
#define BUTTON_Pin GPIO_PIN_4
#define BUTTON_GPIO_Port GPIOF
#define BUTTON_EXTI_IRQn EXTI4_15_IRQn
#define CABEL1_O_Pin GPIO_PIN_4
#define CABEL1_O_GPIO_Port GPIOA
#define CABEL2_O_Pin GPIO_PIN_5
#define CABEL2_O_GPIO_Port GPIOA
#define CABEL3_O_Pin GPIO_PIN_6
#define CABEL3_O_GPIO_Port GPIOA
#define CABEL4_O_Pin GPIO_PIN_7
#define CABEL4_O_GPIO_Port GPIOA
#define CABEL5_O_Pin GPIO_PIN_4
#define CABEL5_O_GPIO_Port GPIOC
#define CABEL6_O_Pin GPIO_PIN_5
#define CABEL6_O_GPIO_Port GPIOC
#define CABEL7_O_Pin GPIO_PIN_0
#define CABEL7_O_GPIO_Port GPIOB
#define CABEL8_O_Pin GPIO_PIN_1
#define CABEL8_O_GPIO_Port GPIOB
#define CABEL9_O_Pin GPIO_PIN_2
#define CABEL9_O_GPIO_Port GPIOB
#define CABEL10_O_Pin GPIO_PIN_10
#define CABEL10_O_GPIO_Port GPIOB
#define CABEL11_O_Pin GPIO_PIN_11
#define CABEL11_O_GPIO_Port GPIOB
#define CABEL8_I_Pin GPIO_PIN_12
#define CABEL8_I_GPIO_Port GPIOB
#define CABEL8_I_EXTI_IRQn EXTI4_15_IRQn
#define CABEL7_I_Pin GPIO_PIN_13
#define CABEL7_I_GPIO_Port GPIOB
#define CABEL7_I_EXTI_IRQn EXTI4_15_IRQn
#define CABEL6_I_Pin GPIO_PIN_14
#define CABEL6_I_GPIO_Port GPIOB
#define CABEL6_I_EXTI_IRQn EXTI4_15_IRQn
#define CABEL5_I_Pin GPIO_PIN_9
#define CABEL5_I_GPIO_Port GPIOA
#define CABEL5_I_EXTI_IRQn EXTI4_15_IRQn
#define CABEL4_I_Pin GPIO_PIN_10
#define CABEL4_I_GPIO_Port GPIOA
#define CABEL4_I_EXTI_IRQn EXTI4_15_IRQn
#define CABEL3_I_Pin GPIO_PIN_11
#define CABEL3_I_GPIO_Port GPIOA
#define CABEL3_I_EXTI_IRQn EXTI4_15_IRQn
#define CABEL2_I_Pin GPIO_PIN_6
#define CABEL2_I_GPIO_Port GPIOF
#define CABEL2_I_EXTI_IRQn EXTI4_15_IRQn
#define CABEL1_I_Pin GPIO_PIN_7
#define CABEL1_I_GPIO_Port GPIOF
#define CABEL1_I_EXTI_IRQn EXTI4_15_IRQn
#define LED11_Pin GPIO_PIN_15
#define LED11_GPIO_Port GPIOA
#define LED10_Pin GPIO_PIN_10
#define LED10_GPIO_Port GPIOC
#define LED9_Pin GPIO_PIN_11
#define LED9_GPIO_Port GPIOC
#define LED8_Pin GPIO_PIN_12
#define LED8_GPIO_Port GPIOC
#define LED7_Pin GPIO_PIN_2
#define LED7_GPIO_Port GPIOD
#define LED6_Pin GPIO_PIN_3
#define LED6_GPIO_Port GPIOB
#define LED5_Pin GPIO_PIN_4
#define LED5_GPIO_Port GPIOB
#define LED4_Pin GPIO_PIN_5
#define LED4_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_6
#define LED3_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_7
#define LED2_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_8
#define LED1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
