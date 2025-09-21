/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BTN_UP_Pin GPIO_PIN_0
#define BTN_UP_GPIO_Port GPIOC
#define BTN_UP_EXTI_IRQn EXTI0_IRQn
#define BTN_DOWN_Pin GPIO_PIN_1
#define BTN_DOWN_GPIO_Port GPIOC
#define BTN_DOWN_EXTI_IRQn EXTI1_IRQn
#define BTN_LEFT_Pin GPIO_PIN_2
#define BTN_LEFT_GPIO_Port GPIOC
#define BTN_LEFT_EXTI_IRQn EXTI2_IRQn
#define LIGHT_MODE_Pin GPIO_PIN_3
#define LIGHT_MODE_GPIO_Port GPIOC
#define LIGHT_MODE_EXTI_IRQn EXTI3_IRQn
#define BTN_POT_MODE_Pin GPIO_PIN_4
#define BTN_POT_MODE_GPIO_Port GPIOC
#define BTN_POT_MODE_EXTI_IRQn EXTI4_IRQn
#define BTN_RIGHT_Pin GPIO_PIN_5
#define BTN_RIGHT_GPIO_Port GPIOC
#define BTN_RIGHT_EXTI_IRQn EXTI9_5_IRQn
#define LED_2_Pin GPIO_PIN_14
#define LED_2_GPIO_Port GPIOB
#define LED_1_Pin GPIO_PIN_15
#define LED_1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
