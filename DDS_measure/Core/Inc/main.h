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
#define ILI9341_DC_Pin GPIO_PIN_4
#define ILI9341_DC_GPIO_Port GPIOC
#define ILI9341_RST_Pin GPIO_PIN_5
#define ILI9341_RST_GPIO_Port GPIOC
#define ILI9341_CS_Pin GPIO_PIN_12
#define ILI9341_CS_GPIO_Port GPIOB
#define KEY5_Pin GPIO_PIN_8
#define KEY5_GPIO_Port GPIOD
#define KEY5_EXTI_IRQn EXTI9_5_IRQn
#define KEY4_Pin GPIO_PIN_9
#define KEY4_GPIO_Port GPIOD
#define KEY4_EXTI_IRQn EXTI9_5_IRQn
#define KEY3_Pin GPIO_PIN_10
#define KEY3_GPIO_Port GPIOD
#define KEY3_EXTI_IRQn EXTI15_10_IRQn
#define KEY2_Pin GPIO_PIN_11
#define KEY2_GPIO_Port GPIOD
#define KEY2_EXTI_IRQn EXTI15_10_IRQn
#define KEY1_Pin GPIO_PIN_12
#define KEY1_GPIO_Port GPIOD
#define KEY1_EXTI_IRQn EXTI15_10_IRQn
#define LED_G_Pin GPIO_PIN_14
#define LED_G_GPIO_Port GPIOD
#define LED_R_Pin GPIO_PIN_8
#define LED_R_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */
#define HSPI_ILI9341 &hspi2
#define ILI9341_Block_Size_Maximum 200
#define ILI9341_SPI_TimeoutDuration 10
#define USE_LARGE_RAM

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
