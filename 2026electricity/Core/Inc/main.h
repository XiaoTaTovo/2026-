/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#define EN_Pin GPIO_PIN_2
#define EN_GPIO_Port GPIOC
#define DIR_Pin GPIO_PIN_3
#define DIR_GPIO_Port GPIOC
#define STEP_Pin GPIO_PIN_6
#define STEP_GPIO_Port GPIOA
#define Key_1_Pin GPIO_PIN_11
#define Key_1_GPIO_Port GPIOE
#define Key_2_Pin GPIO_PIN_12
#define Key_2_GPIO_Port GPIOE
#define Key_3_Pin GPIO_PIN_13
#define Key_3_GPIO_Port GPIOE
#define Key_4_Pin GPIO_PIN_14
#define Key_4_GPIO_Port GPIOE
#define electricity_Pin GPIO_PIN_10
#define electricity_GPIO_Port GPIOB
#define electricityB11_Pin GPIO_PIN_11
#define electricityB11_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
