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
#include "stm32l4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define pcbTempMon_Pin GPIO_PIN_0
#define pcbTempMon_GPIO_Port GPIOC
#define pcb5vMon_Pin GPIO_PIN_1
#define pcb5vMon_GPIO_Port GPIOC
#define pcb3vMon_Pin GPIO_PIN_2
#define pcb3vMon_GPIO_Port GPIOC
#define pulseSignalTop_Pin GPIO_PIN_0
#define pulseSignalTop_GPIO_Port GPIOA
#define pulseSignalBase_Pin GPIO_PIN_1
#define pulseSignalBase_GPIO_Port GPIOA
#define topDir_Pin GPIO_PIN_2
#define topDir_GPIO_Port GPIOA
#define baseDir_Pin GPIO_PIN_3
#define baseDir_GPIO_Port GPIOA
#define alarmTop_Pin GPIO_PIN_4
#define alarmTop_GPIO_Port GPIOA
#define alarmBase_Pin GPIO_PIN_5
#define alarmBase_GPIO_Port GPIOA
#define proxMinTop_Pin GPIO_PIN_6
#define proxMinTop_GPIO_Port GPIOA
#define proxMaxTop_Pin GPIO_PIN_7
#define proxMaxTop_GPIO_Port GPIOA
#define proxMinBase_Pin GPIO_PIN_0
#define proxMinBase_GPIO_Port GPIOB
#define proxMaxBase_Pin GPIO_PIN_1
#define proxMaxBase_GPIO_Port GPIOB
#define systemSw_Pin GPIO_PIN_2
#define systemSw_GPIO_Port GPIOB
#define ENTop_Pin GPIO_PIN_10
#define ENTop_GPIO_Port GPIOB
#define EN_Base_Pin GPIO_PIN_11
#define EN_Base_GPIO_Port GPIOB
#define solenoidTop_Pin GPIO_PIN_6
#define solenoidTop_GPIO_Port GPIOC
#define solenoidBase_Pin GPIO_PIN_7
#define solenoidBase_GPIO_Port GPIOC
#define byps_Pin GPIO_PIN_8
#define byps_GPIO_Port GPIOC
#define dips2_Pin GPIO_PIN_9
#define dips2_GPIO_Port GPIOC
#define dips1_Pin GPIO_PIN_8
#define dips1_GPIO_Port GPIOA
#define led2_Pin GPIO_PIN_9
#define led2_GPIO_Port GPIOA
#define led1_Pin GPIO_PIN_10
#define led1_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
