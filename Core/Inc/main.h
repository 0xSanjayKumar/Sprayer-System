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
#include "string.h"
#include "stdio.h"
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
void check_uart_data();
void Set_Motor_Direction(uint8_t direction);
void Enable_Motor(uint8_t enable);
void Generate_Steps(uint32_t num_steps, uint8_t direction);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define pcbTempMon_Pin GPIO_PIN_0
#define pcbTempMon_GPIO_Port GPIOC
#define pcbVoltageMon5_Pin GPIO_PIN_1
#define pcbVoltageMon5_GPIO_Port GPIOC
#define pcbVoltageMon3_Pin GPIO_PIN_2
#define pcbVoltageMon3_GPIO_Port GPIOC
#define pulseSignalTop_Pin GPIO_PIN_0
#define pulseSignalTop_GPIO_Port GPIOA
#define pulseSignalBase_Pin GPIO_PIN_1
#define pulseSignalBase_GPIO_Port GPIOA
#define servoDirTop_Pin GPIO_PIN_2
#define servoDirTop_GPIO_Port GPIOA
#define servoDirBase_Pin GPIO_PIN_3
#define servoDirBase_GPIO_Port GPIOA
#define alarmSigTop_Pin GPIO_PIN_4
#define alarmSigTop_GPIO_Port GPIOA
#define alarmSigBase_Pin GPIO_PIN_5
#define alarmSigBase_GPIO_Port GPIOA
#define proximityMinTop_Pin GPIO_PIN_6
#define proximityMinTop_GPIO_Port GPIOA
#define proximityMaxTop_Pin GPIO_PIN_7
#define proximityMaxTop_GPIO_Port GPIOA
#define proximityMinBase_Pin GPIO_PIN_0
#define proximityMinBase_GPIO_Port GPIOB
#define proximityMaxBase_Pin GPIO_PIN_1
#define proximityMaxBase_GPIO_Port GPIOB
#define systemSwitch_Pin GPIO_PIN_2
#define systemSwitch_GPIO_Port GPIOB
#define servoEnableTop_Pin GPIO_PIN_10
#define servoEnableTop_GPIO_Port GPIOB
#define servoEnableBase_Pin GPIO_PIN_11
#define servoEnableBase_GPIO_Port GPIOB
#define canSpiNss_Pin GPIO_PIN_12
#define canSpiNss_GPIO_Port GPIOB
#define canSpiSck_Pin GPIO_PIN_13
#define canSpiSck_GPIO_Port GPIOB
#define canSpiMiso_Pin GPIO_PIN_14
#define canSpiMiso_GPIO_Port GPIOB
#define canSpiMosi_Pin GPIO_PIN_15
#define canSpiMosi_GPIO_Port GPIOB
#define solenoidTop_Pin GPIO_PIN_6
#define solenoidTop_GPIO_Port GPIOC
#define solenoidBase_Pin GPIO_PIN_7
#define solenoidBase_GPIO_Port GPIOC
#define bypassSwitch_Pin GPIO_PIN_8
#define bypassSwitch_GPIO_Port GPIOC
#define dipSwitch1_Pin GPIO_PIN_9
#define dipSwitch1_GPIO_Port GPIOC
#define dipSwitch2_Pin GPIO_PIN_8
#define dipSwitch2_GPIO_Port GPIOA
#define canWakeUp_GPIO_Port GPIOB
#define canWakeUp_Pin GPIO_PIN_5
#define emergencySwitch_GPIO_PORT GPIOA
#define emergencySwitch_Pin	GPIO_PIN_11
#define spr_Input_1_GPIO_Port GPIOA
#define spr_Input_1_Pin GPIO_PIN_12
#define spr_Input_2_GPIO_Port GPIOD
#define spr_Input_2_Pin GPIO_PIN_2
#define spr_Input_3_GPIO_Port GPIOB
#define spr_Input_3_Pin GPIO_PIN_3
#define spr_Input_4_GPIO_Port GPIOB
#define spr_Input_4_Pin GPIO_PIN_4
#define spr_Output_1_GPIO_Port GPIOC
#define spr_Output_1_Pin GPIO_PIN_10
#define spr_Output_2_GPIO_Port GPIOC
#define spr_Output_2_Pin GPIO_PIN_11
#define spr_Output_3_GPIO_Port GPIOC
#define spr_Output_3_Pin GPIO_PIN_12
#define spr_Output_4_GPIO_Port GPIOA
#define spr_Output_4_Pin GPIO_PIN_15
#define led1_Pin GPIO_PIN_9
#define led1_GPIO_Port GPIOA
#define led2_Pin GPIO_PIN_10
#define led2_GPIO_Port GPIOA


/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
