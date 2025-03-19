/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.c
  * @brief   This file provides code for the configuration
  *          of the I2C instances.
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
#include "i2c.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

I2C_HandleTypeDef hi2c1;

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00B07CB4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
    PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C1 clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();
  /* USER CODE BEGIN I2C1_MspInit 1 */

  /* USER CODE END I2C1_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspDeInit 0 */

  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();

    /**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_7);

  /* USER CODE BEGIN I2C1_MspDeInit 1 */

  /* USER CODE END I2C1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
// Function to write a byte to EEPROM
HAL_StatusTypeDef EEPROM_WriteByte(uint16_t memAddress, uint8_t data) {
    uint8_t buffer[3];
    buffer[0] = (memAddress >> 8) & 0xFF; // High byte of memory address
    buffer[1] = memAddress & 0xFF;        // Low byte of memory address
    buffer[2] = data;                     // Data byte

    return HAL_I2C_Master_Transmit(&hi2c1, EEPROM_ADDR, buffer, 3, HAL_MAX_DELAY);
}

// Function to read a byte from EEPROM
HAL_StatusTypeDef EEPROM_ReadByte(uint16_t memAddress, uint8_t *data) {
    uint8_t addressBuffer[2];
    addressBuffer[0] = (memAddress >> 8) & 0xFF; // High byte of memory address
    addressBuffer[1] = memAddress & 0xFF;        // Low byte of memory address

    // Send memory address
    if (HAL_I2C_Master_Transmit(&hi2c1, EEPROM_ADDR, addressBuffer, 2, HAL_MAX_DELAY) != HAL_OK) {
        return HAL_ERROR;
    }

    // Read data from EEPROM
    return HAL_I2C_Master_Receive(&hi2c1, EEPROM_ADDR, data, 1, HAL_MAX_DELAY);
}

// Function to write a block of data to EEPROM (Page Write)
HAL_StatusTypeDef EEPROM_WritePage(uint16_t memAddress, uint8_t *data, uint16_t length) {
    uint8_t buffer[66]; // 2 bytes for address + 64 bytes data
    if (length > 64) return HAL_ERROR; // Page size limit

    buffer[0] = (memAddress >> 8) & 0xFF; // High byte
    buffer[1] = memAddress & 0xFF;        // Low byte
    memcpy(&buffer[2], data, length);     // Copy data

    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c1, EEPROM_ADDR, buffer, length + 2, HAL_MAX_DELAY);
    HAL_Delay(5); // EEPROM Write Cycle Time (5ms)
    return status;
}

// Function to read a block of data from EEPROM
HAL_StatusTypeDef EEPROM_ReadBlock(uint16_t memAddress, uint8_t *data, uint16_t length) {
    uint8_t addressBuffer[2];
    addressBuffer[0] = (memAddress >> 8) & 0xFF;
    addressBuffer[1] = memAddress & 0xFF;

    // Send memory address
    if (HAL_I2C_Master_Transmit(&hi2c1, EEPROM_ADDR, addressBuffer, 2, HAL_MAX_DELAY) != HAL_OK) {
        return HAL_ERROR;
    }

    // Read data
    return HAL_I2C_Master_Receive(&hi2c1, EEPROM_ADDR, data, length, HAL_MAX_DELAY);
}
/* USER CODE END 1 */
