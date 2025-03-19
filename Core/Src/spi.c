/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.c
  * @brief   This file provides code for the configuration
  *          of the SPI instances.
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
#include "spi.h"
#include "usart.h"
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

SPI_HandleTypeDef hspi2;

/* SPI2 init function */
void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(spiHandle->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspInit 0 */

  /* USER CODE END SPI2_MspInit 0 */
    /* SPI2 clock enable */
    __HAL_RCC_SPI2_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**SPI2 GPIO Configuration
    PB12     ------> SPI2_NSS
    PB13     ------> SPI2_SCK
    PB14     ------> SPI2_MISO
    PB15     ------> SPI2_MOSI
    */
    GPIO_InitStruct.Pin = canSpiNss_Pin|canSpiSck_Pin|canSpiMiso_Pin|canSpiMosi_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI2_MspInit 1 */

  /* USER CODE END SPI2_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspDeInit 0 */

  /* USER CODE END SPI2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI2_CLK_DISABLE();

    /**SPI2 GPIO Configuration
    PB12     ------> SPI2_CS
    PB13     ------> SPI2_SCK
    PB14     ------> SPI2_MISO
    PB15     ------> SPI2_MOSI
    */
    HAL_GPIO_DeInit(GPIOB, canSpiNss_Pin|canSpiSck_Pin|canSpiMiso_Pin|canSpiMosi_Pin);

  /* USER CODE BEGIN SPI2_MspDeInit 1 */

  /* USER CODE END SPI2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
uint8_t TLE9255_Init(void)
{
    uint8_t spirxData = TLE9255_WriteReg(0x01,0x08);
    TLE9255_WriteReg(0x06, 0x84);
    return spirxData;
}

uint8_t TLE9255_SW_DISBALE_BR(void){
    uint8_t spirxData = TLE9255_WriteReg(0x06, 0x05);
    return spirxData;
}

// ** SPI Read Register **
uint8_t TLE9255_ReadReg(uint8_t reg)
{
    uint8_t txData[1] = {reg & 0x7f}; // Read command (MSB set)
    uint8_t rxData[2];

    HAL_GPIO_WritePin(canSpiNss_GPIO_Port, canSpiNss_Pin, GPIO_PIN_RESET); // CSN LOW
    Delay_us(2);
    HAL_SPI_TransmitReceive(&hspi2, txData, rxData, 2, HAL_MAX_DELAY);
    Delay_us(2);
    HAL_GPIO_WritePin(canSpiNss_GPIO_Port, canSpiNss_Pin, GPIO_PIN_SET); // CSN HIGH
    char response[60];
    memset(response, 0, sizeof(response));
    snprintf(response, sizeof(response), "SPI Read function Complete statusInfo: %d, Value: %d\n", rxData[0], rxData[1]);
    HAL_UART_Transmit(&huart3, (uint8_t*)response, strlen(response), HAL_MAX_DELAY);
    //HAL_UART_Transmit(&huart1, (uint8_t*)response, strlen(response), HAL_MAX_DELAY);
    return rxData[1];
}

// ** SPI Write Register **
uint8_t TLE9255_WriteReg(uint8_t reg, uint8_t value)
{
    uint8_t txData[2] = { reg | 0x80, value }; // Write command (MSB cleared)
    uint8_t rxData[2];

    HAL_GPIO_WritePin(canSpiNss_GPIO_Port, canSpiNss_Pin, GPIO_PIN_RESET); // CSN LOW
    Delay_us(2);
    HAL_SPI_Transmit(&hspi2, txData, 2, HAL_MAX_DELAY);
    Delay_us(2);
    HAL_GPIO_WritePin(canSpiNss_GPIO_Port, canSpiNss_Pin, GPIO_PIN_SET); // CSN HIGH
    Delay_us(5);
    txData[0] = reg & 0x7f;
    HAL_GPIO_WritePin(canSpiNss_GPIO_Port, canSpiNss_Pin, GPIO_PIN_RESET); // CSN LOW
    Delay_us(2);
    HAL_SPI_TransmitReceive(&hspi2, txData, rxData, 2, HAL_MAX_DELAY);
    Delay_us(2);
    HAL_GPIO_WritePin(canSpiNss_GPIO_Port, canSpiNss_Pin, GPIO_PIN_SET);
    char response[50];
    memset(response, 0, sizeof(response));
    snprintf(response, sizeof(response), "SPI Write Complete : %d, %d\t\n", rxData[0], rxData[1]);
    HAL_UART_Transmit(&huart3, (uint8_t*)response, strlen(response), HAL_MAX_DELAY);
    //HAL_UART_Transmit(&huart1, (uint8_t*)response, strlen(response), HAL_MAX_DELAY);

    return rxData[1];
}
/* USER CODE END 1 */
