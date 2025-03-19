/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "test.h"
#include "string.h"
extern volatile uint8_t data_received;
/* USER CODE END 0 */

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
//  if (HAL_LIN_Init(&huart3, UART_LINBREAKDETECTLENGTH_10B) != HAL_OK)
//  {
//    Error_Handler();
//  }
  /* USER CODE BEGIN USART3_Init 2 */
  //__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE); // Enable IDLE line detection
  /* USER CODE END USART3_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART3;
    PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PC4     ------> USART3_TX
    PC5     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* USART3 DMA Init */
    /* USART3_RX Init */
    hdma_usart3_rx.Instance = DMA1_Channel3;
    hdma_usart3_rx.Init.Request = DMA_REQUEST_2;
    hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart3_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart3_rx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    if (HAL_DMA_Init(&hdma_usart3_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart3_rx);

    /* USART3_TX Init */
    hdma_usart3_tx.Instance = DMA1_Channel2;
    hdma_usart3_tx.Init.Request = DMA_REQUEST_2;
    hdma_usart3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart3_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart3_tx.Init.Mode = DMA_NORMAL;
    hdma_usart3_tx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    if (HAL_DMA_Init(&hdma_usart3_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart3_tx);

    /* USART3 interrupt Init */
    HAL_NVIC_SetPriority(USART3_IRQn, 5, 1);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PC4     ------> USART3_TX
    PC5     ------> USART3_RX
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_4|GPIO_PIN_5);

    /* USART3 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* USART3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void process_command(char * cmd){
	char response[50];
	if(strncmp(cmd, "LED_TEST", 8) == 0){
		snprintf(response, sizeof(response), "LED_Test_Received");
		HAL_UART_Transmit_DMA(&huart3, (uint8_t*)response, strlen(response));
		blinkLEDTest();
	}
	if(strncmp(cmd, "CAN_TEST", 8) == 0){
		snprintf(response, sizeof(response), "CAN_Test_Received");
		HAL_UART_Transmit_DMA(&huart3, (uint8_t*)response, strlen(response));
		send_can_data();
	}
	if(strncmp(cmd, "PWM_TEST", 8) == 0){
		snprintf(response, sizeof(response), "MOTOR_Test_Received");
		HAL_UART_Transmit_DMA(&huart3, (uint8_t*)response, strlen(response));
		generateMotorPWM();
	}
	if(strncmp(cmd, "SWI_TEST", 8) == 0){
		snprintf(response, sizeof(response), "SWITCH_Test_Received");
		HAL_UART_Transmit_DMA(&huart3, (uint8_t*)response, strlen(response));
		readSwitchTest();
	}
	if(strncmp(cmd, "SEN_TEST", 8)== 0){
		snprintf(response, sizeof(response), "SENSOR_Test_Received");
		HAL_UART_Transmit_DMA(&huart3, (uint8_t*)response, strlen(response));
		getSensorValues();
	}
	if(strncmp(cmd, "VAL_TEST", 8) == 0){
		snprintf(response, sizeof(response), "VALVE_Test_Received");
		HAL_UART_Transmit_DMA(&huart3, (uint8_t*)response, strlen(response));
		valveTest();
	}
	if(strncmp(cmd, "ADC_TEST", 8) == 0){
		snprintf(response, sizeof(response), "ADC_Test_Received");
		HAL_UART_Transmit_DMA(&huart3, (uint8_t*)response, strlen(response));
		getADCValues();
	}
	if(strncmp(cmd, "LIN_TEST", 8) == 0){
		snprintf(response, sizeof(response), "LIN_Test_Received");
		HAL_UART_Transmit_DMA(&huart3, (uint8_t*)response, strlen(response));
		lin_communication();
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART3) {
        data_received = 1;
    }
}

uint8_t pid_Calc(uint8_t ID){
	if(ID > 0x3F) Error_Handler();
	uint8_t IDBuf[6];
	for(int i=0; i<6; i++){
		IDBuf[i] = (ID>>i)&0x01;
	}
	uint8_t P0 = (IDBuf[0]^IDBuf[1]^IDBuf[2]^IDBuf[4])&0x01;
	uint8_t P1 = ~((IDBuf[1]^IDBuf[3]^IDBuf[4]^IDBuf[5])&0x01);
	ID = ID | (P0<<6) | (P1<<7);
	return ID;
}

uint8_t checksum_Calc(uint8_t PID, uint8_t *data, uint8_t size){
	uint8_t buffer[size+2];
	uint16_t sum=0;
	buffer[0] = PID;
	for(int i=0; i<size; i++){
		buffer[i+1] = data[i];
	}
	for(int i=0;i>size;i++){
		sum = sum+buffer[i];
		if(sum > 0xff) sum = sum - 0xff;
	}
	sum = 0xff - sum;
	return sum;
}

/* USER CODE END 1 */
