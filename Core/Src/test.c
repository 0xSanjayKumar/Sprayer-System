/*
 * test.c
 *
 *  Created on: Feb 10, 2025
 *      Author: Yash
 */
#include "test.h"
#include "can.h"
#include "adc.h"
#include "usart.h"
#include "stdio.h"
#include "gpio.h"

extern uint16_t adc_data[7];
extern float temperature;
extern float voltage_5v;
extern float voltage_3v3;
extern int isADCFinished;

void send_can_data(){
  uint8_t testData[8];

  testData[0] = 0x01;
  testData[1] = 0x02;
  testData[2] = 0x03;
  testData[3] = 0x04;
  testData[4] = 0x05;
  testData[5] = 0x06;
  testData[6] = 0x07;
  testData[7] = 0x08;

  // Send CAN message
  CAN_TxHeaderTypeDef TxHeader;
  uint32_t TxMailbox;
  TxHeader.StdId = 0x00000220;
  TxHeader.ExtId = 0;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.DLC = 8; // Data length code (8 bytes)

  if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, testData, &TxMailbox) != HAL_OK) {
	// Transmission Error
	Error_Handler();
  }
}

void getADCValues(){
	char response[50];
	HAL_ADC_Start_DMA(&hadc1, &adc_data, 7);
	if(isADCFinished == 1){
			isADCFinished = 0;
			processADCData();
			HAL_ADC_Start_DMA(&hadc1, &adc_data, 7);
		}
	snprintf(response, sizeof(response), "voltage3v: %0.2f, voltage5v: %0.2f, temperature: %0.2f", voltage_3v3, voltage_5v, temperature);
	HAL_UART_Transmit_DMA(&huart3, (uint8_t*)response, strlen(response));
}

void getSensorValues(){

}

void generateMotorPWM(){
	Generate_Steps(50, 1);
}

void blinkLEDTest(){
	for(int i = 0; i<10; i++){
		HAL_GPIO_TogglePin(led1_GPIO_Port, led1_Pin);
		HAL_GPIO_TogglePin(led2_GPIO_Port, led2_Pin);
		HAL_Delay(100);
	}

}

void readSwitchTest(){
	int bypassSw = 0, dip_sw1 = 0, dip_sw2 = 0;
	dip_sw1 = HAL_GPIO_ReadPin(dipSwitch1_GPIO_Port, dipSwitch1_Pin);
	dip_sw2 = HAL_GPIO_ReadPin(dipSwitch2_GPIO_Port, dipSwitch2_Pin);
	bypassSw = HAL_GPIO_ReadPin(bypassSwitch_GPIO_Port, bypassSwitch_Pin);

}

void valveTest(){
	HAL_GPIO_WritePin(solenoidTop_GPIO_Port, solenoidTop_Pin, 1);
	HAL_GPIO_WritePin(solenoidBase_GPIO_Port, solenoidBase_Pin, 1);
	HAL_Delay(500);
	HAL_GPIO_WritePin(solenoidTop_GPIO_Port, solenoidTop_Pin, 0);
	HAL_GPIO_WritePin(solenoidBase_GPIO_Port, solenoidBase_Pin, 0);
}
