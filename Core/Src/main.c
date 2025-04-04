/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "string.h"
#include "stdio.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STEP_ANGLE 1.8f
#define BOARDID 0x00000121
#define EEPROM_I2C_ADDR  0xA0
#define VREF 3.3
#define SCALE_FACTOR 2.0
#define BETA 3950
#define R_FIXED 10000.0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart3;

CAN_RxHeaderTypeDef RxHeader;

uint8_t RxData[8];

uint32_t adc_values[3];

/* USER CODE BEGIN PV */
void send_can_data(uint32_t Header, int ack);
uint8_t TLE9255_Readreg(uint8_t reg);
uint8_t TLE9255_WriteReg(uint8_t reg, uint8_t value);
void blink_led(int num);
uint8_t checksum_Calc(uint8_t PID, uint8_t *data, uint8_t size);
uint8_t pid_Calc(uint8_t ID);
void lin_communication();
void EEPROM_Read(uint16_t memAddr, uint8_t *buffer, uint16_t len);
void EEPROM_Write(uint16_t memAddr, uint8_t *data, uint16_t len);
void Generate_Steps(uint32_t num_steps, uint8_t direction);
void TLE9255_Init();
void startSolenoidValveTest();
float temperature_in_celcius(uint32_t adc_value);
float voltage_montior_5v(uint32_t adc_value);
float voltage_montior_3v(uint32_t adc_value);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  TLE9255_Init();
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
  HAL_Delay(500);
  HAL_CAN_Start(&hcan1);
//  lin_communication();
  uint8_t writeData[4] = {
      (BOARDID >> 24) & 0xFF,
      (BOARDID >> 16) & 0xFF,
      (BOARDID >> 8)  & 0xFF,
      (BOARDID & 0xFF)
  };
  uint8_t readData[4] = {0};

  EEPROM_Write(0x0000, writeData, sizeof(writeData));
  HAL_Delay(10);  // Ensure write completion

  HAL_Delay(1000);
//  for(int i =0; i< 1; i++){
  send_can_data(BOARDID,0);
//	  HAL_Delay(1000);
//  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
	  HAL_Delay(100);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
	  lin_communication();
	  HAL_Delay(100);
	  HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData);
	  if(RxHeader.DLC == 8){
		  if(RxHeader.StdId == 0x00000001)
		  {
			  if(RxData[0] == 1)
			  {
				  blink_led(1);
				  send_can_data(RxHeader.StdId, 1);
			  }
			  else if(RxData[0] == 2)
			  {
				  blink_led(2);
			  	  send_can_data(RxHeader.StdId, 1);
			  }
		  }
		  if(RxHeader.StdId == 0x00000002){
			  if(RxData[0] == 1){
				  while(HAL_GPIO_ReadPin(proxMaxTop_GPIO_Port, proxMaxTop_Pin) == 1);
			  }
			  else if(RxData[0] == 2){
				  while(HAL_GPIO_ReadPin(proxMinTop_GPIO_Port,proxMinTop_Pin) == 1);
			  }
			  else if(RxData[0] == 3){
				  while(HAL_GPIO_ReadPin(proxMinBase_GPIO_Port,proxMinBase_Pin) == 1);
			  }
			  else if(RxData[0] == 4){
				  while(HAL_GPIO_ReadPin(proxMaxBase_GPIO_Port,proxMaxBase_Pin) == 1);
			  }
			  send_can_data(RxHeader.StdId, 1);
		  }
		  if(RxHeader.StdId == 0x00000003){
			  startSolenoidValveTest();
			  send_can_data(RxHeader.StdId, 1);
		  }
		  if(RxHeader.StdId == 0x00000004){
			  uint16_t angle_scaled = (RxData[1]<<8)|RxData[0];
			  int direction = RxData[2];
			  float angle = angle_scaled/100.0;
		      uint32_t num_steps = (uint32_t)(angle / STEP_ANGLE);
		      for (int i = 0; i < 5; i++)
		      {
		          Generate_Steps(num_steps, direction);
		          HAL_Delay(1000);
		      }
			  send_can_data(RxHeader.StdId, 1);
		  }
		  if(RxHeader.StdId == 0x00000005){
			  uint8_t TxData[8];
			  Read_ADC_MultipleChannels();
			  float temperature = temperature_in_celcius(adc_values[0]);
			  float voltage_5v = voltage_montior_5v(adc_values[1]);
			  float voltage_3v = voltage_montior_3v(adc_values[2]);
				int16_t temp_scaled = (int16_t)(temperature * 100);  // Scale to 0.01°C precision
				uint16_t volt5_scaled = (uint16_t)(voltage_5v * 1000);  // Scale to 1mV precision
				uint16_t volt3_scaled = (uint16_t)(voltage_3v * 1000);

				TxData[0] = (temp_scaled >> 8) & 0xFF;
				TxData[1] = temp_scaled & 0xFF;
				TxData[2] = (volt5_scaled >> 8) & 0xFF;
				TxData[3] = volt5_scaled & 0xFF;
				TxData[4] = (volt3_scaled >> 8) & 0xFF;
				TxData[5] = volt3_scaled & 0xFF;
				TxData[6] = 0x00;  // Reserved Byte
				TxData[7] = 0x00;  // Reserved Byte

				CAN_TxHeaderTypeDef TxHeader;
				uint32_t TxMailbox;
				TxHeader.StdId = RxHeader.StdId;
				TxHeader.ExtId = 0;
				TxHeader.RTR = CAN_RTR_DATA;
				TxHeader.IDE = CAN_ID_STD;
				TxHeader.DLC = 8; // Data length code (8 bytes)

				if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
				// Transmission Error
				Error_Handler();
				}
		  }
		  if(RxHeader.StdId == 0x00000006){
			  EEPROM_Read(0x0000, readData, sizeof(readData));
			  HAL_Delay(10);
			  send_can_data((readData[0] << 24)|(readData[1] << 16)|(readData[2] << 8)|readData[3] ,0);
		  }
		  if(RxHeader.StdId == 0x00000007){
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
			  lin_communication();
		  }
		  if(RxHeader.StdId == 0x00000008){
			  while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == 1);
			  while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == 1);
			  send_can_data(RxHeader.StdId, 1);
		  }
		  RxHeader.DLC = 0;
		  RxHeader.StdId = 0x00000000;
		  memset(RxData, 0, sizeof(RxData));
	  }
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  CAN_FilterTypeDef canfilterconfig;

  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 0;
  canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  canfilterconfig.FilterIdHigh = 0x0000;
  canfilterconfig.FilterIdLow = 0x0000;
  canfilterconfig.FilterMaskIdHigh = 0x0000;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig.SlaveStartFilterBank = 14;

  HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);
  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00F12981;
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

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 199;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 10;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
   sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
   if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
   {
     Error_Handler();
   }
   sConfigOC.OCMode = TIM_OCMODE_PWM1;
   sConfigOC.Pulse = 10;
   sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
   sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
   if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
   {
     Error_Handler();
   }
   if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
   {
     Error_Handler();
   }
   /* USER CODE BEGIN TIM2_Init 2 */

   /* USER CODE END TIM2_Init 2 */
   HAL_TIM_MspPostInit(&htim2);

  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}
/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 19200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_LIN_Init(&huart3, UART_LINBREAKDETECTLENGTH_10B) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, topDir_Pin|baseDir_Pin|led2_Pin|led1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ENTop_Pin|EN_Base_Pin|GPIO_PIN_12|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, solenoidTop_Pin|solenoidBase_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : topDir_Pin baseDir_Pin led2_Pin led1_Pin */
  GPIO_InitStruct.Pin = topDir_Pin|baseDir_Pin|led2_Pin|led1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : alarmTop_Pin alarmBase_Pin proxMinTop_Pin proxMaxTop_Pin
                           dips1_Pin */
  GPIO_InitStruct.Pin = alarmTop_Pin|alarmBase_Pin|proxMinTop_Pin|proxMaxTop_Pin
                          |dips1_Pin|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : proxMinBase_Pin proxMaxBase_Pin systemSw_Pin */
  GPIO_InitStruct.Pin = proxMinBase_Pin|proxMaxBase_Pin|systemSw_Pin|GPIO_PIN_5|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : ENTop_Pin EN_Base_Pin PB12 */
  GPIO_InitStruct.Pin = ENTop_Pin|EN_Base_Pin|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : solenoidTop_Pin solenoidBase_Pin */
  GPIO_InitStruct.Pin = solenoidTop_Pin|solenoidBase_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : byps_Pin dips2_Pin */
  GPIO_InitStruct.Pin = byps_Pin|dips2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//void prepare_CAN_Data(float temperature, float voltage_5v, float voltage_3v, uint8_t *TxData) {
//    int16_t temp_scaled = (int16_t)(temperature * 100);  // Scale to 0.01°C precision
//    uint16_t volt5_scaled = (uint16_t)(voltage_5v * 1000);  // Scale to 1mV precision
//    uint16_t volt3_scaled = (uint16_t)(voltage_3v * 1000);
//
//    TxData[0] = (temp_scaled >> 8) & 0xFF;
//    TxData[1] = temp_scaled & 0xFF;
//    TxData[2] = (volt5_scaled >> 8) & 0xFF;
//    TxData[3] = volt5_scaled & 0xFF;
//    TxData[4] = (volt3_scaled >> 8) & 0xFF;
//    TxData[5] = volt3_scaled & 0xFF;
//    TxData[6] = 0x00;  // Reserved Byte
//    TxData[7] = 0x00;  // Reserved Byte
//}

void Read_ADC_MultipleChannels() {
    for (uint8_t i = 0; i < 3; i++) {
        HAL_ADC_Start(&hadc1);  // Start ADC conversion
        HAL_ADC_PollForConversion(&hadc1, 10);  // Wait for conversion
        adc_values[i] = HAL_ADC_GetValue(&hadc1);  // Read ADC value
        HAL_ADC_Stop(&hadc1);  // Stop ADC
    }
}

float temperature_in_celcius(uint32_t adc_value){
	float voltage, resistance, temperature;
    voltage = (adc_value * VREF) / 4095.0;

    resistance = R_FIXED * (voltage / (VREF - voltage));

    float A = 0.001129148, B = 0.000234125, C = 0.0000000876741;
    float logR = log(resistance);
    float temp_kelvin = 1.0 / (A + (B * logR) + (C * logR * logR * logR));
    float temp_celsius = temp_kelvin - 273.15;

    return temp_celsius;
}

float voltage_montior_5v(uint32_t adc_value){
    // Convert ADC value to voltage
    float adc_voltage = (adc_value * VREF) / 4095.0;
    float voltage = adc_voltage * SCALE_FACTOR;  // Scale back to original 5V

    return voltage;
}

float voltage_montior_3v(uint32_t adc_value){
    // Convert ADC value to voltage
    float adc_voltage = (adc_value * VREF) / 4095.0;

    return adc_voltage;
}

void lin_communication(){
	uint8_t TxData_Lin[20];
	TxData_Lin[0] = 0x55; //sync field
	TxData_Lin[1] = pid_Calc(0x34); //protectedID
	for(int i=0; i<8; i++){
		TxData_Lin[i+2] = i;
	}
	TxData_Lin[10] = checksum_Calc(TxData_Lin[1], TxData_Lin+2, 8);
	HAL_LIN_SendBreak(&huart3);
	HAL_UART_Transmit(&huart3, TxData_Lin, 11, 1000);
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

void EEPROM_Write(uint16_t memAddr, uint8_t *data, uint16_t len){
    uint8_t buffer[len + 2];

    buffer[0] = (memAddr >> 8) & 0xFF;  // High byte of address
    buffer[1] = memAddr & 0xFF;         // Low byte of address

    memcpy(&buffer[2], data, len);  // Copy data to buffer

    HAL_I2C_Master_Transmit(&hi2c1, EEPROM_I2C_ADDR, buffer, len + 2, HAL_MAX_DELAY);
    HAL_Delay(5);  // EEPROM write cycle time (max 5ms)
}

void EEPROM_Read(uint16_t memAddr, uint8_t *buffer, uint16_t len){
    uint8_t addr[2];

    addr[0] = (memAddr >> 8) & 0xFF;  // High byte of address
    addr[1] = memAddr & 0xFF;         // Low byte of address

    HAL_I2C_Master_Transmit(&hi2c1, EEPROM_I2C_ADDR, addr, 2, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1, EEPROM_I2C_ADDR, buffer, len, HAL_MAX_DELAY);
}

void startSolenoidValveTest(){
    for(int i = 0; i < 2; i++)
    {
        HAL_GPIO_WritePin(solenoidBase_GPIO_Port, solenoidBase_Pin, GPIO_PIN_SET);
        HAL_Delay(1000);
        HAL_GPIO_WritePin(solenoidBase_GPIO_Port, solenoidBase_Pin, GPIO_PIN_RESET);
        HAL_Delay(1000);
    }
    for(int i = 0; i < 2; i++)
    {

        HAL_GPIO_WritePin(solenoidTop_GPIO_Port, solenoidTop_Pin, GPIO_PIN_SET);
        HAL_Delay(1000);
        HAL_GPIO_WritePin(solenoidTop_GPIO_Port, solenoidTop_Pin, GPIO_PIN_RESET);
        HAL_Delay(1000);
    }
}
void Set_Motor_Direction(uint8_t direction) {
    if (direction == 1) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET); // Forward
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET); // Forward
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
    }
    HAL_Delay(1);
}

void Enable_Motor(uint8_t enable) {
    if (enable == 1) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET); // Enable motor
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET); // Enable motor
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET); // Enable motor
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET); // Enable motor
    }
}

void Generate_Steps(uint32_t num_steps, uint8_t direction){
    // Set the direction
    Set_Motor_Direction(direction);

    // Enable the motor
    Enable_Motor(0);

    // Enable PWM
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

    uint32_t move_time = (num_steps * 10) / 1000; // Convert step time to milliseconds
    HAL_Delay(1);

    // Disable PWM
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);

//    Enable_Motor(1);
}

void TLE9255_Init(){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
	  TLE9255_WriteReg(0x01, 0x08);
	  HAL_Delay(1);
	  TLE9255_WriteReg(0x06, 0x05);
	  HAL_Delay(1);
	  TLE9255_WriteReg(0x07, 0x08);
	  HAL_Delay(1);
	  TLE9255_WriteReg(0x08, 0x80);
	  HAL_Delay(1);
	  TLE9255_WriteReg(0x09, 0x00);
	  HAL_Delay(1);
	  TLE9255_WriteReg(0x0A, 0x00);
	  HAL_Delay(1);
	  TLE9255_WriteReg(0x0B, 0x00);
	  HAL_Delay(1);
	  TLE9255_WriteReg(0x0C, 0x00);
	  HAL_Delay(1);
	  TLE9255_WriteReg(0x0D, 0x00);
	  HAL_Delay(1);
	  TLE9255_WriteReg(0x0E, 0x00);
	  HAL_Delay(1);
	  TLE9255_WriteReg(0x0f, 0x08);
	  HAL_Delay(1);

}

void blink_led(int num)
{
	if(num == 1)
	{
		for(int i = 0; i< 5; i++){
			HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, GPIO_PIN_SET);
			HAL_Delay(500);
			HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, GPIO_PIN_RESET);
			HAL_Delay(500);
		}
	}
	if(num == 2)
	{
		for(int i = 0; i< 5; i++){
			HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin, GPIO_PIN_SET);
			HAL_Delay(500);
			HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin, GPIO_PIN_RESET);
			HAL_Delay(500);
		}
	}


}



void send_can_data(uint32_t Header, int ack){
	uint8_t testData[8];
	if(ack == 0){
		  testData[0] = 0x00;
		  testData[1] = 0x00;
		  testData[2] = 0x00;
		  testData[3] = 0x00;
		  testData[4] = 0x00;
		  testData[5] = 0x00;
		  testData[6] = 0x00;
		  testData[7] = 0x00;
	}
	if(ack == 1){
		  testData[0] = 0x01;
		  testData[1] = 0x01;
		  testData[2] = 0x01;
		  testData[3] = 0x01;
		  testData[4] = 0x01;
		  testData[5] = 0x01;
		  testData[6] = 0x01;
		  testData[7] = 0x01;
	}

  // Send CAN message
  CAN_TxHeaderTypeDef TxHeader;
  uint32_t TxMailbox;
  TxHeader.StdId = Header;
  TxHeader.ExtId = 0;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.DLC = 8; // Data length code (8 bytes)

  if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, testData, &TxMailbox) != HAL_OK) {
	// Transmission Error
	Error_Handler();
  }
}
// ** SPI Write Register **
uint8_t TLE9255_WriteReg(uint8_t reg, uint8_t value)
{
    uint8_t txData[2] = { reg | 0x80, value }; // Write command (MSB cleared)
    uint8_t rxData[2];

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // CSN LOW
    HAL_SPI_Transmit(&hspi2, txData, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); // CSN HIGH
    txData[0] = reg & 0x7f;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // CSN LOW
    HAL_SPI_TransmitReceive(&hspi2, txData, rxData, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    char response[50];
    memset(response, 0, sizeof(response));
    snprintf(response, sizeof(response), "SPI Write Complete : %02X, %02X\n", rxData[0], rxData[1]);
    HAL_UART_Transmit(&huart3, (uint8_t*)response, strlen(response), HAL_MAX_DELAY);

//    HAL_Delay(5000);
//
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // CSN LOW
//    HAL_SPI_TransmitReceive(&hspi2, txData, rxData, 2, HAL_MAX_DELAY);
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
//
//    memset(response, 0, sizeof(response));
//    snprintf(response, sizeof(response), "SPI Read Data Values : %02X, %02X\t\n", rxData[0], rxData[1]);
//    HAL_UART_Transmit(&huart3, (uint8_t*)response, strlen(response), HAL_MAX_DELAY);

    return rxData[1];
}

uint8_t TLE9255_Readreg(uint8_t reg){
	char response[60];
	uint8_t txData[2] = {reg & 0x7f};
	uint8_t rxData[2];
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // CSN LOW
    HAL_SPI_TransmitReceive(&hspi2, txData, rxData, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

    memset(response, 0, sizeof(response));
    snprintf(response, sizeof(response), "SPI Read Data reg: %02X, StatusInfo: %02X, Value: %02X\t\n", reg, rxData[0], rxData[1]);
    HAL_UART_Transmit(&huart3, (uint8_t*)response, strlen(response), HAL_MAX_DELAY);

    return rxData[1];
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
