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
#include "cmsis_os.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TEST
#define RX_BUFFER_SIZE 8
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern uint8_t TxData[8];
extern uint8_t isADCFinished;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint8_t data_received = 0;
extern int sprayerSystemDataFlag;
uint8_t rx_buffer[RX_BUFFER_SIZE];
//uint8_t *tx_buffer;
//uint32_t TxMailbox;
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
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  TIM3_Init();
//  DWT_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(canWakeUp_GPIO_Port, canWakeUp_Pin, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in cmsis_os2.c) */
#ifndef TEST
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();
#endif

  /* We should never get here as control is now taken by the scheduler */
#ifdef TEST
  char response[50];
  snprintf(response, sizeof(response), "Hello to Sprayer System : %lu\n",HAL_RCC_GetPCLK1Freq());
  HAL_UART_Transmit(&huart3, (uint8_t*)response, strlen(response), HAL_MAX_DELAY);
//  HAL_UART_Transmit(&huart1, (uint8_t*)response, strlen(response), HAL_MAX_DELAY);

  for(int i = 0; i<5; i++){
	HAL_GPIO_TogglePin(led1_GPIO_Port, led1_Pin);
	HAL_GPIO_TogglePin(led2_GPIO_Port, led2_Pin);
	HAL_Delay(1000);
  }

  HAL_Delay(50);
  uint8_t readData = TLE9255_ReadReg(0x01);

  memset(response, 0, sizeof(response));
  snprintf(response, sizeof(response), "Writing Over SPI\n");
  HAL_UART_Transmit(&huart3, (uint8_t*)response, strlen(response), HAL_MAX_DELAY);
  HAL_Delay(50);
  uint8_t spiData = TLE9255_Init();
  HAL_Delay(50);
  memset(response, 0, sizeof(response));
  snprintf(response, sizeof(response), "Initializing CAN\n");
  HAL_UART_Transmit(&huart3, (uint8_t*)response, strlen(response), HAL_MAX_DELAY);
  HAL_Delay(50);
  HAL_CAN_Start(&hcan1);
  for(int i =0; i< 5; i++){
	  snprintf(response, sizeof(response), "Sending CAN Data: %d\n", i);
	  HAL_UART_Transmit(&huart3, (uint8_t*)response, strlen(response), HAL_MAX_DELAY);
	  send_can_data();
	  HAL_Delay(2000);
  }
  //HAL_UART_Receive_DMA(&huart1, (uint8_t*)rx_buffer, RX_BUFFER_SIZE);
#endif
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
#ifdef TEST
	  if(data_received == 1){
		  check_uart_data();
		  data_received = 0;
	  }
	  int val = HAL_GPIO_ReadPin(proximityMinTop_GPIO_Port, proximityMinTop_Pin);
	  if(val == 0){
		sprintf(response, "Proximity Sensor: %d\n", val);
		HAL_UART_Transmit(&huart3, (uint8_t*)response, strlen(response), HAL_MAX_DELAY);
	  }
#endif
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
//void SystemClock_Config(void)
//{
//  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
//
//  /** Configure the main internal regulator output voltage
//  */
//  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Initializes the RCC Oscillators according to the specified parameters
//  * in the RCC_OscInitTypeDef structure.
//  */
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
//  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
//  RCC_OscInitStruct.HSICalibrationValue = 64;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
//  RCC_OscInitStruct.PLL.PLLM = 1;
//  RCC_OscInitStruct.PLL.PLLN = 16;
//  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
//  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
//  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
//  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Initializes the CPU, AHB and APB buses clocks
//  */
//  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
//
//  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}

void DWT_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  // Enable DWT
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;  // Enable cycle counter
}

void Delay_ns(uint32_t ns) {
    uint32_t cycles = (ns * (SystemCoreClock / 1000000000));  // Convert ns to cycles
    uint32_t start = DWT->CYCCNT;  // Get current cycle count
    while ((DWT->CYCCNT - start) < cycles);  // Wait for the required cycles
}

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 20;
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
/* USER CODE BEGIN 4 */


void check_uart_data() {
	process_command(&rx_buffer[0]);
}



void Set_Motor_Direction(uint8_t direction) {
    if (direction == 1) {
        HAL_GPIO_WritePin(servoDirTop_GPIO_Port, servoDirTop_Pin, GPIO_PIN_SET); // Forward
        HAL_GPIO_WritePin(servoDirBase_GPIO_Port, servoDirBase_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(servoDirTop_GPIO_Port, servoDirTop_Pin, GPIO_PIN_SET); // Forward
        HAL_GPIO_WritePin(servoDirBase_GPIO_Port, servoDirBase_Pin, GPIO_PIN_SET);
    }
    HAL_Delay(1); // Wait for at least 5 µs (adjust as needed)
}

void Enable_Motor(uint8_t enable) {
    if (enable == 1) {
        HAL_GPIO_WritePin(servoEnableTop_GPIO_Port, servoEnableTop_Pin, GPIO_PIN_SET); // Enable motor
        HAL_GPIO_WritePin(servoEnableBase_GPIO_Port, servoEnableBase_Pin, GPIO_PIN_SET); // Enable motor
    } else {
        HAL_GPIO_WritePin(servoEnableTop_GPIO_Port, servoEnableTop_Pin, GPIO_PIN_RESET); // Enable motor
        HAL_GPIO_WritePin(servoEnableBase_GPIO_Port, servoEnableBase_Pin, GPIO_PIN_RESET); // Enable motor
    }
}

void Generate_Steps(uint32_t num_steps, uint8_t direction) {
    // Set the direction
    Set_Motor_Direction(direction);

    // Enable the motor
    Enable_Motor(1);

    // Enable PWM
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

    // Generate the required number of steps
    for (uint32_t i = 0; i < num_steps; i++) {
        // Wait for the pulse width (10 µs)
        HAL_Delay(1); // Adjust delay as needed
    }

    // Disable PWM
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);

    // Disable the motor (optional)
    Enable_Motor(0);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
	isADCFinished = 1;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	char response[50];
	sprintf(response, "Interrupt Triggered");
	HAL_UART_Transmit(&huart3, (uint8_t*)response, strlen(response), HAL_MAX_DELAY);
	if(GPIO_Pin == proximityMinTop_Pin){
		memset(response, 0, sizeof(response));
		int val = HAL_GPIO_ReadPin(proximityMinTop_GPIO_Port, proximityMinTop_Pin);
		sprintf(response, "Proximity Sensor: %d\n", val);
		HAL_UART_Transmit(&huart3, (uint8_t*)response, strlen(response), HAL_MAX_DELAY);
	}
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
