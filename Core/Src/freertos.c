/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can.h"
#include "adc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STEP_ANGLE 1.8f // Step angle in degrees (1.8° for a 200-step motor)
#define STEPS_PER_REVOLUTION (360.0f / STEP_ANGLE) // Steps per full rotation
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
float target_angle = 90; // Target angle for the servo motor
int servo_move_flag = 0; // Flag to indicate when to move the servo
uint8_t servo_direction = 0; // Direction of rotation (0 = reverse, 1 = forward)
extern int sprayerSystemDataFlag;
extern uint8_t RxData[8];
extern uint8_t TxData[8];
int motorTopPosition = 0;
int motorBasePosition = 0;
int pumpStatus = 0;
int blowerStatus = 0;
extern CAN_TxHeaderTypeDef TxHeader;
extern int isADCFinished;
int count = 0;
uint16_t adc_data[7];
float temperature = 0.0f;
float voltage_5v = 0.0f;
float voltage_3v3 = 0.0f;


osThreadId monitoringTaskHandle;
osThreadId motorControlTaskHandle;
osThreadId canTaskHandle;
osThreadId sensorTaskHandle;
osThreadId switchMonitoringTaskHandle;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartMonitoringTask(void const * argument);
void StartMotorControlTask(void const * argument);
void StartCanTask(void const * argument);
void StartSensorTask(void const * argument);
void StartSwitchMonitoringTask(void const * argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  osThreadDef(monitoringTask, StartMonitoringTask, osPriorityNormal, 0, 128);
  monitoringTaskHandle = osThreadCreate(osThread(monitoringTask), NULL);
  osThreadDef(motorControlTask, StartMotorControlTask, osPriorityNormal, 0, 128);
  motorControlTaskHandle = osThreadCreate(osThread(motorControlTask), NULL);
  osThreadDef(canTask, StartCanTask, osPriorityNormal, 0, 128);
  canTaskHandle = osThreadCreate(osThread(canTask), NULL);
  osThreadDef(sensorTask, StartSensorTask, osPriorityNormal, 0, 128);
  sensorTaskHandle = osThreadCreate(osThread(sensorTask), NULL);
  osThreadDef(switchMonitoringTask, StartSwitchMonitoringTask, osPriorityAboveNormal, 0, 128);
  switchMonitoringTaskHandle = osThreadCreate(osThread(switchMonitoringTask), NULL);
  if (switchMonitoringTaskHandle == NULL) {
      Error_Handler(); // Handle task creation error
  }
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(500);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void SendFeedbackDataOverCAN(void) {
  uint8_t feedbackData[8];
  // Pack feedback data into CAN message
  feedbackData[0] = (uint8_t)(motorBasePosition >> 8); // High byte of motor base position
  feedbackData[1] = (uint8_t)(motorBasePosition & 0xFF); // Low byte of motor base position
  feedbackData[2] = (uint8_t)(motorTopPosition >> 8); // High byte of motor top position
  feedbackData[3] = (uint8_t)(motorTopPosition & 0xFF); // Low byte of motor top position
  feedbackData[4] = (uint8_t)pumpStatus; // Pump status
  feedbackData[5] = (uint8_t)blowerStatus; // Blower status
  feedbackData[6] = 0; // Reserved
  feedbackData[7] = 0; // Reserved

  // Send CAN message
  CAN_TxHeaderTypeDef TxHeader;
  uint32_t TxMailbox;
  TxHeader.StdId = 0x00000764;
  TxHeader.ExtId = 0;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.DLC = 8; // Data length code (8 bytes)

  if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, feedbackData, &TxMailbox) != HAL_OK) {
    // Transmission Error
    Error_Handler();
  }
}

void SendADCDataOverCAN(void) {
  uint8_t adcData[8];
  // Pack ADC data into CAN message
  adcData[0] = (uint8_t)(temperature * 10); // Temperature (scaled by 10)
  adcData[1] = (uint8_t)(voltage_5v * 100); // 5V voltage (scaled by 100)
  adcData[2] = (uint8_t)(voltage_3v3 * 100); // 3.3V voltage (scaled by 100)
  adcData[3] = 0; // Reserved
  adcData[4] = 0; // Reserved
  adcData[5] = 0; // Reserved
  adcData[6] = 0; // Reserved
  adcData[7] = 0; // Reserved

  // Send CAN message
  CAN_TxHeaderTypeDef TxHeader;
  uint32_t TxMailbox;
  TxHeader.StdId = 0x00000765;
  TxHeader.ExtId = 0;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.DLC = 8; // Data length code (8 bytes)

  if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, adcData, &TxMailbox) != HAL_OK) {
    // Transmission Error
    Error_Handler();
  }
}

void processADCData(void) {
  // Example: Convert ADC values to temperature, 5V, and 3.3V
  temperature = (float)adc_data[0] * 0.1f; // Example conversion
  voltage_5v = (float)adc_data[1] * 0.01f; // Example conversion
  voltage_3v3 = (float)adc_data[2] * 0.01f; // Example conversion
}

void StartMonitoringTask(void const * argument)
{
  HAL_ADC_Start_DMA(&hadc1, &adc_data, 7);
  /* Infinite loop */
  for(;;)
  {
	if(isADCFinished == 1){
		isADCFinished = 0;
		processADCData();
		HAL_ADC_Start_DMA(&hadc1, &adc_data, 7);
	}

    osDelay(500);
  }
}

void StartMotorControlTask(void const * argument)
{
  /* Infinite loop */
  for(;;)
  {
	  if (servo_move_flag == 1)
	  {
		  servo_move_flag = 0; // Reset the flag
		  uint32_t num_steps = (uint32_t)(motorTopPosition / STEP_ANGLE);

		   // Move the motor to the target angle
		  Generate_Steps(num_steps, 1);
	   }
    osDelay(500);
  }
}

void StartCanTask(void const * argument)
{
  HAL_CAN_Start(&hcan1);
  //Activate the notification
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  /* Infinite loop */
  for(;;)
  {
	if(sprayerSystemDataFlag == 1)
	{
		motorBasePosition = (RxData[0]<<8) | RxData[1];
		motorTopPosition = (RxData[2] << 8) | RxData[3];
		pumpStatus = RxData[4];
		blowerStatus = RxData[5];
		servo_move_flag = 1;
		sprayerSystemDataFlag = 0;
	}

    static uint32_t lastFeedbackTime = 0;
    if (osKernelSysTick() - lastFeedbackTime >= 30000) {
      SendFeedbackDataOverCAN();
      lastFeedbackTime = osKernelSysTick();
    }
	static uint32_t lastADCTime = 0;
	if (osKernelSysTick() - lastADCTime >= 30000){
		SendADCDataOverCAN();
		lastADCTime = osKernelSysTick();
	}
    osDelay(500);
  }
}

void StartSensorTask(void const * argument)
{
  /* Infinite loop */
  for(;;)
  {
    osDelay(500);
  }
}

void StartSwitchMonitoringTask(void const * argument)
{
  uint8_t switchData[8] = {0}; // CAN message data buffer
  uint32_t lastSwitchTime = osKernelSysTick(); // Track last transmission time

  for (;;)
  {
    // Read GPIO states
    uint8_t systemSwitch = HAL_GPIO_ReadPin(systemSwitch_GPIO_Port, systemSwitch_Pin);
    uint8_t selectorBypass = HAL_GPIO_ReadPin(bypassSwitch_GPIO_Port, bypassSwitch_Pin);
//    uint8_t emergencySwitch = HAL_GPIO_ReadPin(emergencySwitch, EMERGENCY_SWITCH_PIN);
    uint8_t dipS1 = HAL_GPIO_ReadPin(dipSwitch1_GPIO_Port, dipSwitch1_Pin);
    uint8_t dipS2 = HAL_GPIO_ReadPin(dipSwitch2_GPIO_Port, dipSwitch2_Pin);
    uint8_t alarmStatusBase = HAL_GPIO_ReadPin(alarmSigBase_GPIO_Port, alarmSigBase_Pin);
    uint8_t alarmStatusTop = HAL_GPIO_ReadPin(alarmSigTop_GPIO_Port, alarmSigTop_Pin);

    // Pack switch states into CAN message
    switchData[0] = systemSwitch;
    switchData[1] = selectorBypass;
    switchData[2] = 0; //Reserved Emergency Switch
    switchData[3] = dipS1;
    switchData[4] = dipS2;
    switchData[5] = alarmStatusBase;
    switchData[6] = alarmStatusTop;
    switchData[7] = 0; // Reserved

    // Send CAN message every 1 second
    if (osKernelSysTick() - lastSwitchTime >= 5000) // Check if 1 second has passed
    {
      CAN_TxHeaderTypeDef TxHeader;
      uint32_t TxMailbox;
      TxHeader.StdId = 0x766; // CAN ID for switch monitoring data
      TxHeader.ExtId = 0;
      TxHeader.RTR = CAN_RTR_DATA;
      TxHeader.IDE = CAN_ID_STD;
      TxHeader.DLC = 8; // Data length code (8 bytes)

      if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, switchData, &TxMailbox) != HAL_OK)
      {
        Error_Handler(); // Handle transmission error
      }

      lastSwitchTime = osKernelSysTick(); // Update last transmission time
    }

    osDelay(500); // Delay to avoid busy-waiting
  }
}

/* USER CODE END Application */
