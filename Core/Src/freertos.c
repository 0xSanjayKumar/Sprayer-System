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
#include "can.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern int sprayerSystemDataFlag;
extern uint8_t RxData[8];
extern uint8_t TxData[8];
int motorTopPosition = 0;
int motorBasePosition = 0;
int pumpStatus = 0;
int blowerStatus = 0;
extern CAN_TxHeaderTypeDef TxHeader;

osThreadId monitoringTaskHandle;
osThreadId motorControlTaskHandle;
osThreadId canTaskHandle;
osThreadId sensorTaskHandle;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartMonitoringTask(void const * argument);
void StartMotorControlTask(void const * argument);
void StartCanTask(void const * argument);
void StartSensorTask(void const * argument);
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
  monitoringTaskHandle = osThreadCreate(osThread(motorControlTask), NULL);
  osThreadDef(canTask, StartCanTask, osPriorityNormal, 0, 128);
  monitoringTaskHandle = osThreadCreate(osThread(canTask), NULL);
  osThreadDef(sensorTask, StartSensorTask, osPriorityNormal, 0, 128);
  monitoringTaskHandle = osThreadCreate(osThread(sensorTask), NULL);
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
void StartMonitoringTask(void const * argument)
{
  /* Infinite loop */
  for(;;)
  {
    osDelay(500);
  }
}

void StartMotorControlTask(void const * argument)
{
  /* Infinite loop */
  for(;;)
  {
    osDelay(500);
  }
}

void StartCanTask(void const * argument)
{
  /* Infinite loop */
  for(;;)
  {
	if(sprayerSystemDataFlag == 1){
		motorBasePosition = RxData[0];
		motorTopPosition = RxData[1];
		pumpStatus = RxData[2];
		blowerStatus = RxData[3];
		updateTxHeader(0x00000F64);
		TxData[0] = motorBasePosition;
		TxData[1] = motorTopPosition;
		TxData[2] = 0;
		TxData[3] = 0;
		TxData[4] = pumpStatus;
		TxData[5] = blowerStatus;
		sendCanMessage();
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

/* USER CODE END Application */
