/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* USER CODE END Variables */
osThreadId CHASSIS_TASKHandle;
osThreadId DETECT_TASKHandle;
osThreadId TRANSMISSIONHandle;
osThreadId DEBUG_TASKHandle;
osThreadId REFEREE_TASKHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Chassis_Task(void const * argument);
void Detect_Task(void const * argument);
void Transmission_Task(void const * argument);
void Debug_Task(void const * argument);
void Referee_Task(void const * argument);

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
  /* definition and creation of CHASSIS_TASK */
  osThreadDef(CHASSIS_TASK, Chassis_Task, osPriorityNormal, 0, 256);
  CHASSIS_TASKHandle = osThreadCreate(osThread(CHASSIS_TASK), NULL);

  /* definition and creation of DETECT_TASK */
  osThreadDef(DETECT_TASK, Detect_Task, osPriorityNormal, 0, 128);
  DETECT_TASKHandle = osThreadCreate(osThread(DETECT_TASK), NULL);

  /* definition and creation of TRANSMISSION */
  osThreadDef(TRANSMISSION, Transmission_Task, osPriorityNormal, 0, 128);
  TRANSMISSIONHandle = osThreadCreate(osThread(TRANSMISSION), NULL);

  /* definition and creation of DEBUG_TASK */
  osThreadDef(DEBUG_TASK, Debug_Task, osPriorityNormal, 0, 128);
  DEBUG_TASKHandle = osThreadCreate(osThread(DEBUG_TASK), NULL);

  /* definition and creation of REFEREE_TASK */
  osThreadDef(REFEREE_TASK, Referee_Task, osPriorityNormal, 0, 2048);
  REFEREE_TASKHandle = osThreadCreate(osThread(REFEREE_TASK), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_Chassis_Task */
/**
  * @brief  Function implementing the CHASSIS_TASK thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Chassis_Task */
__weak void Chassis_Task(void const * argument)
{
  /* USER CODE BEGIN Chassis_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Chassis_Task */
}

/* USER CODE BEGIN Header_Detect_Task */
/**
* @brief Function implementing the DETECT_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Detect_Task */
__weak void Detect_Task(void const * argument)
{
  /* USER CODE BEGIN Detect_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Detect_Task */
}

/* USER CODE BEGIN Header_Transmission_Task */
/**
* @brief Function implementing the TRANSMISSION thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Transmission_Task */
__weak void Transmission_Task(void const * argument)
{
  /* USER CODE BEGIN Transmission_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Transmission_Task */
}

/* USER CODE BEGIN Header_Debug_Task */
/**
* @brief Function implementing the DEBUG_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Debug_Task */
__weak void Debug_Task(void const * argument)
{
  /* USER CODE BEGIN Debug_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Debug_Task */
}

/* USER CODE BEGIN Header_Referee_Task */
/**
* @brief Function implementing the REFEREE_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Referee_Task */
__weak void Referee_Task(void const * argument)
{
  /* USER CODE BEGIN Referee_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Referee_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
