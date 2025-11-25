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
/* Definitions for MotorTask */
osThreadId_t MotorTaskHandle;
const osThreadAttr_t MotorTask_attributes = {
  .name = "MotorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for modbusTassk */
osThreadId_t modbusTasskHandle;
const osThreadAttr_t modbusTassk_attributes = {
  .name = "modbusTassk",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ledAndRelayTask */
osThreadId_t ledAndRelayTaskHandle;
const osThreadAttr_t ledAndRelayTask_attributes = {
  .name = "ledAndRelayTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for uarrt3RxTask */
osThreadId_t uarrt3RxTaskHandle;
const osThreadAttr_t uarrt3RxTask_attributes = {
  .name = "uarrt3RxTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartMotorTask(void *argument);
void StartMosbusTask(void *argument);
void StartLedAndRelayTask(void *argument);
void StartUart3RxTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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
  /* creation of MotorTask */
  MotorTaskHandle = osThreadNew(StartMotorTask, NULL, &MotorTask_attributes);

  /* creation of modbusTassk */
  modbusTasskHandle = osThreadNew(StartMosbusTask, NULL, &modbusTassk_attributes);

  /* creation of ledAndRelayTask */
  ledAndRelayTaskHandle = osThreadNew(StartLedAndRelayTask, NULL, &ledAndRelayTask_attributes);

  /* creation of uarrt3RxTask */
  uarrt3RxTaskHandle = osThreadNew(StartUart3RxTask, NULL, &uarrt3RxTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartMotorTask */
/**
  * @brief  Function implementing the MotorTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartMotorTask */
void StartMotorTask(void *argument)
{
  /* USER CODE BEGIN StartMotorTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartMotorTask */
}

/* USER CODE BEGIN Header_StartMosbusTask */
/**
* @brief Function implementing the modbusTassk thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMosbusTask */
void StartMosbusTask(void *argument)
{
  /* USER CODE BEGIN StartMosbusTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartMosbusTask */
}

/* USER CODE BEGIN Header_StartLedAndRelayTask */
/**
* @brief Function implementing the ledAndRelayTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLedAndRelayTask */
void StartLedAndRelayTask(void *argument)
{
  /* USER CODE BEGIN StartLedAndRelayTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartLedAndRelayTask */
}

/* USER CODE BEGIN Header_StartUart3RxTask */
/**
* @brief Function implementing the uarrt3RxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUart3RxTask */
void StartUart3RxTask(void *argument)
{
  /* USER CODE BEGIN StartUart3RxTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartUart3RxTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

