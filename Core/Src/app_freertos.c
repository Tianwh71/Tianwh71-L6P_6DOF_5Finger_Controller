/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
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
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for MtParserTask */
osThreadId_t MtParserTaskHandle;
const osThreadAttr_t MtParserTask_attributes = {
  .name = "MtParserTask",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 128 * 4
};
/* Definitions for MtControlTask */
osThreadId_t MtControlTaskHandle;
const osThreadAttr_t MtControlTask_attributes = {
  .name = "MtControlTask",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 128 * 4
};
/* Definitions for TipSensorTask */
osThreadId_t TipSensorTaskHandle;
const osThreadAttr_t TipSensorTask_attributes = {
  .name = "TipSensorTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for TipRxDecodeTask */
osThreadId_t TipRxDecodeTaskHandle;
const osThreadAttr_t TipRxDecodeTask_attributes = {
  .name = "TipRxDecodeTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for TipSensorRespon */
osThreadId_t TipSensorResponHandle;
const osThreadAttr_t TipSensorRespon_attributes = {
  .name = "TipSensorRespon",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for CanAnalysisTask */
osThreadId_t CanAnalysisTaskHandle;
const osThreadAttr_t CanAnalysisTask_attributes = {
  .name = "CanAnalysisTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for UpperControlTas */
osThreadId_t UpperControlTasHandle;
const osThreadAttr_t UpperControlTas_attributes = {
  .name = "UpperControlTas",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for Tip_Usart_Queue */
osMessageQueueId_t Tip_Usart_QueueHandle;
const osMessageQueueAttr_t Tip_Usart_Queue_attributes = {
  .name = "Tip_Usart_Queue"
};
/* Definitions for Can_RX_Queue */
osMessageQueueId_t Can_RX_QueueHandle;
const osMessageQueueAttr_t Can_RX_Queue_attributes = {
  .name = "Can_RX_Queue"
};
/* Definitions for TipSensorTxSem */
osSemaphoreId_t TipSensorTxSemHandle;
const osSemaphoreAttr_t TipSensorTxSem_attributes = {
  .name = "TipSensorTxSem"
};
/* Definitions for TipSensorRxBlockSem */
osSemaphoreId_t TipSensorRxBlockSemHandle;
const osSemaphoreAttr_t TipSensorRxBlockSem_attributes = {
  .name = "TipSensorRxBlockSem"
};
/* Definitions for UpperControlSem */
osSemaphoreId_t UpperControlSemHandle;
const osSemaphoreAttr_t UpperControlSem_attributes = {
  .name = "UpperControlSem"
};
/* Definitions for TJ_UART_RX_Sem */
osSemaphoreId_t TJ_UART_RX_SemHandle;
const osSemaphoreAttr_t TJ_UART_RX_Sem_attributes = {
  .name = "TJ_UART_RX_Sem"
};
/* Definitions for TJ_Trans_Sem */
osSemaphoreId_t TJ_Trans_SemHandle;
const osSemaphoreAttr_t TJ_Trans_Sem_attributes = {
  .name = "TJ_Trans_Sem"
};
/* Definitions for DecodeEvent */
osEventFlagsId_t DecodeEventHandle;
const osEventFlagsAttr_t DecodeEvent_attributes = {
  .name = "DecodeEvent"
};
/* Definitions for UART_ReceivedEvent */
osEventFlagsId_t UART_ReceivedEventHandle;
const osEventFlagsAttr_t UART_ReceivedEvent_attributes = {
  .name = "UART_ReceivedEvent"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void MotorDataParserTaskFun(void *argument);
void MotorControlTaskFun(void *argument);
void StartTipSensorTask(void *argument);
void StartTipRxDecodeTask(void *argument);
void StartTipSensorRespon(void *argument);
void StartCanAnalysisTask(void *argument);
void StartUpperControlTask(void *argument);

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

  /* Create the semaphores(s) */
  /* creation of TipSensorTxSem */
  TipSensorTxSemHandle = osSemaphoreNew(1, 1, &TipSensorTxSem_attributes);

  /* creation of TipSensorRxBlockSem */
  TipSensorRxBlockSemHandle = osSemaphoreNew(1, 1, &TipSensorRxBlockSem_attributes);

  /* creation of UpperControlSem */
  UpperControlSemHandle = osSemaphoreNew(1, 1, &UpperControlSem_attributes);

  /* creation of TJ_UART_RX_Sem */
  TJ_UART_RX_SemHandle = osSemaphoreNew(1, 1, &TJ_UART_RX_Sem_attributes);

  /* creation of TJ_Trans_Sem */
  TJ_Trans_SemHandle = osSemaphoreNew(1, 1, &TJ_Trans_Sem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of Tip_Usart_Queue */
  Tip_Usart_QueueHandle = osMessageQueueNew (5, 153, &Tip_Usart_Queue_attributes);

  /* creation of Can_RX_Queue */
  Can_RX_QueueHandle = osMessageQueueNew (16, 10, &Can_RX_Queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of MtParserTask */
  MtParserTaskHandle = osThreadNew(MotorDataParserTaskFun, NULL, &MtParserTask_attributes);

  /* creation of MtControlTask */
  MtControlTaskHandle = osThreadNew(MotorControlTaskFun, NULL, &MtControlTask_attributes);

  /* creation of TipSensorTask */
  TipSensorTaskHandle = osThreadNew(StartTipSensorTask, NULL, &TipSensorTask_attributes);

  /* creation of TipRxDecodeTask */
  TipRxDecodeTaskHandle = osThreadNew(StartTipRxDecodeTask, NULL, &TipRxDecodeTask_attributes);

  /* creation of TipSensorRespon */
  TipSensorResponHandle = osThreadNew(StartTipSensorRespon, NULL, &TipSensorRespon_attributes);

  /* creation of CanAnalysisTask */
  CanAnalysisTaskHandle = osThreadNew(StartCanAnalysisTask, NULL, &CanAnalysisTask_attributes);

  /* creation of UpperControlTas */
  UpperControlTasHandle = osThreadNew(StartUpperControlTask, NULL, &UpperControlTas_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of DecodeEvent */
  DecodeEventHandle = osEventFlagsNew(&DecodeEvent_attributes);

  /* creation of UART_ReceivedEvent */
  UART_ReceivedEventHandle = osEventFlagsNew(&UART_ReceivedEvent_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
__weak void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_MotorDataParserTaskFun */
/**
* @brief Function implementing the MotorParserTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MotorDataParserTaskFun */
__weak void MotorDataParserTaskFun(void *argument)
{
  /* USER CODE BEGIN MotorDataParserTaskFun */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END MotorDataParserTaskFun */
}

/* USER CODE BEGIN Header_MotorControlTaskFun */
/**
* @brief Function implementing the MotorControlT thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MotorControlTaskFun */
__weak void MotorControlTaskFun(void *argument)
{
  /* USER CODE BEGIN MotorControlTaskFun */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END MotorControlTaskFun */
}

/* USER CODE BEGIN Header_StartTipSensorTask */
/**
* @brief Function implementing the TipSensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTipSensorTask */
__weak void StartTipSensorTask(void *argument)
{
  /* USER CODE BEGIN StartTipSensorTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTipSensorTask */
}

/* USER CODE BEGIN Header_StartTipRxDecodeTask */
/**
* @brief Function implementing the TipRxDecodeTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTipRxDecodeTask */
__weak void StartTipRxDecodeTask(void *argument)
{
  /* USER CODE BEGIN StartTipRxDecodeTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTipRxDecodeTask */
}

/* USER CODE BEGIN Header_StartTipSensorRespon */
/**
* @brief Function implementing the TipSensorRespon thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTipSensorRespon */
__weak void StartTipSensorRespon(void *argument)
{
  /* USER CODE BEGIN StartTipSensorRespon */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTipSensorRespon */
}

/* USER CODE BEGIN Header_StartCanAnalysisTask */
/**
* @brief Function implementing the CanAnalysisTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCanAnalysisTask */
__weak void StartCanAnalysisTask(void *argument)
{
  /* USER CODE BEGIN StartCanAnalysisTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartCanAnalysisTask */
}

/* USER CODE BEGIN Header_StartUpperControlTask */
/**
* @brief Function implementing the UpperControlTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUpperControlTask */
__weak void StartUpperControlTask(void *argument)
{
  /* USER CODE BEGIN StartUpperControlTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartUpperControlTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

