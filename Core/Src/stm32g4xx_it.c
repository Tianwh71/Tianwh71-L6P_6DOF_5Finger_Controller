/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32g4xx_it.c
 * @brief   Interrupt Service Routines.
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
#include "stm32g4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "TJ_MotorDrive.h"
#include "Common.h"
#include "tip_sensor_comm_drv.h"
#include "tip_sensor_comm_fml.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern FDCAN_HandleTypeDef hfdcan2;
extern DMA_HandleTypeDef hdma_lpuart1_rx;
extern DMA_HandleTypeDef hdma_lpuart1_tx;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart4_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern UART_HandleTypeDef hlpuart1;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim1;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel2 global interrupt.
  */
void DMA1_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_IRQn 0 */

  /* USER CODE END DMA1_Channel2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
  /* USER CODE BEGIN DMA1_Channel2_IRQn 1 */

  /* USER CODE END DMA1_Channel2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel3 global interrupt.
  */
void DMA1_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel3_IRQn 0 */

  /* USER CODE END DMA1_Channel3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_lpuart1_rx);
  /* USER CODE BEGIN DMA1_Channel3_IRQn 1 */

  /* USER CODE END DMA1_Channel3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel4 global interrupt.
  */
void DMA1_Channel4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel4_IRQn 0 */

  /* USER CODE END DMA1_Channel4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_lpuart1_tx);
  /* USER CODE BEGIN DMA1_Channel4_IRQn 1 */

  /* USER CODE END DMA1_Channel4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel5 global interrupt.
  */
void DMA1_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel5_IRQn 0 */

  /* USER CODE END DMA1_Channel5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA1_Channel5_IRQn 1 */

  /* USER CODE END DMA1_Channel5_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel6 global interrupt.
  */
void DMA1_Channel6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel6_IRQn 0 */

  /* USER CODE END DMA1_Channel6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_tx);
  /* USER CODE BEGIN DMA1_Channel6_IRQn 1 */

  /* USER CODE END DMA1_Channel6_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel7 global interrupt.
  */
void DMA1_Channel7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel7_IRQn 0 */

  /* USER CODE END DMA1_Channel7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart4_rx);
  /* USER CODE BEGIN DMA1_Channel7_IRQn 1 */

  /* USER CODE END DMA1_Channel7_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM16 global interrupt.
  */
void TIM1_UP_TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM16_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM16_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXTI line 26.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt / USART3 wake-up interrupt through EXTI line 28.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles UART4 global interrupt / UART4 wake-up interrupt through EXTI line 34.
  */
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */

  /* USER CODE END UART4_IRQn 0 */
  HAL_UART_IRQHandler(&huart4);
  /* USER CODE BEGIN UART4_IRQn 1 */

  /* USER CODE END UART4_IRQn 1 */
}

/**
  * @brief This function handles FDCAN2 interrupt 0.
  */
void FDCAN2_IT0_IRQHandler(void)
{
  /* USER CODE BEGIN FDCAN2_IT0_IRQn 0 */

  /* USER CODE END FDCAN2_IT0_IRQn 0 */
  HAL_FDCAN_IRQHandler(&hfdcan2);
  /* USER CODE BEGIN FDCAN2_IT0_IRQn 1 */

  /* USER CODE END FDCAN2_IT0_IRQn 1 */
}

/**
  * @brief This function handles LPUART1 global interrupt.
  */
void LPUART1_IRQHandler(void)
{
  /* USER CODE BEGIN LPUART1_IRQn 0 */

  /* USER CODE END LPUART1_IRQn 0 */
  HAL_UART_IRQHandler(&hlpuart1);
  /* USER CODE BEGIN LPUART1_IRQn 1 */
  //HAL_UARTEx_ReceiveToIdle_DMA(&hlpuart1,rx_buffer,BUF_SIZE);	//重新开启串口空闲中断和DMA接收,一定要放在这里
   //HAL_LPUART1_Receive_IDLE(&hlpuart1, &tip_transmit, 303);

  /* USER CODE END LPUART1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel8 global interrupt.
  */
void DMA1_Channel8_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel8_IRQn 0 */

  /* USER CODE END DMA1_Channel8_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart4_tx);
  /* USER CODE BEGIN DMA1_Channel8_IRQn 1 */

  /* USER CODE END DMA1_Channel8_IRQn 1 */
}

/* USER CODE BEGIN 1 */

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance == LPUART1)
  {
    if (huart->RxEventType == HAL_UART_RXEVENT_IDLE)
    {
      HAL_LPUART1_Receive_IDLE(huart, &tip_transmit, Size);

//      tip_sensor_decode(&finger_tip_data_all, &tip_transmit);
    }
    else if (huart->RxEventType == HAL_UART_RXEVENT_HT)
    {
    }
    else if (huart->RxEventType == HAL_UART_RXEVENT_TC)
    {
    }
  }

  if (huart->Instance == USART1)
  {
    if (huart->RxEventType == HAL_UART_RXEVENT_IDLE)
    {
#if (PORT_485_PROTOCOL == SELF_PROTOCOL_485)
      {

        // HAL_UART1_Receive_IDLE(huart,&upper_485_transmit,Size);
      }
#elif (PORT_485_PROTOCOL == MODBUS_485)
      {
        HAL_UART1_Receive_IDLE(huart, &modbus_485_transmit, Size);
      }
#endif
    }
    else if (huart->RxEventType == HAL_UART_RXEVENT_HT)
    {
    }
    else if (huart->RxEventType == HAL_UART_RXEVENT_TC)
    {
    }
  }
  else if (huart->Instance == USART2)
  {
    if (huart->RxEventType == HAL_UART_RXEVENT_IDLE)
    {
      HAL_UART_Receive_IDLE(huart, stTjCommAllInfo[0].pstTjAnswer_frame, Size);
			
    }
		
    else if (huart->RxEventType == HAL_UART_RXEVENT_HT)
    {
      //			HAL_UART_Transmit_DMA(huart,(void*)stTjCommAllInfo.pstTjTransmit->receive_buf,Size);
      //				  HAL_UARTEx_ReceiveToIdle_DMA(huart,(void*)stTjCommAllInfo.pstTjTransmit->receive_buf, 9);
    }
    //else if(huart->RxEventType == HAL_UART_RXEVENT_TC )
    //{
    //			HAL_UART_Transmit_DMA(huart,(void*)stTjCommAllInfo.pstTjTransmit->receive_buf,Size);
    //				  HAL_UARTEx_ReceiveToIdle_DMA(huart,(void*)stTjCommAllInfo.pstTjTransmit->receive_buf, 9);
    //}
    //HAL_UARTEx_ReceiveToIdle_DMA(huart, stTjCommAllInfo[0].pstTjAnswer_frame->u8receive_data, TJ_DATA_RECV_LEN_MAX);
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
  }
  else if (huart->Instance == USART3)
  {
    if (huart->RxEventType == HAL_UART_RXEVENT_IDLE)
    {
      HAL_UART_Receive_IDLE(huart,stTjCommAllInfo[1].pstTjAnswer_frame,Size);
    }
    else if (huart->RxEventType == HAL_UART_RXEVENT_HT)
    {
    }
    else if (huart->RxEventType == HAL_UART_RXEVENT_TC)
    {
    }
		//HAL_UARTEx_ReceiveToIdle_DMA(huart, stTjCommAllInfo[1].pstTjAnswer_frame->u8receive_data, TJ_DATA_RECV_LEN_MAX);
    __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
  }
  else if (huart->Instance == UART4)
  {
    if (huart->RxEventType == HAL_UART_RXEVENT_IDLE)
    {
      HAL_UART_Receive_IDLE(huart,stTjCommAllInfo[2].pstTjAnswer_frame,Size);
    }
    else if (huart->RxEventType == HAL_UART_RXEVENT_HT)
    {
    }
    else if (huart->RxEventType == HAL_UART_RXEVENT_TC)
    {
    }
		//HAL_UARTEx_ReceiveToIdle_DMA(huart, stTjCommAllInfo[2].pstTjAnswer_frame->u8receive_data, TJ_DATA_RECV_LEN_MAX);
    __HAL_DMA_DISABLE_IT(huart4.hdmarx, DMA_IT_HT);
  }
  else if (huart->Instance == UART5)
  {
    if (huart->RxEventType == HAL_UART_RXEVENT_IDLE)
    {
      //				HAL_UART5_Receive_IDLE(huart,&ft_transmit,Size);
    }
    else if (huart->RxEventType == HAL_UART_RXEVENT_HT)
    {
    }
    else if (huart->RxEventType == HAL_UART_RXEVENT_TC)
    {
    }
  }
}

/* USER CODE END 1 */
