/**
  ********************************** Copyright *********************************
  *
  ** (C) Copyright 2022-2024 YaoYandong,China.
  ** All Rights Reserved.
  *                              
  ******************************************************************************
  **--------------------------------------------------------------------------**
  ** @FileName      : tip_sensor_comm_drv.c  
  ** @Brief         : None
  **--------------------------------------------------------------------------**
  ** @Author Data   : Depressed 2024-10-15
  ** @Version       : v1.0				
  **--------------------------------------------------------------------------**
  ** @Modfier Data  : None
  ** @Version       : None
  ** @Description   : None
  **--------------------------------------------------------------------------**
  ** @Function List : None
  **--------------------------------------------------------------------------**
  ** @Attention     : None
  **--------------------------------------------------------------------------**
  ******************************************************************************
  *
 **/
#include "tip_sensor_comm_drv.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "event_groups.h" 
#include <string.h>
#include "my_math.h" 
#include "data_structure.h"

Tip_Usart_Transmit tip_transmit;
Tip_Rx_Data tip_rx_data_put;
Tip_Rx_Data tip_rx_data_get;
void tip_usart_user_init(void)
{
	HAL_UARTEx_ReceiveToIdle_DMA(&huart3,tip_transmit.receive_buf, TIP_USART_BYTES_MAXIMUM);
};

bool HAL_LPUART1_Receive_IDLE(UART_HandleTypeDef *huart,Tip_Usart_Transmit *tran_handle, uint16_t Size)
{
	bool ret = false;
	uint16_t len;
	BaseType_t xHigherPriorityTaskWoken=pdFALSE;
	{
		__HAL_UART_CLEAR_IDLEFLAG(huart); //清除中断标志
		HAL_UART_DMAStop(huart);//停止DMA接收

			tran_handle->receive_len = Size;  //记录接收数据长度
			tran_handle->receive_flag = true;  //置起接收标志
//1、通过信号量告知任务去解析数据:
//			ret = xSemaphoreGiveFromISR(Tip_Uart_Rx_SemHandle,&xHigherPriorityTaskWoken);//释放信号量//触发485接收到数据事件
//	if( ret != pdFAIL )
//			{
//			/* 如果 xHigherPriorityTaskWoken = pdTRUE，那么退出中断后切到当前最高优先级任务执行 */
////			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//			}
//2、或者本地解析:
//			tran_handle->receive_flag = false;
//			memset(tran_handle->receive_buf,0,90);
			HAL_UARTEx_ReceiveToIdle_DMA(&hlpuart1,tip_transmit.receive_buf, TIP_USART_BYTES_MAXIMUM);
//	3、队列
					len = GET_MINIMUM(Size,TIP_USART_BYTES_MAXIMUM);
					memcpy(&tip_rx_data_put.receive_buf,tran_handle->receive_buf,tran_handle->receive_len);
					tip_rx_data_put.receive_len =  tran_handle->receive_len;
					tip_rx_data_put.receive_flag = true;
					osMessageQueuePut (Tip_Usart_QueueHandle, &tip_rx_data_put, 0, 0);
					HAL_UARTEx_ReceiveToIdle_DMA(&TIP_USART,tip_transmit.receive_buf, TIP_USART_BYTES_MAXIMUM);
					xSemaphoreGiveFromISR(TipSensorRxBlockSemHandle,&xHigherPriorityTaskWoken);
					ret = true;
	}
	return ret;
}


