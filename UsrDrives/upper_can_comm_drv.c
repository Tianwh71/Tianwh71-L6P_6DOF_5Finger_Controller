/**
 ********************************** Copyright *********************************
 *
 ** (C) Copyright 2022-2024 YaoYandong,China.
 ** All Rights Reserved.
 *
 ******************************************************************************
 **--------------------------------------------------------------------------**
 ** @FileName      : upper_can_comm_drv.c
 ** @Brief         : can初始化，接收滤波器初始化，can接收
 **--------------------------------------------------------------------------**
 ** @Author Data   : Depressed 2024-04-10
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
#include "upper_can_comm_drv.h"
#include "main.h"
#include "fdcan.h"
#include "data_structure.h"
#include "event_groups.h"
#include "task.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "string.h"

FDCAN_RxHeaderTypeDef fdcan2_RxHeader;
FDCAN_TxHeaderTypeDef fdcan2_TxHeader;
RxDataStruct fdcan2_RxStruct; // 定义接收数据结构体
uint8_t can_recv_flag = 0;		// 接收数据标志位
Upper_Can_Transmit upper_can_transmit;
CAN_Frame_t can_frame;
RxDataStruct can_rx_buffer[CAN_RX_BUFFER_SIZE]; // 定义接收缓冲区
volatile uint8_t can_rx_buffer_head = 0;				// 缓冲区头指针
volatile uint8_t can_rx_buffer_tail = 0;				// 缓冲区尾指针
volatile uint8_t can_rx_buffer_count = 0;				// 缓冲区中未处理的消息数量

Can_Rx_Queue can_rx_put_queue;
Can_Rx_Queue can_rx_get_queue;

// can发送一组数据(固定格式:ID为0X12,标准帧,数据帧)
// len:数据长度(最大为8),可设置为FDCAN_DLC_BYTES_2~FDCAN_DLC_BYTES_8
// msg:数据指针,最大为8个字节.
// 返回值:0,成功;
//		 其他,失败;
uint8_t FDCAN2_Send_Msg(uint8_t *msg, uint32_t len, uint8_t ID)
{

	fdcan2_TxHeader.Identifier = ID;
	fdcan2_TxHeader.IdType = FDCAN_STANDARD_ID;
	fdcan2_TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	fdcan2_TxHeader.DataLength = len; // 数据长度
	fdcan2_TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	fdcan2_TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	fdcan2_TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	fdcan2_TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	fdcan2_TxHeader.MessageMarker = 0x52;

	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &fdcan2_TxHeader, msg) != HAL_OK)
		return 1; // 发送
	return 0;
}
// can口接收数据查询
// buf:数据缓存区;
// 返回值:0,无数据被收到;
// 其他,接收的数据长度;
uint8_t FDCAN2_Receive_Msg(uint8_t *buf, uint16_t *Identifier, uint16_t *len)
{

	if (HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &fdcan2_RxHeader, buf) != HAL_OK)
		return 0; // 接收数据
	*Identifier = fdcan2_RxHeader.Identifier;
	*len = fdcan2_RxHeader.DataLength;
	return fdcan2_RxHeader.DataLength;
}

// CAN接收中断回调函数

// 用来保存接收数据端数据
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	uint32_t filllevel = 0;
	static BaseType_t ret;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2, FDCAN_RX_FIFO0) != RESET) // 接收队列不为0，有数据可读
	{
		filllevel = HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2, FDCAN_RX_FIFO0);
		for(int i = 0; i < filllevel; i++)
		{
			// 在此处理接收数据（如存入缓冲区）
			upper_can_transmit.receive_len = FDCAN2_Receive_Msg(upper_can_transmit.receive_buf, &fdcan2_RxStruct.stdId, &fdcan2_RxStruct.length);
			if (fdcan2_RxStruct.stdId == SELF_ID || fdcan2_RxStruct.stdId == 0xFF)
			{
				// /* 检查缓冲区是否已满 */
				if (upper_can_transmit.receive_len != 0 && can_rx_buffer_count < CAN_RX_BUFFER_SIZE) 
				{
					upper_can_transmit.receive_flag=true;
					// 将数据放到can接收队列中
					memcpy(&can_rx_put_queue.receive_buf, upper_can_transmit.receive_buf, upper_can_transmit.receive_len);
					can_rx_put_queue.receive_len = fdcan2_RxStruct.length;
					can_rx_put_queue.receive_flag = true;
					osMessageQueuePut (Can_RX_QueueHandle, &can_rx_put_queue, 0, 0);
					ret = xEventGroupSetBitsFromISR(DecodeEventHandle, RX_CAN_EVENT, &xHigherPriorityTaskWoken); // 触发CAN接收到数据事件
					/* 消息被成功发出 */
					if( ret != pdFAIL )
					{
					/* 如果 xHigherPriorityTaskWoken = pdTRUE，那么退出中断后切到当前最高优先级任务执行 */
						portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
					}
				}
				else
				{
					/* 缓冲区已满，丢弃当前帧 */
				}
			}
		}
	}
}
