/**
 ********************************** Copyright *********************************
 *
 ** (C) Copyright 2022-2024 YaoYandong,China.
 ** All Rights Reserved.
 *
 ******************************************************************************
 **--------------------------------------------------------------------------**
 ** @FileName      : upper_can_comm_drv.c
 ** @Brief         : can��ʼ���������˲�����ʼ����can����
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
RxDataStruct fdcan2_RxStruct; // ����������ݽṹ��
uint8_t can_recv_flag = 0;		// �������ݱ�־λ
Upper_Can_Transmit upper_can_transmit;
CAN_Frame_t can_frame;
RxDataStruct can_rx_buffer[CAN_RX_BUFFER_SIZE]; // ������ջ�����
volatile uint8_t can_rx_buffer_head = 0;				// ������ͷָ��
volatile uint8_t can_rx_buffer_tail = 0;				// ������βָ��
volatile uint8_t can_rx_buffer_count = 0;				// ��������δ�������Ϣ����

Can_Rx_Queue can_rx_put_queue;
Can_Rx_Queue can_rx_get_queue;

// can����һ������(�̶���ʽ:IDΪ0X12,��׼֡,����֡)
// len:���ݳ���(���Ϊ8),������ΪFDCAN_DLC_BYTES_2~FDCAN_DLC_BYTES_8
// msg:����ָ��,���Ϊ8���ֽ�.
// ����ֵ:0,�ɹ�;
//		 ����,ʧ��;
uint8_t FDCAN2_Send_Msg(uint8_t *msg, uint32_t len, uint8_t ID)
{

	fdcan2_TxHeader.Identifier = ID;
	fdcan2_TxHeader.IdType = FDCAN_STANDARD_ID;
	fdcan2_TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	fdcan2_TxHeader.DataLength = len; // ���ݳ���
	fdcan2_TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	fdcan2_TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	fdcan2_TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	fdcan2_TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	fdcan2_TxHeader.MessageMarker = 0x52;

	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &fdcan2_TxHeader, msg) != HAL_OK)
		return 1; // ����
	return 0;
}
// can�ڽ������ݲ�ѯ
// buf:���ݻ�����;
// ����ֵ:0,�����ݱ��յ�;
// ����,���յ����ݳ���;
uint8_t FDCAN2_Receive_Msg(uint8_t *buf, uint16_t *Identifier, uint16_t *len)
{

	if (HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &fdcan2_RxHeader, buf) != HAL_OK)
		return 0; // ��������
	*Identifier = fdcan2_RxHeader.Identifier;
	*len = fdcan2_RxHeader.DataLength;
	return fdcan2_RxHeader.DataLength;
}

// CAN�����жϻص�����

// ��������������ݶ�����
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	uint32_t filllevel = 0;
	static BaseType_t ret;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2, FDCAN_RX_FIFO0) != RESET) // ���ն��в�Ϊ0�������ݿɶ�
	{
		filllevel = HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2, FDCAN_RX_FIFO0);
		for(int i = 0; i < filllevel; i++)
		{
			// �ڴ˴���������ݣ�����뻺������
			upper_can_transmit.receive_len = FDCAN2_Receive_Msg(upper_can_transmit.receive_buf, &fdcan2_RxStruct.stdId, &fdcan2_RxStruct.length);
			if (fdcan2_RxStruct.stdId == SELF_ID || fdcan2_RxStruct.stdId == 0xFF)
			{
				// /* ��黺�����Ƿ����� */
				if (upper_can_transmit.receive_len != 0 && can_rx_buffer_count < CAN_RX_BUFFER_SIZE) 
				{
					upper_can_transmit.receive_flag=true;
					// �����ݷŵ�can���ն�����
					memcpy(&can_rx_put_queue.receive_buf, upper_can_transmit.receive_buf, upper_can_transmit.receive_len);
					can_rx_put_queue.receive_len = fdcan2_RxStruct.length;
					can_rx_put_queue.receive_flag = true;
					osMessageQueuePut (Can_RX_QueueHandle, &can_rx_put_queue, 0, 0);
					ret = xEventGroupSetBitsFromISR(DecodeEventHandle, RX_CAN_EVENT, &xHigherPriorityTaskWoken); // ����CAN���յ������¼�
					/* ��Ϣ���ɹ����� */
					if( ret != pdFAIL )
					{
					/* ��� xHigherPriorityTaskWoken = pdTRUE����ô�˳��жϺ��е���ǰ������ȼ�����ִ�� */
						portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
					}
				}
				else
				{
					/* ������������������ǰ֡ */
				}
			}
		}
	}
}
