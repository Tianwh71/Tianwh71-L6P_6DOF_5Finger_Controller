/**
 ********************************** Copyright *********************************
 *
 ** (C) Copyright 2022-2024 YaoYandong,China.
 ** All Rights Reserved.
 *
 ******************************************************************************
 **--------------------------------------------------------------------------**
 ** @FileName      : data_structure.h
 ** @Description   : None
 **--------------------------------------------------------------------------**
 ** @Author        : Depressed
 ** @Version       : v1.0
 ** @Creat Date    : 2024-04-10
 **--------------------------------------------------------------------------**
 ** @Modfier       : None
 ** @Version       : None
 ** @Modify Date   : None
 ** @Description   : None
 **--------------------------------------------------------------------------**
 ** @Function List : None
 **--------------------------------------------------------------------------**
 ** @Attention     : None
 **--------------------------------------------------------------------------**
 ******************************************************************************
 *
 **/

/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __DATA_STRUCTURE_H_
#define __DATA_STRUCTURE_H_
#include "stdint.h"
#include "stdbool.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

// extern osSemaphoreId_t ModbusRxSemHandle;			//modbus����֡����
extern osSemaphoreId_t UpperControlSemHandle; 		//��λ������ͬ��
// extern osMutexId_t RS485_MutexHandle;         	//rs485�շ������ź���
extern osEventFlagsId_t DecodeEventHandle; 				// ��λ��ͨ�Ž����¼�
extern osSemaphoreId_t TJ_UART_RX_SemHandle;    	//���ض�����ڽӿ��ź���
// extern osSemaphoreId_t FT_ACK_SEMHandle;				//�������ݷ����ź���
extern osSemaphoreId TJ_Trans_SemHandle;       		//����������ʱ���ź���

extern osMessageQueueId_t Can_RX_QueueHandle;
extern osMessageQueueId_t Tip_Usart_QueueHandle;
extern osSemaphoreId_t TipSensorTxSemHandle;
extern osSemaphoreId_t TipSensorRxBlockSemHandle;
extern osSemaphoreId_t UART_ReceivedEventHandle;
// extern osSemaphoreId_t CanTxBlockCountSemHandle;
#endif

/******************************** END OF FILE *********************************/
