/**
 ********************************** Copyright *********************************
 *
 ** (C) Copyright 2022-2024 YaoYandong,China.
 ** All Rights Reserved.
 *
 ******************************************************************************
 **--------------------------------------------------------------------------**
 ** @FileName      : upper_can_comm_drv.h
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

#ifndef __UPPER_CAN_COMM_DRV_H_
#define __UPPER_CAN_COMM_DRV_H_
#include "stdint.h"
#include "stdbool.h"
#include "stm32g4xx_hal.h"
#include "Common.h"
#include "data_structure.h"

// FDCAN1����RX0�ж�ʹ��
#define FDCAN1_RX0_INT_ENABLE 1   // 0,��ʹ��;1,ʹ��.
#define UPPER_CAN_BYTES_MAXIMUM 8 // modbus��������ֽ�

#define CAN_RX_BUFFER_SIZE 32     // ����CAN���ջ������Ĵ�С�����ɴ洢32������֡
#define UPPER_CAN_BYTES_MAXIMUM 8 // ��λ��CAN��������ֽ���
typedef struct
{
  uint16_t stdId;
  uint16_t length;
  uint8_t data[8];
} RxDataStruct;

typedef struct
{
  uint8_t id;        /* ID */
  uint8_t len;       /* ���ݳ��� */
  uint8_t buffer[8]; /*����*/
} CAN_Frame_t;

#pragma pack(1)
typedef struct
{
  uint8_t receive_buf[UPPER_CAN_BYTES_MAXIMUM];
  uint8_t receive_len;
  bool receive_flag;
} Can_Rx_Queue;
#pragma pack()

typedef struct
{
  uint8_t send_buf[UPPER_CAN_BYTES_MAXIMUM];
  uint16_t send_len;
  bool send_en_flag;
  uint8_t receive_buf[UPPER_CAN_BYTES_MAXIMUM];
  uint16_t receive_len;
  bool receive_flag;
} Upper_Can_Transmit;
extern Upper_Can_Transmit upper_can_transmit;
extern RxDataStruct can_rx_buffer[CAN_RX_BUFFER_SIZE]; // ������ջ�����
extern CAN_Frame_t can_frame;
extern volatile uint8_t can_rx_buffer_head;  // ������ͷָ��
extern volatile uint8_t can_rx_buffer_tail;  // ������βָ��
extern volatile uint8_t can_rx_buffer_count; // ��������δ�������Ϣ����
extern Upper_Can_Transmit upper_can_transmit;
extern Can_Rx_Queue can_rx_put_queue;
extern Can_Rx_Queue can_rx_get_queue;

uint8_t FDCAN2_Send_Msg(uint8_t *msg, uint32_t len, uint8_t ID);
uint8_t FDCAN2_Receive_Msg(uint8_t *buf, uint16_t *Identifier, uint16_t *len);
#endif

/******************************** END OF FILE *********************************/
