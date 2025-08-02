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

// FDCAN1接收RX0中断使能
#define FDCAN1_RX0_INT_ENABLE 1   // 0,不使能;1,使能.
#define UPPER_CAN_BYTES_MAXIMUM 8 // modbus传输最大字节

#define CAN_RX_BUFFER_SIZE 32     // 定义CAN接收缓冲区的大小，最多可存储32个接收帧
#define UPPER_CAN_BYTES_MAXIMUM 8 // 上位机CAN数据最大字节数
typedef struct
{
  uint16_t stdId;
  uint16_t length;
  uint8_t data[8];
} RxDataStruct;

typedef struct
{
  uint8_t id;        /* ID */
  uint8_t len;       /* 数据长度 */
  uint8_t buffer[8]; /*数据*/
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
extern RxDataStruct can_rx_buffer[CAN_RX_BUFFER_SIZE]; // 定义接收缓冲区
extern CAN_Frame_t can_frame;
extern volatile uint8_t can_rx_buffer_head;  // 缓冲区头指针
extern volatile uint8_t can_rx_buffer_tail;  // 缓冲区尾指针
extern volatile uint8_t can_rx_buffer_count; // 缓冲区中未处理的消息数量
extern Upper_Can_Transmit upper_can_transmit;
extern Can_Rx_Queue can_rx_put_queue;
extern Can_Rx_Queue can_rx_get_queue;

uint8_t FDCAN2_Send_Msg(uint8_t *msg, uint32_t len, uint8_t ID);
uint8_t FDCAN2_Receive_Msg(uint8_t *buf, uint16_t *Identifier, uint16_t *len);
#endif

/******************************** END OF FILE *********************************/
