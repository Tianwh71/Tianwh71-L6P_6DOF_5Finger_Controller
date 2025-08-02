/**
  ********************************** Copyright *********************************
  *
  ** (C) Copyright 2022-2024 YaoYandong,China.
  ** All Rights Reserved.
  *                              
  ******************************************************************************
  **--------------------------------------------------------------------------**
  ** @FileName      : tip_sensor_comm_drv.h  
  ** @Description   : None
  **--------------------------------------------------------------------------**
  ** @Author        : Depressed	  
  ** @Version       : v1.0				
  ** @Creat Date    : 2024-10-15  
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

#ifndef __TIP_SENSOR_COMM_DRV_H_
#define __TIP_SENSOR_COMM_DRV_H_

#include "stdint.h"
#include "stdbool.h"
#include "usart.h"

#define TIP_USART_BYTES_MAXIMUM                           150   //modbus传输最大字节
typedef struct
{
	uint8_t send_buf[TIP_USART_BYTES_MAXIMUM];
	uint16_t send_len;
	bool send_en_flag;
	uint8_t receive_buf[TIP_USART_BYTES_MAXIMUM]; 
	uint16_t receive_len;
	bool receive_flag;
}Tip_Usart_Transmit;

#pragma pack(1)
typedef struct
{
	uint8_t receive_buf[TIP_USART_BYTES_MAXIMUM]; 
	uint16_t receive_len;
	bool receive_flag;
}Tip_Rx_Data;
#pragma pack()

extern Tip_Usart_Transmit tip_transmit;
extern Tip_Rx_Data tip_rx_data_put;
extern Tip_Rx_Data tip_rx_data_get;
void tip_usart_user_init(void);

void tip_usart_user_init(void);

bool HAL_LPUART1_Receive_IDLE(UART_HandleTypeDef *huart,Tip_Usart_Transmit *tran_handle, uint16_t Size);
	


#endif


/******************************** END OF FILE *********************************/


 

