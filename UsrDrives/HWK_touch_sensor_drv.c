/**
  ********************************** Copyright *********************************
  *
  ** (C) Copyright 2022-2025 YKT,China.
  ** All Rights Reserved.
  *                              
  ******************************************************************************
  **--------------------------------------------------------------------------**
  ** @FileName      : HWK_touch_sensor_drv.c  
  ** @Brief         : None
  **--------------------------------------------------------------------------**
  ** @Author Data   : Depressed 2025-04-02
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
#include "HWK_touch_sensor_drv.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "data_structure.h"
#include "usart.h"
#include "main.h"

HWK_Master_Slave_Frame hwk_MS_tx_frame; 
HWK_Master_Slave_Frame hwk_MS_rx_frame; 
HWK_Sensor hwk_sensor;


uint16_t crc_16(const uint8_t *data, uint8_t length)
{
	uint16_t crc = 0;
	while (length--) {
		crc ^= *data++;
		for (int i = 0; i < 8; ++i) {
			crc = (crc & 0x0001) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
		}
	}
	return crc;
}

void get_sensor_data(UART_HandleTypeDef *huart,Tip_Usart_Transmit *transmit,HWK_Master_Slave_Frame *MS_frame,uint8_t id,uint8_t req_pack_id)
{
	C2_MS_Request_Data_payload payload;
	
	MS_frame->head = HWK_FRAME_HEAD;
	MS_frame->id_channel.id = id;
	MS_frame->id_channel.channel = HWK_DATA;
	MS_frame->flags.request_pack_id = req_pack_id;
	MS_frame->flags.pack_type = HWK_GET;
	
	payload.payload = 0x01;//根据测试而来
	
	MS_frame->payload = &payload;
	MS_frame->length = sizeof(payload);
	MS_frame->chaecksum = crc_16((uint8_t*)&payload,sizeof(payload));
	MS_frame->tail = HWK_FRAME_TAIL;
	
	memcpy(transmit->send_buf,MS_frame,6);
	memcpy(&transmit->send_buf[6],&payload,sizeof(payload));
	memcpy(&transmit->send_buf[6+sizeof(payload)],&MS_frame->chaecksum,4);
	transmit->send_len = 6+sizeof(payload) +4;
	transmit->send_en_flag = true;
	
	if(transmit->send_en_flag == true)
	{
		xSemaphoreTake(TipSensorTxSemHandle,15);
		HAL_UART_Transmit_DMA(huart,transmit->send_buf,transmit->send_len);
	}
}

void to_zero_cal(UART_HandleTypeDef *huart,Tip_Usart_Transmit *transmit,HWK_Master_Slave_Frame *MS_frame,uint8_t id,Cal_Switch_Instruct cal_switch)
{
	Calibrate_Switch payload;
	
	MS_frame->head = HWK_FRAME_HEAD;
	MS_frame->id_channel.id = 0;
	MS_frame->id_channel.channel = HWK_MAKE_ZERO_CALIBRATION;
	MS_frame->flags.request_pack_id = 0;
	MS_frame->flags.pack_type = HWK_PUT;
	
	payload.cmd = 0x01;//固定值
	payload.calibrate_switch_instruct = cal_switch;
	
	
	MS_frame->payload = &payload;
	MS_frame->length = sizeof(payload);
	MS_frame->chaecksum = crc_16((uint8_t*)&payload,sizeof(payload));
	MS_frame->tail = HWK_FRAME_TAIL;
	
	memcpy(transmit->send_buf,MS_frame,6);
	memcpy(&transmit->send_buf[6],&payload,sizeof(payload));
	memcpy(&transmit->send_buf[6+sizeof(payload)],&MS_frame->chaecksum,4);
	transmit->send_len = 6+sizeof(payload) +4;
	transmit->send_en_flag = true;
	
	if(transmit->send_en_flag == true)
	{
		xSemaphoreTake(TipSensorTxSemHandle,15);
		HAL_UART_Transmit_DMA(huart,transmit->send_buf,transmit->send_len);
	}
}



/*主从模式协议解读*/
int ms_decode(Tip_Rx_Data *rx_data,HWK_Master_Slave_Frame *MS_frame,HWK_Sensor *sensor)
{
	uint16_t check=0;
	memcpy(MS_frame,rx_data->receive_buf,6);
	memcpy(&MS_frame->chaecksum,&rx_data->receive_buf[rx_data->receive_len -4],4);
	if((MS_frame->head != HWK_FRAME_HEAD)||(MS_frame->tail != HWK_FRAME_TAIL))
	{
		return -1;
	}
	if(rx_data->receive_len >= 10){
		check = crc_16(&rx_data->receive_buf[6],rx_data->receive_len-10);
	}else{
		return -1;
	}
	if(MS_frame->chaecksum != check)
	{
		return -1;
	}
	if(MS_frame->flags.pack_type != HWK_ACK)
	{
		return -1;
	}
	//校验通过
	switch(MS_frame->id_channel.channel){
	
		case HWK_DEVICE_INFO 					:{
		}break;
		case HWK_DATA 								:{
			C2_Report_Payload *payload;
			payload = (C2_Report_Payload*)&rx_data->receive_buf[6];
			sensor->column = payload->column;
			sensor->line = payload->line;
			for(int i = 0;i<sensor->line;i++)
				{
				for(int j = 0;j<sensor->column;j++)
					{
						sensor->data[i][j] = payload->data[i*payload->column +j];
					}
				}
			
		}break;
		case HWK_TOUCH_SPOT 					:{
		}break;
		case HWK_GESTURE							:{
		}break;
		case HWK_LOG 									:{
		}break;
		case HWK_SELECT_REPORT_TYPE 	:{
		}break;
		case HWK_MAKE_ZERO_CALIBRATION:{
		}break;
		case HWK_SET_DEVICE_ADDR 			:{
		}break;
		case HWK_FILTER 							:{
		
		}break;
		default:
		{
		
		}break;
	}
	return 0;

}

