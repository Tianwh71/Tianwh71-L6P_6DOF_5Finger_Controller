/**
  ********************************** Copyright *********************************
  *
  ** (C) Copyright 2022-2024 YaoYandong,China.
  ** All Rights Reserved.
  *                              
  ******************************************************************************
  **--------------------------------------------------------------------------**
  ** @FileName      : tip_sensor_comm_fml.c  
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
#include "tip_sensor_comm_fml.h"
Finger_Tip_Data_All finger_tip_data_all;
Tip_Sensor_Cmd tip_sensor_cmd;
Tip_Send_Data tip_send_data;

//向数据采集板发送数据请求指令
bool tip_sensor_init(Tip_Usart_Transmit *tran_handle,Tip_Sensor_Cmd *cmd)
{
	cmd->period = 3;
	cmd->data_sel.bit_select.thumb = 1;
	cmd->data_sel.bit_select.index = 1;
	cmd->data_sel.bit_select.middle = 1;
	cmd->data_sel.bit_select.ring = 1;
	cmd->data_sel.bit_select.little = 1;
	cmd->data_sel.bit_select.maintain = 1;
	memcpy(tran_handle->send_buf,(void*)cmd,sizeof(Tip_Sensor_Cmd));
	tran_handle->send_len = sizeof(Tip_Sensor_Cmd);
	HAL_UART_Transmit(&huart3,tran_handle->send_buf,tran_handle->send_len,3);
	return true;
}


int len1,len2;
//接收的指尖传感器数据解码
bool tip_sensor_decode(Finger_Tip_Data_All *tip_data_all,Tip_Usart_Transmit *tran_handle)
{
	Finger_Tip_Data *tip_data = (Finger_Tip_Data*)tip_data_all;
	Finger_Tip_Data temp_data;
	len1 = sizeof(Finger_Tip_Data_All);
	len2 = tran_handle->receive_len;
	if(sizeof(Finger_Tip_Data) == tran_handle->receive_len)
//	if(sizeof(Finger_Tip_Data) == 17)
	{
		memcpy((void*)&temp_data,tran_handle->receive_buf,17);
		memcpy((void*)&tip_data[temp_data.finger_number],(void*)&temp_data,17);
	}
	return true;
}

void Prepare_Data_Tip_Force(Finger_Tip_Data_All *tip_data_all,Tip_Send_Data *tip_send)
{
	//Thumb Begain
	if(tip_data_all->thumb.approach_inc < 200000)
	{
			tip_send->Thumb.approach_inc = tip_data_all->thumb.approach_inc/784.3137254f;           //接近变化值
	}
	else
	{
			tip_send->Thumb.approach_inc = 0xff;
	}
	tip_send->Thumb.normal_force = tip_data_all->thumb.normal_force*12.75f;                 //法向力
	tip_send->Thumb.tangential_force = tip_data_all->thumb.tangential_force*12.75f;         //切向力
	if(tip_data_all->thumb.tangential_force_dir == 0xffff)
	{
			tip_send->Thumb.tangential_force_dir = 0xff;
	}
	else
	{
			tip_send->Thumb.tangential_force_dir = tip_data_all->thumb.tangential_force_dir/2.834645f; //切向力方向
	}
	//Thumb End
	
	//Index Begain
	if(tip_data_all->index.approach_inc < 200000)
	{
			tip_send->Index.approach_inc = tip_data_all->index.approach_inc/784.3137254f;           //接近变化值
	}
	else
	{
			tip_send->Index.approach_inc = 0xff;
	}
	tip_send->Index.normal_force = tip_data_all->index.normal_force*12.75f;                 //法向力
	tip_send->Index.tangential_force = tip_data_all->index.tangential_force*12.75f;         //切向力
	if(tip_data_all->index.tangential_force_dir == 0xffff)
	{
			tip_send->Index.tangential_force_dir = 0xff;
	}
	else
	{
			tip_send->Index.tangential_force_dir = tip_data_all->index.tangential_force_dir/2.834645f; //切向力方向
	}
	//Index End
	
	//Middle Begain
	if(tip_data_all->middle.approach_inc < 200000)
	{
			tip_send->Middle.approach_inc = tip_data_all->middle.approach_inc/784.3137254f;           //接近变化值
	}
	else
	{
			tip_send->Middle.approach_inc = 0xff;
	}
	tip_send->Middle.normal_force = tip_data_all->middle.normal_force*12.75f;                 //法向力
	tip_send->Middle.tangential_force = tip_data_all->middle.tangential_force*12.75f;         //切向力
	if(tip_data_all->middle.tangential_force_dir == 0xffff)
	{
			tip_send->Middle.tangential_force_dir = 0xff;
	}
	else
	{
			tip_send->Middle.tangential_force_dir = tip_data_all->middle.tangential_force_dir/2.834645f; //切向力方向
	}
	//Middle End
	
	//Ring Begain
	if(tip_data_all->ring.approach_inc < 200000)
	{
			tip_send->Ring.approach_inc = tip_data_all->ring.approach_inc/784.3137254f;           //接近变化值
	}
	else
	{
			tip_send->Ring.approach_inc = 0xff;
	}
	tip_send->Ring.normal_force = tip_data_all->ring.normal_force*12.75f;                 //法向力
	tip_send->Ring.tangential_force = tip_data_all->ring.tangential_force*12.75f;         //切向力
	if(tip_data_all->ring.tangential_force_dir == 0xffff)
	{
			tip_send->Ring.tangential_force_dir = 0xff;
	}
	else
	{
			tip_send->Ring.tangential_force_dir = tip_data_all->ring.tangential_force_dir/2.834645f; //切向力方向
	}
	//Ring End
	
	//Little Begain
	if(tip_data_all->little.approach_inc < 200000)
	{
			tip_send->Little.approach_inc = tip_data_all->little.approach_inc/784.3137254f;           //接近变化值
	}
	else
	{
			tip_send->Little.approach_inc = 0xff;
	}
	tip_send->Little.normal_force = tip_data_all->little.normal_force*12.75f;                 //法向力
	tip_send->Little.tangential_force = tip_data_all->little.tangential_force*12.75f;         //切向力
	if(tip_data_all->little.tangential_force_dir == 0xffff)
	{
			tip_send->Little.tangential_force_dir = 0xff;
	}
	else
	{
			tip_send->Little.tangential_force_dir = tip_data_all->little.tangential_force_dir/2.834645f; //切向力方向
	}
	//Little End
	
}



