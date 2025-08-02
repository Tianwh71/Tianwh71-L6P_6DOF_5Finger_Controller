/**
 ********************************** Copyright *********************************
 *
 ** (C) Copyright 2022-2025 YKT,China.
 ** All Rights Reserved.
 *
 ******************************************************************************
 **--------------------------------------------------------------------------**
 ** @FileName      : HWK_touch_sensor_fml.c
 ** @Brief         : None
 **--------------------------------------------------------------------------**
 ** @Author Data   : Depressed 2025-04-08
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

#include "HWK_touch_sensor_fml.h"
HWK_Hand_Sensor hwk_hand_sensor;

void to_hand_sensor(HWK_Master_Slave_Frame *MS_frame,HWK_Sensor *sensor,HWK_Hand_Sensor*hand_sensor)
{
	HWK_Sensor *pFinger_sensor = (HWK_Sensor*)hand_sensor;
switch(MS_frame->id_channel.channel){
	
		case HWK_DEVICE_INFO 					:{
		}break;
		case HWK_DATA 								:{
			pFinger_sensor[MS_frame->id_channel.id -1]= *sensor;
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

}