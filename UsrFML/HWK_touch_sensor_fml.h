/**
  ********************************** Copyright *********************************
  *
  ** (C) Copyright 2022-2025 xxx,China.
  ** All Rights Reserved.
  *                              
  ******************************************************************************
  **--------------------------------------------------------------------------**
  ** @FileName      : HWK_touch_sensor_fml.h  
  ** @Description   : None
  **--------------------------------------------------------------------------**
  ** @Author        : Depressed	  
  ** @Version       : v1.0				
  ** @Creat Date    : 2025-04-08  
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

#ifndef __H_W_K_TOUCH_SENSOR_FML_H_
#define __H_W_K_TOUCH_SENSOR_FML_H_
#include "HWK_touch_sensor_drv.h"


typedef enum
{
	HWK_THUMB = 1,
	HWK_INDEX = 2,
	HWK_MIDDLE = 3,
	HWK_RING = 4,
	HWK_LITTLE = 5,
	HWK_PALM = 6,
}HWK_HAND_SENSOR_ID;
typedef struct
{
	HWK_Sensor thumb;
	HWK_Sensor index;
	HWK_Sensor middle;
	HWK_Sensor ring;
	HWK_Sensor little;
	HWK_Sensor palm;		//手掌
}HWK_Hand_Sensor;





extern HWK_Hand_Sensor hwk_hand_sensor;
void to_hand_sensor(HWK_Master_Slave_Frame *MS_frame,HWK_Sensor *sensor,HWK_Hand_Sensor*hand_sensor);
#endif


/******************************** END OF FILE *********************************/


 

