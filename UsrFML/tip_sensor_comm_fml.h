/**
  ********************************** Copyright *********************************
  *
  ** (C) Copyright 2022-2024 YaoYandong,China.
  ** All Rights Reserved.
  *                              
  ******************************************************************************
  **--------------------------------------------------------------------------**
  ** @FileName      : tip_sensor_comm_fml.h  
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

#ifndef __TIP_SENSOR_COMM_FML_H_
#define __TIP_SENSOR_COMM_FML_H_
#include "stdint.h"
#include "stdbool.h"
#include "tip_sensor_comm_drv.h"
#include "string.h"
#include "my_math.h"

#define NORMAL_FORCE_MAX 20.0			//法向压力最大值
#define TANGENTIAL_FORCE_MAX 20.0	//切向力最大值
#define APPROACH_INC	10000000			//接近增量最大值

typedef enum
{
	INVALID_TIP_SENSOR = 0,
	TASHAN = 1,
}TIP_SENSOR_TYPE;
typedef enum
{
	THUMB 	= 0,//大拇指
	INDEX 	= 1,//食指
	MIDDLE	= 2,//中指
	RING 		= 3,//无名指
	LITTLE 	= 4,//小拇指
	FINGER_NUMBER_MAXIMUM,	//
}Finger_Number;//手指序号

#pragma pack(1)
typedef struct
{
	bool data_valid;									//数据是否是最新的，有效的
	TIP_SENSOR_TYPE tip_sensor_type;	//传感器类型
	Finger_Number finger_number;			//手指序号
	float normal_force;								//法向力
	float tangential_force;						//切向力
	uint16_t tangential_force_dir;		//切向力方向
	uint32_t approach_inc;						//接近变化值
}Finger_Tip_Data;

typedef struct
{
	Finger_Tip_Data thumb;
	Finger_Tip_Data index;
	Finger_Tip_Data middle;
	Finger_Tip_Data ring;
	Finger_Tip_Data little;
}Finger_Tip_Data_All;
	
#pragma pack()
typedef struct
{
	uint8_t thumb:1;			//0		大拇指
	uint8_t index:1;			//1		食指
	uint8_t middle:1;			//2		中指
	uint8_t ring:1;				//3		无名指
	uint8_t little:1;			//4		小拇指
	uint8_t :2;						//5-6	预留，未用到
	uint8_t maintain:1;		//7		保持
}Ret_Tip_Data_Bit_Select;
typedef union
{
	Ret_Tip_Data_Bit_Select bit_select;
	uint8_t select_code;
}Ret_Tip_Data_Select;
#pragma pack(1)
typedef struct
{
	Ret_Tip_Data_Select data_sel;		//返回数据选取
	uint16_t period;								//数据返回间隔
}Tip_Sensor_Cmd;
#pragma pack()

typedef struct
{
//	TIP_SENSOR_TYPE tip_sensor_type;	//传感器类型
//	Finger_Number finger_number;			//手指序号
	uint8_t normal_force;								//法向力
	uint8_t tangential_force;						//切向力
	uint8_t tangential_force_dir;		    //切向力方向
	uint8_t approach_inc;						    //接近变化值
}Tip_Simplify_Data;

typedef struct
{
		Tip_Simplify_Data Thumb;
	  Tip_Simplify_Data Index;
	  Tip_Simplify_Data Middle;
	  Tip_Simplify_Data Ring;
	  Tip_Simplify_Data Little;
}Tip_Send_Data;

extern Tip_Sensor_Cmd tip_sensor_cmd;
extern Finger_Tip_Data_All finger_tip_data_all;
extern Tip_Send_Data tip_send_data;
	
bool tip_sensor_init(Tip_Usart_Transmit *tran_handle,Tip_Sensor_Cmd *cmd);
bool tip_sensor_decode(Finger_Tip_Data_All *tip_data_all,Tip_Usart_Transmit *tran_handle);
void Prepare_Data_Tip_Force(Finger_Tip_Data_All *tip_data_all,Tip_Send_Data *tip_send);

#endif


/******************************** END OF FILE *********************************/


 

