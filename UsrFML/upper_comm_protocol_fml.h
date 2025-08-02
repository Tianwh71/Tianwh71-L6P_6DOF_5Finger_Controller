/**
 ********************************** Copyright *********************************
 *
 ** (C) Copyright 2022-2024 YaoYandong,China.
 ** All Rights Reserved.
 *
 ******************************************************************************
 **--------------------------------------------------------------------------**
 ** @FileName      : upper_comm_protocol_fml.h
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

#ifndef __UPPER_COMM_PROTOCOL_FML_H_
#define __UPPER_COMM_PROTOCOL_FML_H_
#include "stdint.h"
#include "stdbool.h"
#include "tip_sensor_comm_fml.h"
#include "HWK_touch_sensor_fml.h"
#include "data_structure.h"
#include "upper_can_comm_drv.h"
#define RETURN_POSITION (0x00000001 << 0) // 位置1
#define RETURN_PRESS1 (0x00000001 << 1)		// 压力1
// #define RETURN_PRESS2       (0x00000001<<2)			//压力2
// #define RETURN_POSITION2 	(0x00000001<<3)			//位置2
#define RETURN_SPEED (0x00000001 << 4) // 速度1
// #define RETURN_SPEED1		(0x00000001<<5)			//速度2
#define RETURN_ERROR (0x00000001 << 6)		// 帧错误
#define RETURN_ALL_DATA (0x00000001 << 7) // 长帧整包发送
#define RETURN_TEMP1 (0x00000001 << 8)		// 温度1
#define RETURN_TEMP2		(0x00000001<<9)			//电流
#define RETURN_ERROR_CODE1 (0x00000001 << 10) // 帧错误1
// #define RETURN_ERROR_CODE2	(0x00000001<<11)			//帧错误2
#define RETURN_ACCELERATION (0x00000001 << 12) // 电机加速度帧返回

#define RETURN_NONE (0x00000000)

#define FRAME_HEAD_485 0X55 // 485帧头

#define CRC8_POLYNOMIAL 0x31

#define RETURN_THUMB_TOUCH (0x00000001 << 0)	//
#define RETURN_INDEX_TOUCH (0x00000001 << 1)	//
#define RETURN_MIDDLE_TOUCH (0x00000001 << 2) //
#define RETURN_RING_TOUCH (0x00000001 << 3)		//
#define RETURN_LITTLE_TOUCH (0x00000001 << 4) //
#define RETURN_PALM_TOUCH (0x00000001 << 5)		//

#pragma pack(1)

typedef enum
{
	INVALID_COMM_INTERFACE = 0, // 无效的接口选择						|
	COMM_485 = 1,								// 485接口									|
	COMM_CAN = 2,								// can接口									|
	COMM_NET = 3,								// 网口										|
} Comm_Interface;

// 用一个字节表示的矩阵大小
typedef struct
{
	uint8_t column : 4; // 低四位列
	uint8_t row : 4;		// 高四位行

} Matrix_Size_u8;

// 为了照顾can传输的传输长度，此处数据类型都定义成uint8_t,在使用时进行单位转换
typedef struct
{
	uint8_t joint_angle_1; // 每个手指的关节位置设定
	uint8_t joint_angle_2;
	uint8_t joint_angle_3;
	uint8_t joint_angle_4;
	uint8_t joint_angle_5;
	uint8_t joint_angle_6;
	

	uint8_t pressure_1; // 每个指尖的压力设定
	uint8_t pressure_2;
	uint8_t pressure_3;
	uint8_t pressure_4;
	uint8_t pressure_5;
	uint8_t pressure_6; // 每个指尖的压力设定


	uint8_t speed_1;
	uint8_t speed_2;
	uint8_t speed_3;
	uint8_t speed_4;
	uint8_t speed_5;
	uint8_t speed_6;


	uint8_t acceleration_1; // 电机加速度
	uint8_t acceleration_2;
	uint8_t acceleration_3;
	uint8_t acceleration_4;
	uint8_t acceleration_5;
	uint8_t acceleration_6;


	bool press_calibration;			// 压力传感器校准，暂时不启用
	uint8_t clear_fault;				// 对应位高电平有效，高电平清除故障,低六位对应六个电机的清除电流故障
	Matrix_Size_u8 matrix_size; // 指尖传感器数据
} Upper_Request;

// 为了照顾can传输的传输长度，此处数据类型都定义成uint8_t,提醒上位机在使用时进行单位转换
typedef struct
{
	uint8_t curr_joint_angle_1; // 手指的关节位置
	uint8_t curr_joint_angle_2;
	uint8_t curr_joint_angle_3;
	uint8_t curr_joint_angle_4;
	uint8_t curr_joint_angle_5;
	uint8_t curr_joint_angle_6;


	uint8_t curr_pressure_1; // 指尖的当前压力
	uint8_t curr_pressure_2;
	uint8_t curr_pressure_3;
	uint8_t curr_pressure_4;
	uint8_t curr_pressure_5;
	uint8_t curr_pressure_6; // 指尖的当前压力


	uint8_t current_speed_1;
	uint8_t current_speed_2;
	uint8_t current_speed_3;
	uint8_t current_speed_4;
	uint8_t current_speed_5;
	uint8_t current_speed_6;


	uint8_t curr_temp_1; // 温度传感器数据
	uint8_t curr_temp_2; // 温度传感器数据
	uint8_t curr_temp_3; // 温度传感器数据
	uint8_t curr_temp_4; // 温度传感器数据
	uint8_t curr_temp_5; // 温度传感器数据
	uint8_t curr_temp_6; // 温度传感器数据


	uint8_t curr_error_code_1; // 电机故障码
	uint8_t curr_error_code_2; // 电机故障码
	uint8_t curr_error_code_3; // 电机故障码
	uint8_t curr_error_code_4; // 电机故障码
	uint8_t curr_error_code_5; // 电机故障码
	uint8_t curr_error_code_6; // 电机故障码


	uint8_t curr_acceleration_1; // 电机加速度
	uint8_t curr_acceleration_2; // 电机加速度
	uint8_t curr_acceleration_3; // 电机加速度
	uint8_t curr_acceleration_4; // 电机加速度
	uint8_t curr_acceleration_5; // 电机加速度
	uint8_t curr_acceleration_6; // 电机加速度
	
	uint8_t current_mA_1;        //电流
	uint8_t current_mA_2;  
	uint8_t current_mA_3;
	uint8_t current_mA_4;
	uint8_t current_mA_5;
	uint8_t current_mA_6;
	

	uint8_t is_force_calibration; // 压力传感器是否校准标志位
	uint8_t fault_code;						// 故障码低六位分别是六个电机的过扭矩标志

	HWK_Hand_Sensor *hwk_hand_sensor;
	Matrix_Size_u8 matrix_size; // 指尖传感器数据
} Lower_Response;
// 协议的帧属性
typedef enum
{
	INVALID_FRAME_PROPERTY = 0x00, // 无效的can帧属性
	JOINT_POSITION_RCO = 0x01,		 // 关节位置
	MAX_PRESS_RCO = 0x02,					 // 最大压力
	MAX_PRESS_RCO1 = 0x03,				 // 其它数据
	JOINT_POSITION2_RCO = 0X04,		 // 关节位置2
	SPEED_RCO = 0X05,							 // 五个手指的速度
	SPEED_RCO1 = 0X06,						 // 五个手指的速度
	ACCELERATION_RCO = 0X07,			 // 七个电机手指的加速度

	Hand_Normal_Force = 0x20,					// 五个手指的法向压力
	Hand_Tangential_Force = 0x21,			// 五个手指的切向压力
	Hand_Tangential_Force_Dir = 0x22, // 五个手指的切向方向
	Hand_Approach_Inc = 0x23,					// 五个手指指接近感应

	Thumb_All_Data = 0x28,	// 大拇指所有数据
	Index_All_Data = 0x29,	// 食指所有数据
	Middle_All_Data = 0x30, // 中指所有数据
	Ring_All_Data = 0x31,		// 无名指所有数据
	Little_All_Data = 0x32, // 小拇指所有数据

	Temp1 = 0x33,           //温度
	Temp2 = 0x34,   				//电流

	Error_Code1 = 0x35,  
	Error_Code2 = 0x36,

	Home_Rewrite = 0x38,
	Home_Rewrite2 = 0x39,

	WHOLE_FRAME = 0X40, // 整帧传输							|返回一字节帧属性+整个结构体

	Version = 0x64, // 版本号
	SYS_RESET = 0x65,
	ERROR_CODE = 0x66,

	// 华威科、福莱传感器接口指令
	TOUCH_SENSOR_TYPE = 0XB0,
	THUMB_TOUCH = 0XB1,	 // 大拇指
	INDEX_TOUCH = 0XB2,	 // 食指
	MIDDLE_TOUCH = 0XB3, // 中指
	RING_TOUCH = 0XB4,	 // 无名指
	LITTLE_TOUCH = 0XB5, // 小拇指
	PALM_TOUCH = 0XB6,	 // 手掌

	// 配置命令·CONFIG
	HAND_UID = 0XC0,							// 设备唯一标识码
	HAND_HARDWARE_VERSION = 0XC1, // 硬件版本
	HAND_SOFTWARE_VERSION = 0XC2, // 软件版本
	CAN_ID_SET = 0xC3,						// 修改id
	LOCK_ROTOR_THRESHOLD = 0xC4,	// 锁转阈值
	LOCK_ROTOR_TIME = 0xC5,				// 锁转时间
	LOCK_ROTOR_TORQUE = 0xC6,			// 锁转扭矩
	HAND_FACTORY_RESET = 0XCE,		// 恢复出厂设置
	SAVE_PARAMETER = 0xCF,				// 保存
} FRAME_PROPERTY;

typedef struct
{
	uint8_t comm_id;
	Comm_Interface comm_interface; // 标记数据接收时用到的通信接口，在发返回帧的时候用该接口
	uint32_t return_frame_makers;	 // 按位标记要返回的返回帧 |位置 bit0|压力 bit1|其它数据 bit2|暂空bit3|暂空bit4|暂空bit5|暂空bit6|暂空bit7|
	FRAME_PROPERTY frame_property;
	uint32_t return_touch_sensor_makers;
} Protocol_Aux_Data;
// 上位机控制协议
typedef struct
{
	Upper_Request *upper_request;
	Lower_Response *lower_response;
	Protocol_Aux_Data *protocol_aux_data;
} Upper_Protocol;

/*can*/
/*帧结构
|0			|1		|3		|4			|5			|6			|7			|
|帧属性	|			|			|				|				|				|				|
|				|			|			|				|				|				|				|
*/
/*RCO RETURN CURRENT DATA ONCE;N NOT RETURN*/

#pragma pack()

#endif
extern Upper_Request upper_request;
extern Lower_Response lower_response;
extern Upper_Protocol upper_protocol;
extern Protocol_Aux_Data protocol_aux_data;
/*485*/
//void comm_485_parser(Upper_485_Transmit *rs485_trans, Upper_Request *upper_request, Protocol_Aux_Data *aux_data);
//void comm_485_send(Upper_485_Transmit *rs485_trans, Lower_Response *lower_response, FRAME_PROPERTY frame_property);
//void event_485_dispose(void);
/*can*/
//void comm_can_parser(Upper_Can_Transmit *can_trans, Upper_Request *upper_request, Protocol_Aux_Data *aux_data);
//void comm_can_send(Upper_Can_Transmit *can_trans, Lower_Response *lower_response, FRAME_PROPERTY frame_property);
void event_can_dispose(void);
void comm_can_touch_sensor_parser(Can_Rx_Queue *pRx_queue, Upper_Request *pRequest, Lower_Response *pResponse, Protocol_Aux_Data *aux_data);
void comm_can_touch_sensor_send(Upper_Can_Transmit *can_trans, Lower_Response *pResponse, FRAME_PROPERTY frame_property);
void event_can_tip_sensor_dispose(void);
// void comm_modbus_touch_sensor_get(Modbus_485 *modbus, Lower_Response *pResponse, FRAME_PROPERTY frame_property);

/******************************** END OF FILE *********************************/
