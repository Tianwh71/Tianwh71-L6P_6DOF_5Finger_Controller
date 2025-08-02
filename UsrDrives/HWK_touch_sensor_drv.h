/**
  ********************************** Copyright *********************************
  *
  ** (C) Copyright 2022-2025 xxx,China.
  ** All Rights Reserved.
  *                              
  ******************************************************************************
  **--------------------------------------------------------------------------**
  ** @FileName      : HWK_touch_sensor.h  
  ** @Description   : None
  **--------------------------------------------------------------------------**
  ** @Author        : Depressed	  
  ** @Version       : v1.0				
  ** @Creat Date    : 2025-04-02  
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

#ifndef __H_W_K_TOUCH_SENSOR_H_
#define __H_W_K_TOUCH_SENSOR_H_
#include "stdint.h"
#include "stdbool.h"
#include "stm32g4xx_hal.h"
#include "string.h"
#include "tip_sensor_comm_drv.h"

/*串口接收最大字节数量*/
#define HWK_USART_RX_BYTES_MAXIMUM                           230   //华威科传感器最大接收数量
#define HWK_USART_TX_BYTES_MAXIMUM                           50   //华威科传感器最大发送数量
/*默认参数*/
#define HWK_UART_BAUDRATE 921600
#define HWK_UART_DATA_BIT_NUM	8
#define HWK_UART_STOP_BIT_NUM	1


//
#define HWK_FRAME_HEAD 0X3C3C		//帧头
#define HWK_FRAME_TAIL 0X3E3E		//帧尾

/*触点最大支持数量*/
#define TOUCH_SPOT_NUMBER_MAXIMUM 2 
/*日志内容最大长度*/
#define LOG_CONTENT_MAXIMUM 50



#define C2_DATA_WIDTH_1_BYTE 1
#define C2_DATA_WIDTH_2_BYTE 2
#define C2_DATA_WIDTH C2_DATA_WIDTH_1_BYTE



/*帧格式*/
/*小端*/
#pragma pack(1)//全部单字节对齐
//ID 与信道占一个字节，高位为 ID，低位为信道
typedef struct
{
	uint8_t channel:4;
	uint8_t id:4;
}HWK_Id_Channel;
typedef enum
{
	HWK_PUT = 0X00,	//0B00
	HWK_GET = 0X01,	//0B01
	HWK_ACK = 0X02,	//0B10//实测应该是这个
//	HWK_ACK = 0X03,	//0B11//?
}HWK_Pack_Type;
//触觉传感器接收到 GET 请求后，相应包需要将低两位置为 ACK，高位为相同 ID。

typedef struct//主从帧flags
{
	uint8_t pack_type:2;
	uint8_t request_pack_id:6;
}MS_Frame_Flags;

typedef struct//数据负载解析里的flags
{
	uint8_t device_id:5;				//器件id,仅单设备多器件时有作用
	uint8_t data_bit_nums:3;		//数据位数
}Data_Flag;
typedef enum
{
	UINT16 		= 0X00,
	UINT8 		= 0X01,
	INT8 			= 0X02,
	INT16 		= 0X03,
	UINT32 		= 0X04,
	INT32 		= 0X05,
	FLOAT32 	= 0X06,
	UINT16_2 	= 0X07,
}Data_Bit_Nums;
//通道
typedef enum
{
	HWK_DEVICE_INFO 					= 0X01,		//设备信息
	HWK_DATA 									= 0X02,   //数据
	HWK_TOUCH_SPOT 						= 0X03,   //触点
	HWK_GESTURE								= 0X04,   //手势
	HWK_LOG 									= 0X05,   //日志
	HWK_SELECT_REPORT_TYPE 		= 0X06,   //控制上报类型
	HWK_MAKE_ZERO_CALIBRATION = 0X07,   //归零校准
	HWK_SET_DEVICE_ADDR 			= 0X09,   //设置设备地址
	HWK_FILTER 								= 0X0A,   //滤波控制
}HWK_Channel;
/*设备信息 channel1*/
typedef struct
{
	/*0x01 VERSION 版本号*/
	struct {
		uint8_t major;
		uint8_t minor;
		uint32_t patch;
	}version;		
	/*0x02 WHISPER 通信协议版本号*/
	struct {
		uint8_t major;
		uint8_t minor;
		uint32_t patch;
	}whisper;	
	/*0x03 设备的一系列描述*/
	struct {
		uint8_t number;			//器件数量
		struct {						//分辨率
			uint16_t width;				//宽度
			uint16_t height;			//高度
		}resolution;
		uint8_t format;			//数据格式
		struct {						//物理尺寸
			uint16_t width;				//宽度	mm
			uint16_t height;			//高度	mm
		}physical_size;
		struct {						//采样长宽比
			uint8_t width;				//宽度
			uint8_t height;				//高度
		}sar;
		uint8_t reserved;		//缺省占位
	}device;	
	/*0x04 数值范围*/
	struct {
		uint16_t min;						//最小值
		uint16_t max;						//最大值
	}range;								
	/*0x05 串口号*/
	char serial_number[32];		//以0终结的字符串
	/*0x06 设备地址*/
	uint8_t device_addr;	//IIC或其它设置的地址=
}Device_Info;
typedef struct
{
	uint8_t id_channel;
	uint8_t cmd;
}C1_Equal_Device_Info_Request;
typedef struct
{
	uint8_t id_channel;
	Device_Info device_info;//可能只有部分数据，不能直接写入整个内存
}C1_Equal_Device_Info_Report;
/*数据 channel 2*/
typedef struct
{
	uint8_t channel;		//通道，手册描述固定值0x02,实测固定值0x01
	Data_Flag flag;			
	uint8_t column;			//列数
	uint8_t line;				//行数

#if(C2_DATA_WIDTH == C2_DATA_WIDTH_1_BYTE)
	uint8_t data[96];	//数据
#elif(C2_DATA_WIDTH == C2_DATA_WIDTH_2_BYTE)
	uint16_t data[96];	//数据
#endif
	
}C2_Report_Payload;
typedef struct
{
	uint8_t payload;		//0x01
}C2_Request_data_Payload;
/*触点channel3*/
typedef struct
{
	uint16_t x;							//外接矩形，x坐标
	uint16_t y;							//外接矩形，Y坐标
	uint16_t width;					//外接矩形，宽度
	uint16_t hight;					//外接矩形，高度
	struct {
		uint16_t x;
		uint16_t y;
	}center;								//外接矩形，中心点坐标
	uint16_t area;					//触发点数
	uint16_t peak;					//峰值
	uint16_t peak_point;		//峰值点坐标
	uint16_t avg;						//所有点均值
}Touch_Spot_Message;
typedef struct
{
	uint8_t channel;
	uint8_t id;
	uint8_t touch_spot_number;
	Touch_Spot_Message touch_spot_message[TOUCH_SPOT_NUMBER_MAXIMUM];
}C3_Touch_Spot;

/*手势 channel4*/
typedef enum{
	INVALID_EVENT 			= 0X00,
	NONE_EVENT 					= 0X01,
	PRESS_EVENT 				= 0X02,
	LONGPRESS_EVENT 		= 0X03,
	RELEASE_EVENT 			= 0X04,
	GRASP_EVENT 				= 0X05,
	MOVE_EVENT 					= 0X06,
	DOUBLE_CLICK_EVENT 	= 0X08,
}Event_Type;
typedef struct
{
	uint8_t area[TOUCH_SPOT_NUMBER_MAXIMUM];			//触点面积	全部事件类型均有该消息
	uint8_t ppeak[TOUCH_SPOT_NUMBER_MAXIMUM];			//触点峰值	全部事件类型均有该消息
	struct{
		uint16_t x;
		uint16_t y;
	}Start_Position;	//起点坐标			仅MOVE和ZOOM
	struct{
		uint16_t x;
		uint16_t y;		
	}Stop_Position;		//终点坐标			仅MOVE和ZOOM
	uint16_t speed;		//移动速度			仅MOVE和ZOOM
}Gesture_Data;
typedef struct
{
	uint8_t channel;					//信道
	uint8_t id;								//器件id
	Event_Type event_type;		//事件类型
	uint8_t number;						//触点数量
	uint16_t duration;				//持续时长ms
	Gesture_Data gesture_data;//数据
}C4_Gesture;

/*日志  channel 5*/
//上报级别
typedef enum
{
	DEBUG_LEVEL	= 0X00,
	INFO_LEVEL 	= 0X01,
	WARING_LEVEL 	= 0X02,
	ERROR_LEVEL	= 0X03,
	FATAL_LEVEL	= 0X04,
	CLOSE_LEVEL	= 0XFF,
}Log_Report_Level;
typedef struct
{
	uint8_t channel;			//信道 固定值0x05
	uint8_t cmd;					//命令 固定值0x01
	Log_Report_Level log_report_level;		//日志上报级别
}set_log_level;

typedef struct
{
	uint8_t channel;
	Log_Report_Level log_report_level;		//日志级别
	char content[LOG_CONTENT_MAXIMUM];
}C5_Log;

/*控制上报类型 channel 6*/
typedef enum
{
	DATA_FRAME = 0X01,
	TOUCH_SPOT_FRAME = 0X02,
	EVENT_FRAME = 0X03,
	LOG_FRAME = 0X04,
}Frame_Type;
typedef enum
{
	REPORT_OFF = 0X00,
	REPORT_ON = 0X01,
}Report_Switch;
typedef struct
{
	uint8_t channel;				//信道 固定值0x06
	Frame_Type frame_type;
	Report_Switch report_switch;
}C6_Report_Type;

/*归零校准 channel 0x07*/
typedef enum
{
	DISABLE_CALIBRATE = 0X00,
	ENABLE_CALIBRATE 	= 0X01,
	RE_CALIBRATE 			= 0X02,
}Cal_Switch_Instruct;
typedef enum
{
	CALIBRATE_SWITCH = 0X01,		//开启关闭归零校准
	CALIBRATE_PARAMETER = 0X02,	//校准参数
}Cal_Cmd;
typedef enum
{
	MAX = 0X01,
	MEAN = 0X02,
}Cal_Method;
typedef enum
{
	TRUNC = 0X01,
	TOZERO = 0X02,
}Cal_Type;
typedef struct
{
//	uint8_t channel;		//信道		?实际没有
	uint8_t cmd;			//指令
	Cal_Switch_Instruct calibrate_switch_instruct;//控制指令
}Calibrate_Switch;
typedef struct
{
	uint8_t channel;		//信道
	uint8_t cmd;			//指令
	Cal_Method cal_method;	//方式
	uint16_t frames;					//帧数
	uint32_t zoom;						//缩放
	uint16_t static_offset;		//静偏移
	uint32_t dynamic_offset;	//动偏移
	Cal_Type cal_type;
}Calibrate_Parameter;

/*设置设备地址 channe 0x09*/
typedef struct
{
	uint8_t channel;		//信道
	uint8_t addr;				//地址
}Set_Device_Addr;

/*滤波控制 channel 0x0a*/
typedef enum
{
	CLOSE = 0X00,
	OPEN = 0X01,
	SWITCH = 0X02,//切换 意义不明
}Filter_Switch;
typedef struct
{
	uint8_t channel;										//信道
	Filter_Switch filter_control;				//控制
}Filter_Control;
//平等模式下指令帧结构
typedef struct
{
	uint16_t hand;							//帧头
	uint8_t id_channel;					//id/信道
	MS_Frame_Flags flags;				//标志位
	uint16_t length;						//负载长度
	uint8_t *payload;						//负载//使用联合体
	uint16_t chaecksum;					//负载校验
	uint16_t tail;							//帧尾
}HWK_EQUAL_Frame;

typedef struct
{
	uint16_t head;							//帧头
	HWK_Id_Channel id_channel;	//id/信道
	MS_Frame_Flags flags;						//标志位
	uint16_t length;						//负载长度
	void *payload;						//负载//使用联合体
	uint16_t chaecksum;					//负载校验
	uint16_t tail;							//帧尾
}HWK_Master_Slave_Frame;


typedef struct
{
	HWK_Id_Channel id_channel;
	uint8_t cmd;
}C1_MS_Device_Info_Request;
typedef struct
{
	HWK_Id_Channel id_channel;
	Device_Info device_info;//可能只有部分数据，不能直接写入整个内存
}C1_MS_Device_Info_Report;

/*主从模式下请求数据 channel 2*/
typedef struct
{
	uint8_t payload;		//0x01  //意义不明暂时命名为这个
}C2_MS_Request_Data_payload;
/*设置设备地址 channe 0x09*/
typedef struct
{
	HWK_Id_Channel channel;		//信道
	uint8_t addr;				//地址
}MS_Set_Device_Addr;

/*滤波控制*/
typedef struct
{
	HWK_Id_Channel channel;							//信道
	Filter_Switch filter_control;				//控制
}MS_Filter_Control;

#pragma pack(1)   
typedef struct
{
	Device_Info device_info;
	
	uint8_t column;			//列数
	uint8_t line;				//行数
	
#if(C2_DATA_WIDTH == C2_DATA_WIDTH_1_BYTE)
	uint8_t data[12][8];//最大12行8列
#elif(C2_DATA_WIDTH == C2_DATA_WIDTH_2_BYTE)
	uint16_t data[12][8];//最大12行8列
#endif
	
	uint8_t touch_spot_number;
	Touch_Spot_Message touch_spot_message[TOUCH_SPOT_NUMBER_MAXIMUM];

	uint8_t number;						//触点数量
	uint16_t duration;				//持续时长ms
	Gesture_Data gesture_data;//数据
	
	Log_Report_Level log_report_level;		//日志级别
	char content[LOG_CONTENT_MAXIMUM];
}HWK_Sensor;

//传输数据

#pragma pack()


uint16_t crc_16(const uint8_t *data, uint8_t length);


extern HWK_Master_Slave_Frame hwk_MS_tx_frame; 
extern HWK_Master_Slave_Frame hwk_MS_rx_frame; 

extern HWK_Sensor hwk_sensor;
void get_sensor_data(UART_HandleTypeDef *huart,Tip_Usart_Transmit *transmit,HWK_Master_Slave_Frame *MS_frame,uint8_t id,uint8_t req_pack_id);
int ms_decode(Tip_Rx_Data *rx_data,HWK_Master_Slave_Frame *MS_frame,HWK_Sensor *sensor);
void to_zero_cal(UART_HandleTypeDef *huart,Tip_Usart_Transmit *transmit,HWK_Master_Slave_Frame *MS_frame,uint8_t id,Cal_Switch_Instruct cal_switch);
#endif


/******************************** END OF FILE *********************************/


 

