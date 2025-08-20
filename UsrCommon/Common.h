#ifndef __CmnON_H_
#define __CmnON_H_
#include <FreeRTOS.h>
#include "cmsis_os.h"
#include <usart.h>

#define LEFT_HAND 1  // 左手
#define RIGHT_HAND 2 // 右手

// #if !defined(HAND_TYPE)
#define HAND_TYPE RIGHT_HAND
/*5、本节点id*/
#if (HAND_TYPE == LEFT_HAND)
#define SELF_ID 0x28
#elif (HAND_TYPE == RIGHT_HAND)
#define SELF_ID 0x27
#endif

//版本号
#define Hand_Freedom   6   //自由度
#define Hand_Version   1    //手版本
#define Hand_Number    1  //手版本序号
#if(HAND_TYPE == LEFT_HAND) //手方向 L对应ASCII码大写L为76(0x4c)
#define Hand_Direction   76      
#elif (HAND_TYPE == RIGHT_HAND) //R对应ASCII码大写R为83(0x52)
	#define Hand_Direction 82	
#endif
#define SoftWare_Version 0x11  //软件版本 数据为十六进制 高四位为大版本，低四位为小版本 如0x11为V1.1
#define HardWare_Version 0x11  //硬件版本 数据为十六进制 高四位为大版本，低四位为小版本 如0x11为V1.1
#define Revision_State   0x10  //稳定版本为0 若软件正在修订则高四位为1，硬件正在修改则低四位为1


typedef enum {
    RX_UART2_EVENT = (0x01 << 0),  // 串口2
    RX_UART3_EVENT = (0x01 << 1),  // 串口3
    RX_UART4_EVENT = (0x01 << 2),  // 串口4
    RX_CAN_EVENT   = (0x01 << 3),  // CAN
} UART_Event_t;
typedef union
{
    float fValue;
    uint32_t u32Value;
    uint8_t au8Bytes[4];
} FloatConverter_t;

typedef enum
{
    ENDIAN_LITTLE = 0,
    ENDIAN_BIG = 1
} EndianType_t;

typedef enum
{
	TASHAN_GATHER = 1,
	HUAWEIKE = 2,
	FULAI = 3,
}Tip_Sensor_Select;

extern Tip_Sensor_Select tip_sensor_select;

extern float Cmn_bytes_to_float(const uint8_t *bytes, EndianType_t EndianMode);
extern void Cmn_float_to_bytes(float value, uint8_t *bytes, EndianType_t EndianMode);
extern float Cmn_bytes_to_uint(const uint8_t *bytes, EndianType_t EndianMode);
extern void Cmn_uint_to_bytes(uint32_t value, uint8_t *bytes, EndianType_t EndianMode);

uint16_t map_0xff_to_2000(uint8_t value, uint16_t angle_max);
uint8_t map_2000_to_0xff(uint16_t value, uint16_t angle_max);
uint16_t map_0xff_to_4000(uint8_t value);
uint8_t map_4000_to_oxff(uint16_t value);

#endif
