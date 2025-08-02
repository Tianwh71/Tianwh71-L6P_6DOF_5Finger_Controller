#include "Common.h"




float Cmn_bytes_to_float(const uint8_t* bytes,EndianType_t EndianMode)
{
    FloatConverter_t conv;
	
	  if(ENDIAN_LITTLE == EndianMode)
		{
        conv.au8Bytes[0] = bytes[0];  // LSB (小端模式)
        conv.au8Bytes[1] = bytes[1];
        conv.au8Bytes[2] = bytes[2];
        conv.au8Bytes[3] = bytes[3];  // MSB		
		}else
    {
        conv.au8Bytes[0] = bytes[3];  // MSB (大端模式)
        conv.au8Bytes[1] = bytes[2];
        conv.au8Bytes[2] = bytes[1];
        conv.au8Bytes[3] = bytes[0];  // SSB			
		
		}

    return conv.fValue;
}

// 将float拆分为4字节
void  Cmn_float_to_bytes(float value, uint8_t* bytes,EndianType_t EndianMode) 
{
    FloatConverter_t conv;
    conv.fValue = value;
	  
	  if(ENDIAN_LITTLE == EndianMode)
		{
        bytes[0] = conv.au8Bytes[0];
        bytes[1] = conv.au8Bytes[1];
        bytes[2] = conv.au8Bytes[2];
        bytes[3] = conv.au8Bytes[3];		
		}else
    {
        bytes[0] = conv.au8Bytes[3];
        bytes[1] = conv.au8Bytes[2];
        bytes[2] = conv.au8Bytes[1];
        bytes[3] = conv.au8Bytes[0];				
		
		}
}

float Cmn_bytes_to_uint(const uint8_t* bytes,EndianType_t EndianMode)
{
    FloatConverter_t conv;
	
	  if(ENDIAN_LITTLE == EndianMode)
		{
        conv.au8Bytes[0] = bytes[0];  // LSB (小端模式)
        conv.au8Bytes[1] = bytes[1];
        conv.au8Bytes[2] = bytes[2];
        conv.au8Bytes[3] = bytes[3];  // MSB		
		}else
    {
        conv.au8Bytes[0] = bytes[3];  // MSB (大端模式)
        conv.au8Bytes[1] = bytes[2];
        conv.au8Bytes[2] = bytes[1];
        conv.au8Bytes[3] = bytes[0];  // SSB			
		
		}

    return conv.u32Value;
}

// 将无符号拆分为4字节
void Cmn_uint_to_bytes(uint32_t value, uint8_t* bytes,EndianType_t EndianMode) 
{
    FloatConverter_t conv;
    conv.u32Value = value;
	  
	  if(ENDIAN_LITTLE == EndianMode)
		{
        bytes[0] = conv.au8Bytes[0];
        bytes[1] = conv.au8Bytes[1];
        bytes[2] = conv.au8Bytes[2];
        bytes[3] = conv.au8Bytes[3];		
		}else
    {
        bytes[0] = conv.au8Bytes[3];
        bytes[1] = conv.au8Bytes[2];
        bytes[2] = conv.au8Bytes[1];
        bytes[3] = conv.au8Bytes[0];				
		
		}
}

uint16_t map_0xff_to_2000(uint8_t value)
{
    // 线性映射：0 -> 0, 255 -> 2000
    return (uint16_t)((value * 2000) / 255);
}
uint8_t map_2000_to_oxff(uint16_t value)
{
	return (uint8_t)((value * 255) /2000);
}
uint16_t map_0xff_to_4000(uint8_t value)
{
    // 线性映射：0 -> 0, 255 -> 4000
    return (uint16_t)((value * 4000) / 255);
}
uint8_t map_4000_to_oxff(uint16_t value)
{
	return (uint8_t)((value * 255) /4000);
}