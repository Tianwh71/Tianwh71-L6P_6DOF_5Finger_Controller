/**
 ********************************** Copyright *********************************
 *
 ** (C) Copyright 2022-2025 YKT,China.
 ** All Rights Reserved.
 *
 ******************************************************************************
 **--------------------------------------------------------------------------**
 ** @FileName      : TipRxDecodeTask.c
 ** @Brief         : None
 **--------------------------------------------------------------------------**
 ** @Author Data   : Depressed 2025-04-03
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

#include "TipSensorTask.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "usart.h"
#include "data_structure.h"
#include "tip_sensor_comm_drv.h"
#include "HWK_touch_sensor_drv.h"
#include "HWK_touch_sensor_fml.h"
/* USER CODE BEGIN Header_StartTipRxDecodeTask */
/**
 * @brief Function implementing the TipRxDecodeTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTipRxDecodeTask */
void StartTipRxDecodeTask(void *argument)
{
  /* USER CODE BEGIN StartTipRxDecodeTask */
  /* Infinite loop */
  for (;;)
  {
    osMessageQueueGet(Tip_Usart_QueueHandle, &tip_rx_data_get, 0, (TickType_t)portMAX_DELAY);
    // 解析
    ms_decode(&tip_rx_data_get, &hwk_MS_rx_frame, &hwk_sensor);
    if(hwk_sensor.column != 6)
    {
      __NOP(); // 调试用
    }
    if(hwk_sensor.line != 0x0c)
    {
      __NOP(); // 调试用
    }
    //to_hand_sensor(&hwk_MS_rx_frame, &hwk_sensor, &hwk_hand_sensor);
		switch (hwk_MS_rx_frame.id_channel.id) 
		{
		case 1: // 大拇指
				hwk_hand_sensor.thumb = hwk_sensor;
			/* code */
			break;
		case 2: // 食指
				hwk_hand_sensor.index = hwk_sensor;
			/* code */
			break;
		case 3: // 中指
				hwk_hand_sensor.middle = hwk_sensor;
				break;
		case 4: // 无名指
				hwk_hand_sensor.ring = hwk_sensor;
				break;
		case 5: // 小指
				hwk_hand_sensor.little = hwk_sensor;
				break;

		default:
			break;
		}
		
    if(hwk_hand_sensor.thumb.column != 6)
    {
      __NOP(); // 调试用
    }
    if(hwk_hand_sensor.thumb.line != 0x0c)
    {
      __NOP(); // 调试用
    }
    memset(tip_rx_data_get.receive_buf, 0, TIP_USART_BYTES_MAXIMUM);
    tip_rx_data_get.receive_flag = false;
    tip_rx_data_get.receive_len = 0;
  }
  /* USER CODE END StartTipRxDecodeTask */
}
