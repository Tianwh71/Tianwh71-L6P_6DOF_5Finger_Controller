/**
 ********************************** Copyright *********************************
 *
 ** (C) Copyright 2022-2024 YaoYandong,China.
 ** All Rights Reserved.
 *
 ******************************************************************************
 **--------------------------------------------------------------------------**
 ** @FileName      : UpperControlTask.c
 ** @Brief         : 上位机指令解析，输出任务
 **--------------------------------------------------------------------------**
 ** @Author Data   : Depressed 2024-04-10
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
#include "UpperControlTask.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "event_groups.h"
#include "data_structure.h"
#include "fdcan.h"
#include "upper_comm_protocol_fml.h"
#include "upper_can_comm_drv.h"
#include "data_structure.h"
#include "TJ_Motor_Control_fml.h"

/* USER CODE BEGIN Header_StartUpperControlTask */
/**
 * @brief Function implementing the UpperControlTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartUpperControlTask */
void StartUpperControlTask(void *argument)
{
	static EventBits_t event_ret;
	/* USER CODE BEGIN StartUpperControlTask */
	/* Infinite loop */
	for (;;)
	{
		// 等待485接收事件，can接收事件，以太网接收事件
		event_ret = xEventGroupWaitBits(DecodeEventHandle,  RX_CAN_EVENT, pdTRUE, pdFALSE, (TickType_t)portMAX_DELAY);

		if (event_ret & RX_CAN_EVENT)
		{
			tj_set_status(tj_servo,&lower_response);
			event_can_dispose();
		}
	}
	/* USER CODE END StartUpperControlTask */
}
