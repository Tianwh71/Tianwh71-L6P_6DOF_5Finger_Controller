#include "TaskMotorDataParser.h"
#include "TJ_MotorDrive.h"
#include "Common.h"
#include "fdcan.h"
#include "data_structure.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "TJ_Motor_Control_fml.h"
#include "event_groups.h"


static EventBits_t xEvent;

void MotorDataParserTaskFun(void *argument)
{
  /* USER CODE BEGIN MotorDataParserTaskFun */
  /* Infinite loop */
  for (;;)
  {
			xEvent = xEventGroupWaitBits(UART_ReceivedEventHandle,  RX_UART2_EVENT | RX_UART3_EVENT | RX_UART4_EVENT, pdTRUE, pdFALSE, (TickType_t)portMAX_DELAY);

			/* USART2 */
			if((xEvent & RX_UART2_EVENT) != 0)
			{
				decode_mc_ans_frame(&stTjCommAllInfo[0], &tjMotorParsedData[0]);
				rx_data_2tj_servo(&tjMotorParsedData[0], tj_servo);
				stTjCommAllInfo[0].pstTjAnswer_frame->u8Len = 0;
				
				if(stTjCommAllInfo[0].pstTjTransmit->bNeed_block == true)
				{
					stTjCommAllInfo[0].pstTjTransmit->block_count --;
					stTjCommAllInfo[0].pstTjTransmit->bNeed_block = false;
					xSemaphoreGive(TJ_Trans_SemHandle);
				}
				
			}
			
			/* USART3 */
			if((xEvent & RX_UART3_EVENT) != 0)
			{
				decode_mc_ans_frame(&stTjCommAllInfo[1], &tjMotorParsedData[1]);
				rx_data_2tj_servo(&tjMotorParsedData[1], tj_servo);
				stTjCommAllInfo[1].pstTjAnswer_frame->u8Len = 0;
				
				if(stTjCommAllInfo[1].pstTjTransmit->bNeed_block == true)
				{
					stTjCommAllInfo[1].pstTjTransmit->block_count --;
					stTjCommAllInfo[1].pstTjTransmit->bNeed_block = false;
					xSemaphoreGive(TJ_Trans_SemHandle);
				}
				
			}
			
			/* UART4 */
			if((xEvent & RX_UART4_EVENT) != 0)
			{
				decode_mc_ans_frame(&stTjCommAllInfo[2], &tjMotorParsedData[2]);
				rx_data_2tj_servo(&tjMotorParsedData[2], tj_servo);
				stTjCommAllInfo[2].pstTjAnswer_frame->u8Len = 0;
				
				if(stTjCommAllInfo[2].pstTjTransmit->bNeed_block == true)
				{
					stTjCommAllInfo[2].pstTjTransmit->block_count --;
					stTjCommAllInfo[2].pstTjTransmit->bNeed_block = false;
					xSemaphoreGive(TJ_Trans_SemHandle);
				}
				
			}
			
    }
  /* USER CODE END MotorDataParserTaskFun */
}
