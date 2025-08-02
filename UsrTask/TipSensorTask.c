/**
 ********************************** Copyright *********************************
 *
 ** (C) Copyright 2022-2024 YaoYandong,China.
 ** All Rights Reserved.
 *
 ******************************************************************************
 **--------------------------------------------------------------------------**
 ** @FileName      : TipSensorTask.c
 ** @Brief         : None
 **--------------------------------------------------------------------------**
 ** @Author Data   : Depressed 2024-10-15
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

Cal_Switch_Instruct cal_switch = ENABLE_CALIBRATE;

/* USER CODE BEGIN Header_StartTipSensorTask */
/**
 * @brief Function implementing the TipSensorTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTipSensorTask */
void StartTipSensorTask(void *argument)
{
  BaseType_t ret;
  HAL_UARTEx_ReceiveToIdle_DMA(&TIP_USART, tip_transmit.receive_buf, TIP_USART_BYTES_MAXIMUM);
  to_zero_cal(&TIP_USART, &tip_transmit, &hwk_MS_tx_frame, 0, cal_switch);
  osDelay(50);

  /* USER CODE BEGIN StartTipSensorTask */
  /* Infinite loop */
  for (;;)
  {
    for (int i = 1; i < 6; i++)
    {
      ret = xSemaphoreTake(TipSensorRxBlockSemHandle, 30); // 921600波特率下//该值要大于9
      if (ret == pdFALSE)
      {
        HAL_UARTEx_ReceiveToIdle_DMA(&TIP_USART, tip_transmit.receive_buf, TIP_USART_BYTES_MAXIMUM);
      }
      get_sensor_data(&TIP_USART, &tip_transmit, &hwk_MS_tx_frame, i, 1);
    }
    osDelay(23);
  }
  /* USER CODE END StartTipSensorTask */
}
