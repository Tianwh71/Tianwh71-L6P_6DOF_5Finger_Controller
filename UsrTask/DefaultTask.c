/**
  ********************************** Copyright *********************************
  *
  ** (C) Copyright 2022-2024 YaoYandong,China.
  ** All Rights Reserved.
  *                              
  ******************************************************************************
  **--------------------------------------------------------------------------**
  ** @FileName      : DefaultTask.c  
  ** @Brief         : 运行指示灯任务
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
#include "DefaultTask.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "gpio.h"
#include "stm32g4xx_hal.h"
/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the DefaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
		//test_weighted_avg_pooling_tempera();
		HAL_GPIO_TogglePin(TEST_LIGHT_GPIO_Port,TEST_LIGHT_Pin);
    osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}


