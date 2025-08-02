
#include "TaskMotorControl.h"
#include "Common.h"
#include "TJ_MotorDrive.h"
#include "TJ_Motor_Control_fml.h"
void MotorControlTaskFun(void *argument)
{
	
	 // 启动初始接收
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, stTjCommAllInfo[0].pstTjAnswer_frame->u8receive_data, TJ_DATA_RECV_LEN_MAX);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart3, stTjCommAllInfo[1].pstTjAnswer_frame->u8receive_data, TJ_DATA_RECV_LEN_MAX);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart4, stTjCommAllInfo[2].pstTjAnswer_frame->u8receive_data, TJ_DATA_RECV_LEN_MAX);
	//下发读取帧获取当前位置
	TJ_Init_Pos(tj_servo, &tj_control_data);
	for (;;)
	{
		TJ_Motor_Control(tj_servo, &tj_control_data); // 控制电机
		tj_locked_rotor_detection(tj_servo, &tj_control_data);
		osDelay(1);
	}
	/* USER CODE END MotorControlTaskFun */
}
