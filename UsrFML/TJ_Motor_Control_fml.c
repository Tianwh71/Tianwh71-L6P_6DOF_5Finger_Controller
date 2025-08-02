#include "TJ_Motor_Control_fml.h"
#include "TJ_MotorDrive.h"
#include "upper_can_comm_drv.h"
#include "Common.h"
#include "my_math.h"

TJ_Servo tj_servo[TJ_ID_MAXIMUM] = {
    // 舵机1
    [TJ_ID_1].is_servo_valid = true,
    [TJ_ID_1].is_servo_online = true,
    [TJ_ID_1].go_home = true,
    [TJ_ID_1].pos_limit_min = 0,
    [TJ_ID_1].pos_limit_max = 2000,
    [TJ_ID_1].servo_id = TJ_ID_1,

    // 舵机2
    [TJ_ID_2].is_servo_valid = true,
    [TJ_ID_2].is_servo_online = true,
    [TJ_ID_2].go_home = true,
    [TJ_ID_2].pos_limit_min = 0,
    [TJ_ID_2].pos_limit_max = 2000,
    [TJ_ID_2].servo_id = TJ_ID_2,
    // 舵机3
    [TJ_ID_3].is_servo_valid = true,
    [TJ_ID_3].is_servo_online = true,
    [TJ_ID_3].go_home = true,
    [TJ_ID_3].pos_limit_min = 0,
    [TJ_ID_3].pos_limit_max = 2000,
    [TJ_ID_3].servo_id = TJ_ID_3,
    // 舵机4
    [TJ_ID_4].is_servo_valid = true,
    [TJ_ID_4].is_servo_online = true,
    [TJ_ID_4].go_home = true,
    [TJ_ID_4].pos_limit_min = 0,
    [TJ_ID_4].pos_limit_max = 2000,
    [TJ_ID_4].servo_id = TJ_ID_4,
    // 舵机5
    [TJ_ID_5].is_servo_valid = true,
    [TJ_ID_5].is_servo_online = true,
    [TJ_ID_5].go_home = true,
    [TJ_ID_5].pos_limit_min = 0,
    [TJ_ID_5].pos_limit_max = 2000,
    [TJ_ID_5].servo_id = TJ_ID_5,
    // 舵机6
    [TJ_ID_6].is_servo_valid = true,
    [TJ_ID_6].is_servo_online = true,
    [TJ_ID_6].go_home = true,
    [TJ_ID_6].pos_limit_min = 0,
    [TJ_ID_6].pos_limit_max = 2000,
    [TJ_ID_6].servo_id = TJ_ID_6,
};

TJ_Control_Data tj_control_data =
    {
        .thumb = {
            .angle = 0.0f,
            .last_angle = 0.0f,
            .angle_min = 0.0f,
            .angle_max = 2000.0f,
            .init_angle = 1000.0f,
						.locked_rotor.threshold = JOINT_LOCK_THRESHOLD,
						.locked_rotor.time = JOINT_LOCK_TIME,
						.locked_rotor.safe_current = JOINT_LOCK_TORQUE,
        },
        .index = {
            .angle = 2000.0f,
            .last_angle = 0.0f,
            .angle_min = 0.0f,
            .angle_max = 2000.0f,
            .init_angle = 1000.0f,
						.locked_rotor.threshold = JOINT_LOCK_THRESHOLD,
						.locked_rotor.time = JOINT_LOCK_TIME,
						.locked_rotor.safe_current = JOINT_LOCK_TORQUE,
        },
        .middle = {
            .angle = 2000.0f,
            .last_angle = 0.0f,
            .angle_min = 0.0f,
            .angle_max = 2000.0f,
            .init_angle = 1000.0f,
						.locked_rotor.threshold = JOINT_LOCK_THRESHOLD,
						.locked_rotor.time = JOINT_LOCK_TIME,
						.locked_rotor.safe_current = JOINT_LOCK_TORQUE,
        },
        .ring = {
            .angle = 2000.0f,
            .last_angle = 0.0f,
            .angle_min = 0.0f,
            .angle_max = 2000.0f,
            .init_angle = 1000.0f,
						.locked_rotor.threshold = JOINT_LOCK_THRESHOLD,
						.locked_rotor.time = JOINT_LOCK_TIME,
						.locked_rotor.safe_current = JOINT_LOCK_TORQUE,
        },
        .little = {
            .angle = 2000.0f,
            .last_angle = 0.0f,
            .angle_min = 0.0f,
            .angle_max = 2000.0f,
            .init_angle = 1000.0f,
					  .locked_rotor.threshold = JOINT_LOCK_THRESHOLD,
						.locked_rotor.time = JOINT_LOCK_TIME,
						.locked_rotor.safe_current = JOINT_LOCK_TORQUE,
        },
        .thumb_yaw = {
            .angle = 0.0f,
            .last_angle = 0.0f,
            .angle_min = 0.0f,
            .angle_max = 2000.0f,
            .init_angle = 1000.0f,
						.locked_rotor.threshold = JOINT_LOCK_THRESHOLD,
						.locked_rotor.time = JOINT_LOCK_TIME,
						.locked_rotor.safe_current = JOINT_LOCK_TORQUE,
        },

};
void rx_data_2tj_servo(tjData *pProtocol, TJ_Servo *pServo)
{
	// todo:将协议初步解析的数据放入到tj舵机数据结构体
	if(pProtocol->decode_state == PARSING_SUCCESS)
	{
		tj_servo[pProtocol->stTjConfig_data.u8MotorID].servo_id = pProtocol->stTjConfig_data.u8MotorID;
		tj_servo[pProtocol->stTjConfig_data.u8MotorID].is_servo_online = true;
		//todo:将协议初步解析的数据放入到飞特舵机数据结构体
		if(tj_servo[pProtocol->stTjConfig_data.u8MotorID].is_servo_valid == true)
		{
			memcpy(&(tj_servo[pProtocol->stTjConfig_data.u8MotorID].tj_servo_data), pProtocol, sizeof(tjData));  
		}	
	}		
}
void TJ_Motor_Control(TJ_Servo *pServo, TJ_Control_Data *tj_control_data)
{
  TJ_Control_Data_Unit *data_unit;
	for (uint8_t i = TJ_ID_1; i < TJ_ID_MAXIMUM; i++)
	{
		switch (i) 
		{
		case 1: data_unit = &tj_control_data->thumb; break;
		case 2: data_unit = &tj_control_data->index; break;
		case 3: data_unit = &tj_control_data->middle; break;
		case 4: data_unit = &tj_control_data->ring; break;
		case 5: data_unit = &tj_control_data->little; break;
		case 6: data_unit = &tj_control_data->thumb_yaw; break;
		default: continue;
		}
		
    //if (data_unit[i - 1].angle != data_unit[i - 1].last_angle) // 指令改变才去控制电机
    {
			clear_usart2_fault();
			clear_usart3_fault();
			clear_usart4_fault();
      position_mode_r(stTjCommAllInfo, i, data_unit->angle, TJ_TARGET_POS);
      data_unit[i - 1].last_angle = data_unit->angle;
			osDelay(1);
    }
  }
}

void TJ_Init_Pos(TJ_Servo *pServo, TJ_Control_Data *tj_control_data)
{
	static uint16_t target_pos;
	uint8_t get_data_count = 0;
	TJ_Control_Data_Unit *data_unit;
	for (uint8_t i = TJ_ID_1; i < TJ_ID_MAXIMUM; i++)
	{
		switch (i) 
		{
		case 1: data_unit = &tj_control_data->thumb; break;
		case 2: data_unit = &tj_control_data->index; break;
		case 3: data_unit = &tj_control_data->middle; break;
		case 4: data_unit = &tj_control_data->ring; break;
		case 5: data_unit = &tj_control_data->little; break;
		case 6: data_unit = &tj_control_data->thumb_yaw; break;
		default: continue;
		}
		uint8_t retry = 0;
		do {
				read_table(stTjCommAllInfo, i, TJ_TORQUE_ACK);
				osDelay(1);
		} while(pServo[i].tj_servo_data.stTjConfig_data.u8MotorID == 0 && retry++ < MAX_WAIT_ACK_TIME); 
		if (pServo[i].tj_servo_data.stTjConfig_data.u8MotorID != (TJ_SERVO_ID)i) // 通过这个判断去判定通讯是否正常，电机是否在线
		{
			pServo[i].is_servo_online = false; // 如果电机不在线，就将其状态更改为无效
		}
		target_pos = CONSTRAIN(pServo[i].tj_servo_data.stTjStatus_data.Current_position,pServo[i].pos_limit_max, pServo[i].pos_limit_min);
		position_mode_r(stTjCommAllInfo, i, target_pos, TJ_TARGET_POS);                                                                                                                                                                                                                                                                                                                        
		data_unit->angle = target_pos;
		data_unit->last_angle = data_unit->angle;
		osDelay(1);
	}
}

void clear_usart2_fault(void)
{
		if (((huart2.Instance->ISR & UART_FLAG_FE) != RESET))
	{
		__HAL_UART_CLEAR_FEFLAG(&huart2);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, stTjCommAllInfo[0].pstTjAnswer_frame->u8receive_data, TJ_DATA_RECV_LEN_MAX);
	}
	if (((huart2.Instance->ISR & UART_FLAG_ORE) != RESET))
	{
		__HAL_UART_CLEAR_OREFLAG(&huart2);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, stTjCommAllInfo[0].pstTjAnswer_frame->u8receive_data, TJ_DATA_RECV_LEN_MAX);
	}	
	if (((huart2.Instance->ISR & UART_FLAG_NE) != RESET))
	{
		__HAL_UART_CLEAR_NEFLAG(&huart2);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, stTjCommAllInfo[0].pstTjAnswer_frame->u8receive_data, TJ_DATA_RECV_LEN_MAX);	
	}	

}
void clear_usart3_fault(void)
{
		if (((huart3.Instance->ISR & UART_FLAG_FE) != RESET))
	{
		__HAL_UART_CLEAR_FEFLAG(&huart3);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart3, stTjCommAllInfo[1].pstTjAnswer_frame->u8receive_data, TJ_DATA_RECV_LEN_MAX);
	}
	if (((huart3.Instance->ISR & UART_FLAG_ORE) != RESET))
	{
		__HAL_UART_CLEAR_OREFLAG(&huart3);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart3, stTjCommAllInfo[1].pstTjAnswer_frame->u8receive_data, TJ_DATA_RECV_LEN_MAX);
	}	
	if (((huart3.Instance->ISR & UART_FLAG_NE) != RESET))
	{
		__HAL_UART_CLEAR_NEFLAG(&huart3);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart3, stTjCommAllInfo[1].pstTjAnswer_frame->u8receive_data, TJ_DATA_RECV_LEN_MAX);
	}	

}

void clear_usart4_fault(void)
{
		if (((huart4.Instance->ISR & UART_FLAG_FE) != RESET))
	{
		__HAL_UART_CLEAR_FEFLAG(&huart4);
		
	HAL_UARTEx_ReceiveToIdle_DMA(&huart4, stTjCommAllInfo[2].pstTjAnswer_frame->u8receive_data, TJ_DATA_RECV_LEN_MAX);
	}
	if (((huart4.Instance->ISR & UART_FLAG_ORE) != RESET))
	{
		__HAL_UART_CLEAR_OREFLAG(&huart4);
		
	HAL_UARTEx_ReceiveToIdle_DMA(&huart4, stTjCommAllInfo[2].pstTjAnswer_frame->u8receive_data, TJ_DATA_RECV_LEN_MAX);
	}	
	if (((huart4.Instance->ISR & UART_FLAG_NE) != RESET))
	{
		__HAL_UART_CLEAR_NEFLAG(&huart4);
		
	HAL_UARTEx_ReceiveToIdle_DMA(&huart4, stTjCommAllInfo[2].pstTjAnswer_frame->u8receive_data, TJ_DATA_RECV_LEN_MAX);
	}	

}


void tj_get_cmd(TJ_Servo *pServo, TJ_Control_Data *ft_control_data, Upper_Request *upper_request, Protocol_Aux_Data *aux_data)
{
  TJ_Control_Data_Unit *data_unit = (TJ_Control_Data_Unit *)ft_control_data;
  uint8_t *request_pos = (uint8_t *)&upper_request->joint_angle_1;
  uint8_t *request_torque = (uint8_t *)&upper_request->pressure_1;
  uint8_t *request_speed = (uint8_t *)&upper_request->speed_1;
  uint8_t *request_acceleration = (uint8_t *)&upper_request->acceleration_1;
  switch (aux_data->frame_property)
  {
  case JOINT_POSITION_RCO:
  {
    // 电机位置
     for (int i = TJ_ID_1; i < TJ_ID_MAXIMUM; i++) 
     {
     if (pServo[i].is_servo_valid == true)
     {
        if (i == TJ_ID_1 || i == TJ_ID_6) 
        {
           data_unit[i - 1].angle = map_0xff_to_2000(255 - request_pos[i-1]);
        }
        else
				{
					data_unit[i - 1].angle = map_0xff_to_2000(request_pos[i-1]);
				}
			}
		}
   
  }
  break;
  case MAX_PRESS_RCO:
  {
    data_unit[0].target_torque = map_0xff_to_4000(request_torque[0]);
    data_unit[1].target_torque = map_0xff_to_4000(request_torque[1]);
    data_unit[2].target_torque = map_0xff_to_4000(request_torque[2]);
    data_unit[3].target_torque = map_0xff_to_4000(request_torque[3]);
    data_unit[4].target_torque = map_0xff_to_4000(request_torque[4]);
		data_unit[5].target_torque = map_0xff_to_4000(request_torque[5]);
  }
  break;

  case SPEED_RCO:
  {
    if (pServo[TJ_ID_1].is_servo_valid == true)
    {
      data_unit[TJ_ID_1 - 1].speed_ref = map_0xff_to_4000(request_speed[0]);
    }
     if (pServo[TJ_ID_2].is_servo_valid == true)
    {
			 data_unit[TJ_ID_2 - 1].speed_ref = map_0xff_to_4000(request_speed[1]);
		}
		if (pServo[TJ_ID_3].is_servo_valid == true)
		{
			 data_unit[TJ_ID_3 - 1].speed_ref = map_0xff_to_4000(request_speed[2]);
		}
		if (pServo[TJ_ID_4].is_servo_valid == true)
		{
			 data_unit[TJ_ID_4 - 1].speed_ref = map_0xff_to_4000(request_speed[3]);
		}
		 if (pServo[TJ_ID_5].is_servo_valid == true)
		 {
			 data_unit[TJ_ID_5 - 1].speed_ref = map_0xff_to_4000(request_speed[4]);
		 }
		 if (pServo[TJ_ID_6].is_servo_valid == true)
		 {
			 data_unit[TJ_ID_6 - 1].speed_ref = map_0xff_to_4000(request_speed[0]);
		 }
		}
  break;

  case ACCELERATION_RCO:
  {
    data_unit[TJ_ID_1 - 1].acceleration_ref = request_acceleration[0];
    data_unit[TJ_ID_2 - 1].acceleration_ref = request_acceleration[1];
    data_unit[TJ_ID_3 - 1].acceleration_ref = request_acceleration[2];
    data_unit[TJ_ID_4 - 1].acceleration_ref = request_acceleration[3];
    data_unit[TJ_ID_5 - 1].acceleration_ref = request_acceleration[4];
    data_unit[TJ_ID_6 - 1].acceleration_ref = request_acceleration[5];
  }
  case WHOLE_FRAME:
  {
  }
  break;
  default:
  {
    return;
  }
  break;
  }
}
// 处理读取数据
void tj_set_status(TJ_Servo *pServo, Lower_Response *lower_response)
{
	static float current_angle, current_speed, current_load;
	uint8_t *response_pos = (uint8_t *)&lower_response->curr_joint_angle_1;
	uint8_t *response_load = (uint8_t *)&lower_response->curr_pressure_1;
	uint8_t *response_speed = (uint8_t *)&lower_response->current_speed_1;
	uint8_t *response_acceleration = (uint8_t *)&lower_response->curr_acceleration_1;
	uint8_t *response_temp = (uint8_t *)&lower_response->curr_temp_1;
	uint8_t *response_error_code = (uint8_t *)&lower_response->curr_error_code_1;
	//uint8_t *response_current = (int8_t *)&lower_response->current_mA_1;

	for (int i = TJ_ID_1; i < (TJ_ID_MAXIMUM); i++)
	{
			if(i == TJ_ID_1 || i == TJ_ID_6)
			{	
         response_pos[i-1] = map_2000_to_oxff(2000- pServo[i].tj_servo_data.stTjStatus_data.Current_position);				
			}
			else
			{
				response_pos[i-1] = map_2000_to_oxff(pServo[i].tj_servo_data.stTjStatus_data.Current_position);
			}
			//response_load[i - 1] = map_4000_to_oxff(pServo[i].tj_servo_data.stTjStatus_data.Current_torque);
			response_temp[i - 1] = pServo[i].tj_servo_data.stTjStatus_data.u8Tempture;
			//response_current[i -1] = pServo[i].tj_servo_data.stTjStatus_data.Current;
			response_error_code[i - 1] = pServo[i].tj_servo_data.stTjStatus_data.stFault.u8FaultInfo;
			// speed_get
//			current_speed = pServo[i].ft_servo_data.ft_058_current_velocity * 255 / (FT_ALL_MAX_SPEED - FT_ALL_MIN_SPEED);
//			response_speed[i - 1] = float_to_uint8_round(ABS(current_speed));
//			//通过读取电机电流，代替实时扭矩
//			current_load = pServo[i].ft_servo_data.ft_069_current * 255 / (FT_ALL_MAX_Torque);


			
//			// acceleration_get
//			response_acceleration[i - 1] = pServo[i].ft_servo_data.ft_041_acceleration;
	}
}

uint16_t time_count[TJ_ID_MAXIMUM];
uint8_t time_count_state[TJ_ID_MAXIMUM];
uint8_t torque_low_down_state[TJ_ID_MAXIMUM];
int16_t rec_pos_differ[TJ_ID_MAXIMUM];
int16_t rec_torque_differ[TJ_ID_MAXIMUM];
int16_t current_limit_set[TJ_ID_MAXIMUM] = {
																						DEFAULT_LIMIT_T, 
																						DEFAULT_LIMIT_T,
																						DEFAULT_LIMIT_T,
																						DEFAULT_LIMIT_T,
																						DEFAULT_LIMIT_T,
																						DEFAULT_LIMIT_T,
																						DEFAULT_LIMIT_T };

float lock_rotor_last_pos[TJ_ID_MAXIMUM];
void tj_locked_rotor_detection(TJ_Servo *pServo, TJ_Control_Data *tj_control_data)
{
	TJ_Control_Data_Unit *data_unit;
	for (uint8_t i = TJ_ID_1; i < TJ_ID_MAXIMUM; i++)
	{
		switch (i) 
		{
		case 1: data_unit = &tj_control_data->thumb; break;
		case 2: data_unit = &tj_control_data->index; break;
		case 3: data_unit = &tj_control_data->middle; break;
		case 4: data_unit = &tj_control_data->ring; break;
		case 5: data_unit = &tj_control_data->little; break;
		case 6: data_unit = &tj_control_data->thumb_yaw; break;
		default: continue;
		}
		if (data_unit->locked_rotor.time == 0)
		{
				continue;
		}
		rec_pos_differ[i] = lock_rotor_last_pos[i] - data_unit->angle;
		lock_rotor_last_pos[i] = data_unit->angle;
		if(abs(pServo[i].tj_servo_data.stTjStatus_data.Current) > data_unit->locked_rotor.threshold)
		{
			if (time_count[i] > data_unit->locked_rotor.time)
			{
				time_count[i] = 0;
				torque_low_down_state[i] = 1;
			}
			time_count[i]++;
		}
		int16_t safe_current = (pServo[i].tj_servo_data.stTjStatus_data.Current > 0) ? 
                    data_unit->locked_rotor.safe_current : 
                    data_unit->locked_rotor.safe_current;
	
		if (torque_low_down_state[i] == 1)
		{
			 if (current_limit_set[i] != safe_current)
			{
					position_mode_r(stTjCommAllInfo, i, safe_current , TJ_LIMIT_I); // 降低电流
					current_limit_set[i] = safe_current;
			}
		}
		// 如果位置有变化，说明电机不是堵住的，恢复原限流
		if (abs(rec_pos_differ[i]) > 50 && torque_low_down_state[i] == 1)
		{
				torque_low_down_state[i] = 0;
				time_count[i] = 0;
				position_mode_r(stTjCommAllInfo, i, DEFAULT_LIMIT_T , TJ_LIMIT_I);// 恢复默认限流值
				current_limit_set[i] = DEFAULT_LIMIT_T;
		}
	}
	
}