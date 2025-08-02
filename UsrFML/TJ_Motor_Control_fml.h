#ifndef _TJ_MOTOR_CONTROL_FML_H
#define _TJ_MOTOR_CONTROL_FML_H

#include "TJ_MotorDrive.h"
#include "upper_can_comm_drv.h"
#include "data_structure.h"
#include "upper_comm_protocol_fml.h"

#define JOINT_LOCK_THRESHOLD 500	 // 堵转阈值
#define JOINT_LOCK_TIME 50				 // 堵转时间
#define JOINT_LOCK_TORQUE 200			 // 堵转电流
#define MAX_LOCKED_ROTOR_TIME 2550 // 最大堵转时间
#define MAX_WAIT_ACK_TIME 500			 //上电后读取当前位置最大等待返回帧时间
#define DEFAULT_LIMIT_T 0x044C		 //电机默认电流1.1A
/*飞特舵机设备地址码*/
typedef enum
{
  INVALID_TJ_ID = 0,
  TJ_ID_1 = 1,
  TJ_ID_2 = 2,
  TJ_ID_3 = 3,
  TJ_ID_4 = 4,
  TJ_ID_5 = 5,
  TJ_ID_6 = 6,
  TJ_ID_MAXIMUM,
} TJ_SERVO_ID;

typedef struct
{
  bool is_servo_valid;
  bool is_servo_online;
  bool go_home;
  bool go_home_success;
  int16_t go_home_speed;
  float virtual_compensation; // 找零点过程虚伪补偿
  float pos_limit_min;        // 0-4095
  float pos_limit_max;        // 0-4095

  TJ_SERVO_ID servo_id;
  tjData tj_servo_data;
  uint8_t motor_fault_code; // 电机故障码,最低位是堵转
} TJ_Servo;

typedef struct
{
	uint16_t threshold; // 堵转阈值
	uint16_t time;			// 堵转时间
	uint16_t safe_current;		// 堵转扭矩
} LockedRotorParams;

typedef struct
{
  float angle;                   // 电机角度，角度单位
  float last_angle;              // 上一个角度设定值
  float angle_min;               // 电机最小角度，角度单位
  float angle_max;               // 电机最大角度，角度单位
  float init_angle;              // 初始位置，角度单位
  uint16_t torque_lim;           // 电机最大转矩				无单位，转矩范围0-1000
  uint16_t last_torque_lim;      // 上一个转矩设定
  int16_t speed_ref;             // 速度设定
  int16_t last_speed_ref;        // 上一个速度设定
  int16_t speed_max;             // 速度设定最小值
  int16_t speed_min;             // 速度设定最大值
  uint8_t acceleration_ref;      // 加速度设定
  uint8_t last_acceleration_ref; // 上一个加速度设定
  bool press_sensor_calibration; // 力传感器校准（上位机命令）
  bool dir_flip;                 // 运动方向反翻转
	LockedRotorParams locked_rotor;
  int16_t target_torque;
  int16_t last_target_torque;
} TJ_Control_Data_Unit;

typedef struct
{
  TJ_Control_Data_Unit thumb;     // 拇指
  TJ_Control_Data_Unit index;     // 食指
  TJ_Control_Data_Unit middle;    // 中指
  TJ_Control_Data_Unit ring;      // 无名指
  TJ_Control_Data_Unit little;    // 小拇指
  TJ_Control_Data_Unit thumb_yaw; // 大拇指横摆
} TJ_Control_Data;

extern TJ_Servo tj_servo[TJ_ID_MAXIMUM];
extern TJ_Control_Data tj_control_data;

void clear_usart2_fault(void);
void clear_usart3_fault(void);
void clear_usart4_fault(void);
	
void rx_data_2tj_servo(tjData *pProtocol, TJ_Servo *pServo);
void tj_get_cmd(TJ_Servo *pServo, TJ_Control_Data *ft_control_data, Upper_Request *upper_request, Protocol_Aux_Data *aux_data);
void TJ_Init_Pos(TJ_Servo *pServo, TJ_Control_Data *tj_control_data);
void TJ_Motor_Control(TJ_Servo *pServo, TJ_Control_Data *ft_control_data);
void tj_locked_rotor_detection(TJ_Servo *pServo, TJ_Control_Data *tj_control_data);
void tj_set_status(TJ_Servo *pServo, Lower_Response *lower_response);
#endif // _TJ_MOTOR_CONTROL_FML_H
