#ifndef __TJ_MOTORDRIVE_H_
#define __TJ_MOTORDRIVE_H_

#include <FreeRTOS.h>
#include <stm32g4xx_hal.h>
#include <stdbool.h>


#define YS_INS_FRAME_HEAD 0X55AA//指令帧帧头
#define YS_ANS_FRAME_HEAD 0XAA55//应答帧帧头

#define FRAME_TIMEOUT_MS   50      // 拼帧超时时间(ms)

#define FRAME_SIZE         22      // 固定帧长度
#define TJ_DATA_RECV_LEN_MAX 64
#define TJ_DATA_SEND_LEN_MAX 67
#define TJ_MOTOR_NUM_MAX 17
#define min(a, b) ((a) < (b) ? (a) : (b))
#define WAIT_TIME_MAXIMUM 100      //在需要等待返回帧时，等待的最长时间。超时则跳过等待，接收失败

typedef struct
{
	__IO uint8_t send_buf[TJ_DATA_SEND_LEN_MAX];
	__IO uint8_t send_len;
	__IO bool bSend_en_flag;
	__IO bool bNeed_block;						 // 需要阻塞标志，如果需要等待返回帧，需要主动置起该标志位。阻塞操作完成后自动清除
	__IO int32_t block_count;					 // 阻塞状态计数，发送时阻塞+1，接收到数据后阻塞-1，如果该值在0~1之间变化，那么通信运行时序稳定。电机也未掉线。
	__IO uint32_t block_timeout_count; // 阻塞时间溢出
	uint32_t nop_count;								 // 指令之间需要有一定延迟，不然推杆不响应指令。
	uint32_t usart_rx_fault;					 // 串口接收发生错误计数
	uint8_t receive_buf[TJ_DATA_RECV_LEN_MAX];
	uint8_t receive_len;
	__IO bool bReceive_flag;
} tjTransmit_t;

/*
帧头			帧长度		ID号			指令号			控制表索引			数据段			校验和
2字节			一字节		一字节		一字节			一字节					n字节				一字节
*/
/*
帧长度 = n+2
*/
typedef enum
{
	TJ_CMD_RD = 0X01,										 // 查询控制表内的数据
	TJ_CMD_WR = 0X02,										 // 向控制表内写入数据
	TJ_CMD_POSITION_R = 0X21,						 // 定位模式（反馈状态信息）
	TJ_CMD_BROADCAST_POSITION_NR = 0XF2, // 广播定位模式（无反馈）内含N个电缸位置信息
	TJ_CMD_MC = 0X04,										 // 实现对电缸的功能控制
} tjCmdTy_t;

typedef enum
{
	TJ_TARGET_POS = 0x37,
	TJ_TARGET_SPEED = 0X38,						//目标速度
	TJ_TARGET_TORQUE = 0X39,					//目标力矩
	TJ_TARGET_KP = 0X40,							//kp
	TJ_TARGET_KD = 0X41,							//kd
	TJ_LIMIT_I = 0X42,								//最大堵转电流
	TJ_CUR_KPKD_ACK = 0X51,           //kp.kd查询
	TJ_TORQUE_ACK = 0x52,							//力矩指令返回
	TJ_SPEED_ACK = 0x53,							//速度指令返回
	TJ_MAX_LIMIT_ACK = 0X54,          //查询最大电流值
	
}tjColTable_index;

typedef enum
{
	/*															偏移地址			名称						                  注释																	权限				默认值		*/
	TJ_TORQUE_CMD_TYPE = 0,		 /*  0-1      Tq_callType                            -                                      -          0xF     */
	TJ_POLE_PAIRS = 2,				 /*  2-3      NPP                                    极对数                                 -          -       */
	TJ_MOTOR_ID = 4,					 /*  4        电机ID                                 0x00-0xFE                              R/W        0x0.    */
	TJ_UART_BAUD = 5,					 /*  5        UART波特率                             0~19200,1~57600,2~115200,3~921600       -          -       */
	TJ_OTP_THRESHOLD = 6,			 /*  6-9      过温保护值                             [回温启动温度-5,80]°C                   R          -       */
	TJ_OTP_RECOVERY = 10,			 /*  10-13    回温启动设置                           [20,过温保护上限+5]                     R          -       */
	TJ_TARGET_POSITION = 0x37, /*  14-15    目标位置设置                           0-2000                                R/W         -       */
	TJ_FORCE_SENSOR_CAL = 16,	 /*  16-17    力传感器位校准设置                     可设置范围1                            R/W         -       */
	TJ_OCP_THRESHOLD = 18,		 /*  18-21    过流保护电流值                         0.3-3.5A                              R          -       */
	TJ_RESERVED_22 = 22,			 /*  22-23    reserve                                -                                      -          -       */
	TJ_QDEC_TX_MODE = 24,			 /*  24-25    tx_qd_mode                            -                                      -          -       */
	TJ_MCU_OTP = 26,					 /*  26-29    MCU过温保护                           mcuWarnTempFilter                       -          -       */
	TJ_POS_KP = 30,						 /*  30-33    Kp                                    位置环Kp                               R/W         -       */
	TJ_POS_KD = 34,						 /*  34-37    kd                                    位置环kd                               R/W         -       */
	TJ_TORQUE_CONSTANT = 38,	 /*  38-41    kt                                    力矩系数(0~100)                        R/W         -       */
	TJ_ENC_OFFSET = 42,				 /*  42-45    Epos_offset                           初始角度(-6.28~6.28)                   R/          -       */
	TJ_RUN_MODE = 46,					 /*  46-47    run_mode                              运行模式:0~15                         R/W         -       */
	TJ_CTRL_STATE = 48,				 /*  48-49    state                                 控制状态:0~4                          R/W         -       */
	TJ_FAULT_STATUS = 50,			 /*  50-53    faultSta                              故障字:bit32/上位机解析故障             R          -       */
	TJ_CSP_KP = 54,						 /*  54-57    loc_kp                                csp:kp:(1~50)                          -          -       */
	TJ_VEL_KP = 58,						 /*  58-61    spd_kp                                速度环kp(1~50)                         -          -       */
	TJ_VEL_KI = 62,						 /*  62-65    spd_ki                                速度环ki(0.0001~1)                     -          -       */
	TJ_MAX_VELOCITY = 66			 /*  66-69    max_spd                               最大速度(1~2000rad/s)                   -          -       */
} tjMemoryTable_t;

typedef enum
{
	TJ_SIG_RUNNING = 0X04,			// 工作
	TJ_SIG_STOP = 0X23,					// 急停
	TJ_SIG_PAUSE = 0X14,				// 暂停
	TJ_SIG_SAVE = 0X20,					// 参数装订
	TJ_SIG_QUERY_STATUS = 0X22, // 查询电缸状态信息（bit）包括目标位置、当前位置、温度、电机驱动电流以及异常信息
	TJ_SIG_CLEAR_FAULT = 0X1E,	// 故障清除，当电缸发生过流、堵转、电机异常故障时可通过该指令清除故障码，恢复电缸的正常工作
	TJ_SIG_HALL_CALIB_MODEL = 0x55,
	TJ_SIG_CALIB_MODE_ENCODER = 0X51,
} tjSigleCtrlInstr_t;

typedef struct
{
	uint16_t u16Head;																						//包头
	uint8_t u8Len;  																						//当前接收长度
	uint16_t frame_start_pos; 																	// 记录帧头在u8receive_data中的位置
	uint8_t u8ID;
	tjCmdTy_t stCmdType;
	tjColTable_index stTableIndex;
	uint8_t receive_buf[FRAME_SIZE];														//完整包
	uint8_t u8receive_data[TJ_DATA_RECV_LEN_MAX];								//接收缓冲区
	uint8_t u8SumCheck;
	bool assembling;                            								// 正在拼帧标志
  uint32_t last_receive_time;                 								// 最后接收时间(用于超时)
} tjAnswerFrame_t;

typedef union
{
	struct
	{
		uint8_t locked_rotor_t : 1;		// 堵转
		uint8_t overTemp : 1;				// 过温
		uint8_t overcurrent : 1;		// 过流
		uint8_t motor_abnormal : 1; // 电机异常（失速）
		//	  uint8_t volatile_abnormal:1;  //电压异常
		//	  uint8_t CurAbnormal:1;        //电流自检异常
		//	  uint8_t PosCheckFail:1;       //位置自检异常
		uint8_t reserved : 4; // 占位
	} FaultBit_t;
	uint8_t u8FaultInfo;
} tjFault_t;

// 存储电机的配置参数信息（内存表参数）
typedef struct
{
	uint16_t u16TorqueCmdType;	/* [0-1] Tq_callType: - (权限: -, 默认值: 0xF) */
	uint16_t u16PolePairs;			/* [2-3] NPP: 极对数 (权限: -) */
	uint8_t u8MotorID;					/* [4] 电机ID: 0x00-0xFE (权限: R/W, 默认值: 0x0.) */
	uint8_t u8UartBaud;					/* [5] UART波特率: 0~19200,1~57600,2~115200,3~921600 (权限: -) */
	float fOtpThreshold;				/* [6-9] 过温保护值: [回温启动温度-5,80]°C (权限: R) */
	float fOtpRecovery;					/* [10-13] 回温启动设置: [20,过温保护上限+5] (权限: R) */
	uint16_t u16TargetPosition; /* [14-15] 目标位置设置: 0-2000 (权限: R/W) */
	uint16_t u16ForceSensorCal; /* [16-17] 力传感器位校准设置: 可设置范围1 (权限: R/W) */
	float fOcpThreshold;				/* [18-21] 过流保护电流值: 0.3-3.5A (权限: R) */
	uint16_t u16Reserved22;			/* [22-23] reserve: - (权限: -) */
	uint16_t u16QdecTxMode;			/* [24-25] tx_qd_mode: - (权限: -) */
	float fMcuOtp;							/* [26-29] MCU过温保护: mcuWarnTempFilter (权限: -) */
	float fPosKp;								/* [30-33] Kp: 位置环Kp (权限: R/W) */
	float fPosKd;								/* [34-37] kd: 位置环kd (权限: R/W) */
	float fTorqueConstant;			/* [38-41] kt: 力矩系数(0~100) (权限: R/W) */
	float fEncoderOffset;				/* [42-45] Epos_offset: 初始角度(-6.28~6.28) (权限: R/) */
	uint16_t u16RunMode;				/* [46-47] run_mode: 运行模式:0~15 (权限: R/W) */
	uint16_t u16CtrlState;			/* [48-49] state: 控制状态:0~4 (权限: R/W) */
	uint32_t u32FaultStatus;		/* [50-53] faultSta: 故障字:bit32/上位机解析故障 (权限: R) */
	uint32_t u32CspKp;					/* [54-57] loc_kp: csp:kp:(1~50) (权限: -) */
	uint32_t u32SpdKp;					/* [58-61] spd_kp: 速度环kp(1~50) (权限: -) */
	float fSpdKi;								/* [62-65] spd_ki: 速度环ki(0.0001~1) (权限: -) */
	uint32_t u32MaxVelocity;		/* [66-69] max_spd: 最大速度(1~2000rad/s) (权限: -) */
} tjConfigData_t;

// 存储电机的状态信息
typedef struct
{
	uint16_t u16Target_position;	// 目标位置
	int16_t Current_position; // 当前位置  返回的当前位置存在负数
	int8_t u8Tempture;						// 温度
	int16_t Current;					// 电流  存在负数
	// uint8_t  u8VolaValue;             //电压值
	uint16_t u16SpeedRpm; // 转速
	uint16_t u16OC_Limit; // 过流保护阈值
	tjFault_t stFault;		// 电机错误信息反馈
} tjStatusData_t;
//解析状态
typedef enum
{
	PARSING_LEISURE = 0,      /*解析空闲，未开始解析*/
	PARSING_SUCCESS = 1,			/*返回报文解析成功*/
	TIMEOUT = 2,
	PARSING = 3,
	HEAD_NO_MATCH = 4,				//帧头不匹配
	ID_NO_MATCH = 5,					//设备id不匹配
	SUM_CHECK_NO_MATCH = 6,		//和校验不匹配
	LENGTH_ERROR = 7,         //越界	
}Parsing_State;
typedef struct
{
	uint8_t no_available_data : 1;		// 没有可用的数据
	uint8_t unreasonable_length : 1;	// 不合理的数据长度
	uint8_t no_match_head : 1;				// 帧头不符
	uint8_t no_match_data_len : 1;		// 不符合解析过程的帧长度
	uint8_t no_match_id : 1;					// id不符
	uint8_t no_match_cmd_type : 1;		// 没有相符合的指令类型
	uint8_t no_match_table_index : 1; // 没有匹配的控制表索引
	uint8_t no_match_sum_check : 1;		// 不正确的和校验
	uint8_t rev_timeout : 1;
} tjDecode_Fault;

typedef struct
{
	tjConfigData_t stTjConfig_data;
	tjStatusData_t stTjStatus_data;
	Parsing_State decode_state; // 解码状态
} tjData;

typedef struct
{
	uint32_t send_time;					 // 发送时刻
	uint32_t receive_time;			 // 接收时刻
	uint8_t get_sem_error;			 // 获取信号量超时错误
	tjDecode_Fault decode_fault; // 解码过程故障
} tjCommDetection_t;

typedef enum
{
	INVALID_MODE = 0,
	WAIT_FLAG = 1,
	WAIT_SEM_NOP = 2,
} TRANS_BLOCK_MODE;

typedef struct
{
	tjTransmit_t *pstTjTransmit;
	tjAnswerFrame_t *pstTjAnswer_frame;
	tjCommDetection_t stTjCommDetection;
	TRANS_BLOCK_MODE Tjtrans_block_mode; 		// 传输时序阻塞模式，收发过程在374us左右，如果不想追求极致的高频控制，使用延时去控制时序间隔更适合.
} tjComm_t;

typedef struct
{
	uint16_t thumb_target_position;			// 目标位置 thumb;
	uint16_t index_target_position;			// 目标位置 index;
	uint16_t middle_target_position;		// 目标位置 middle;
	uint16_t ring_target_position;			// 目标位置 ring;
	uint16_t little_target_position;		// 目标位置 little;
	uint16_t thumb_aux_target_position; // 目标位置 thumb_aux;
} HandTargetPosition_t;

extern tjData tjMotorParsedData[TJ_MOTOR_NUM_MAX]; // 存放电机解析后的数据
extern tjComm_t stTjCommAllInfo[3];									 // 关于电机控制、数据存储的总结构体。(一个串口控制两个电机为保证每个串口都用独立信号量所以创建三个结构体)

extern HandTargetPosition_t tjTarPosition; // 手部控制结构体
extern tjTransmit_t stTjMotorTransmit; // 电机通信发送结构体

bool decode_mc_ans_frame(tjComm_t *pTjComm, tjData *pTjData);
void read_table(tjComm_t *pTjComm, uint8_t id, tjColTable_index MemtableIndex);
void position_mode_r(tjComm_t *pTjComm, uint8_t id, int16_t target_position, tjColTable_index tj_table);
bool HAL_UART_Receive_IDLE(UART_HandleTypeDef *huart, tjAnswerFrame_t *tran_handle, uint16_t Size);
#endif
