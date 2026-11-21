#ifndef MOTORSTRUCTS_H
#define MOTORSTRUCTS_H

#include <stdint.h>

// 电机控制参数范围定义
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -45.0f
#define V_MAX 45.0f
#define T_MIN -18.0f
#define T_MAX 18.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define KI_MIN 0.0f
#define KI_MAX 100.0f

// 编码器相关定义
#define ENCODER_POSITION_MIN 0			// 32位位置值最小
#define ENCODER_POSITION_MAX 0xFFFFFFFF // 32位位置值
#define ENCODER_ID_MIN 0				// 最小ID
#define ENCODER_ID_MAX 255				// 最大ID
#define ENCODER_BAUDRATE_DEFAULT 500000 // 默认500kbps

// CAN指令功能码定义
#define ENCODER_CMD_READ_POSITION 0x01	 // 读取位置命令
#define ENCODER_CMD_SET_ZERO 0x02		 // 设置零点命令
#define ENCODER_CMD_SET_ID 0x03			 // 设置ID命令
#define ENCODER_CMD_SET_BAUDRATE 0x04	 // 设置波特率命令
#define ENCODER_CMD_READ_STATUS 0x05	 // 读取状态命令
#define ENCODER_CMD_RESTORE_FACTORY 0x06 // 恢复出厂设置命令

// ODrive状态定义 (参考文档2中的interfaces.hpp)
enum ODriveAxisState_t
{
	AXIS_STATE_UNDEFINED = 0,				   // 未定义状态
	AXIS_STATE_IDLE = 1,					   // 空闲状态
	AXIS_STATE_STARTUP_SEQUENCE = 2,		   // 启动序列
	AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3,  // 完整校准序列
	AXIS_STATE_MOTOR_CALIBRATION = 4,		   // 电机校准
	AXIS_STATE_SENSORLESS_CONTROL = 5,		   // 无传感器控制
	AXIS_STATE_ENCODER_INDEX_SEARCH = 6,	   // 编码器索引搜索
	AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7, // 编码器偏移校准
	AXIS_STATE_CLOSED_LOOP_CONTROL = 8		   // 闭环控制
};

// 控制模式定义 (参考文档2中的interfaces.hpp)
enum ODriveControlMode_t
{
	CONTROL_MODE_VOLTAGE_CONTROL = 0,  // 电压控制模式
	CONTROL_MODE_TORQUE_CONTROL = 1,   // 力矩控制模式
	CONTROL_MODE_VELOCITY_CONTROL = 2, // 速度控制模式
	CONTROL_MODE_POSITION_CONTROL = 3  // 位置控制模式
};

// 输入模式定义
enum ODriveInputMode_t
{
	INPUT_MODE_INACTIVE = 0,	 // 非活动模式
	INPUT_MODE_PASSTHROUGH = 1,	 // 直通模式
	INPUT_MODE_VEL_RAMP = 2,	 // 速度斜坡模式
	INPUT_MODE_POS_FILTER = 3,	 // 位置滤波模式
	INPUT_MODE_MIX_CHANNELS = 4, // 混合通道模式
	INPUT_MODE_TRAP_TRAJ = 5	 // 斜坡轨迹模式
};

// ODrive命令ID定义 (参考文档1)
enum ODriveCommandID_t
{
	ODRIVE_CMD_SET_AXIS_STATE = 0x007,		  // 设置轴状态
	ODRIVE_CMD_SET_CONTROLLER_MODES = 0x00B,  // 设置控制器模式
	ODRIVE_CMD_SET_INPUT_POS = 0x00C,		  // 设置位置输入
	ODRIVE_CMD_GET_ENCODER_ESTIMATES = 0x009, // 获取编码器估计值
	ODRIVE_CMD_GET_BUS_VOLTAGE = 0x017,		  // 获取总线电压
	ODRIVE_CMD_REBOOT = 0x016,				  // 重启命令
	ODRIVE_CMD_SET_INPUT_VEL = 0x00D		  // 设置速度输入
};

// 编码器波特率定义
typedef enum
{
	ENCODER_BAUDRATE_9600 = 0,
	ENCODER_BAUDRATE_19200,
	ENCODER_BAUDRATE_38400,
	ENCODER_BAUDRATE_57600,
	ENCODER_BAUDRATE_115200,
	ENCODER_BAUDRATE_100000,
	ENCODER_BAUDRATE_125000,
	ENCODER_BAUDRATE_250000,
	ENCODER_BAUDRATE_500000,
	ENCODER_BAUDRATE_1000000
} EncoderBaudrate_t;

// 达妙电机数据结构
typedef struct
{
	uint16_t DM_id;
	float DM_position;
	float DM_speed;
	float DM_kp;
	float DM_kd;
	float DM_torque;
	float DM_max_position;
	float DM_min_position;
} Motor_DM_CAN_Send_Struct;

typedef struct
{
	uint16_t DM_ERR;
	uint16_t DM_current_position;
	uint16_t DM_current_speed;
	uint16_t DM_current_torque;
	uint16_t DM_current_temp_MOS;
	uint16_t DM_current_temp_Rotor;

	float DM_current_position_f;
	float DM_current_speed_f;
	float DM_current_torque_f;
} Motor_DM_CAN_Recieve_Struct;

// 大疆电机数据结构
typedef struct
{
	uint16_t DJI_id;
	float DJI_position;
	float DJI_speed;
	float DJI_kp;
	float DJI_kd;
	float DJI_ki;
	float DJI_max_I;
	float DJI_min_I;
} Motor_DJI_CAN_Send_Struct;

typedef struct
{
	uint16_t DJI_last_position;
	uint16_t DJI_current_position;
	uint16_t DJI_current_speed;
	uint16_t DJI_current_Temperature;
	uint16_t DJI_current_I;

	float DJI_current_position_f;
	float DJI_current_speed_f;
	float DJI_current_Temperature_f;
} Motor_DJI_CAN_Recieve_Struct;

// 编码器数据结构 - 新增完善
typedef struct
{
	uint16_t Encoder_id;
	uint32_t Encoder_position;			// 32位绝对位置值
	uint16_t Encoder_multi_turn;		// 多圈值
	uint16_t Encoder_single_turn;		// 单圈值
	EncoderBaudrate_t Encoder_baudrate; // 波特率设置
	uint8_t Encoder_resolution;			// 分辨率设置
} Encoder_CAN_Send_Struct;

typedef struct
{
	uint16_t Encoder_ERR;
	uint32_t Encoder_current_position;	  // 32位当前位置
	uint16_t Encoder_current_multi_turn;  // 多圈当前位置
	uint16_t Encoder_current_single_turn; // 单圈当前位置
	uint8_t Encoder_status;				  // 状态字节
	uint8_t Encoder_temperature;		  // 温度值
	uint16_t Encoder_voltage;			  // 电压值
	uint16_t Encoder_current_angle_deg;
	uint16_t Encoder_current_angle_rad;

	uint8_t Encoder_id;
	uint8_t Encoder_function_code;
	uint8_t Encoder_data_length;

	float Encoder_current_position_f; // 浮点位置值
} Encoder_CAN_Recieve_Struct;

// ODrive CAN发送数据结构
typedef struct ODrive_CAN_Send_Struct
{
	uint8_t node_id;			  // 节点ID (0-63)
	bool extended_id;			  // 是否使用扩展ID
	ODriveCommandID_t command_id; // 命令ID
} ODrive_CAN_Send_Struct;

// ODrive CAN接收数据结构
typedef struct ODrive_CAN_Receive_Struct
{
	uint8_t node_id;
	ODriveCommandID_t command_id;
	float position;	   // 编码器位置
	float velocity;	   // 编码器速度
	float bus_voltage; // 总线电压
	uint8_t data[8];   // 原始数据
} ODrive_CAN_Recieve_Struct;

// CAN总线结构体
typedef struct
{
	Motor_DM_CAN_Send_Struct ID_1_motor_DM_send;
	Motor_DM_CAN_Recieve_Struct ID_1_motor_DM_recieve;
} CANable_DM_CAN_Bus_Struct;

typedef struct
{
	Motor_DJI_CAN_Send_Struct ID_1_motor_DJI_send, ID_2_motor_DJI_send;
	Motor_DJI_CAN_Recieve_Struct ID_1_motor_DJI_recieve, ID_2_motor_DJI_recieve;
} CANable_DJI_CAN_Bus_Struct;

typedef struct
{
	Encoder_CAN_Send_Struct ID_1_Encoder_send, ID_2_ODrive_Send;
	Encoder_CAN_Recieve_Struct ID_1_Encoder_recieve;
} CANable_Encoder_CAN_Bus_Struct;

typedef struct
{
	ODrive_CAN_Send_Struct ID_1_ODrive_Send;
	ODrive_CAN_Recieve_Struct ID_1_ODrive_recieve;
} CANable_ODrive_CAN_Bus_Struct;

// 全局变量声明（使用extern）
extern CANable_DM_CAN_Bus_Struct CANable0_DM_CAN_Bus;
extern Motor_DM_CAN_Recieve_Struct CANable0_DM_DEV0_RX;

extern CANable_DJI_CAN_Bus_Struct CANable0_DJI_CAN_Bus;
extern Motor_DJI_CAN_Recieve_Struct CANable0_DJI_DEV0_RX;

extern CANable_Encoder_CAN_Bus_Struct CANable0_Encoder_CAN_Bus;
extern Encoder_CAN_Recieve_Struct CANable0_Encoder_DEV0_RX;

extern CANable_ODrive_CAN_Bus_Struct CANable0_ODrive_CAN_Bus;
extern ODrive_CAN_Recieve_Struct CANable0_ODrive_DEV0_RX;
#endif