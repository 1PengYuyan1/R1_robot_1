/**
 * @file dvc_odrive.cpp
 * @brief ODrive简化版驱动实现
 */

#include "dvc_odrive.h"
#include <string.h>

/* 通信超时时间（毫秒） */
#define ODRIVE_COMM_TIMEOUT_MS     200

/**
 * @brief 简化初始化函数
 */
void Class_ODrive::Init(CAN_HandleTypeDef *hcan, uint8_t node_id, uint32_t baudrate)
{
    if (hcan->Instance == CAN1) {
        can_handle = &CAN1_Manage_Object;
    }

    /* 基础参数配置 */
    config.node_id = node_id;
    config.baudrate = baudrate;

    /* 数据初始化 */
    memset(&data, 0, sizeof(data));
    data.axis_state = ODRIVE_STATE_CLOSED_LOOP_CONTROL;
    data.is_connected = 0;

    Closed_loop();

    comm_timeout_counter = 0;
    last_comm_time = HAL_GetTick();
    Set_Axis_State(data.axis_state);

}

/**
 * @brief 设置轴状态
 */
void Class_ODrive::Set_Axis_State(Enum_ODrive_Axis_State state)
{
    uint8_t cmd_data[8] = {0};
    cmd_data[0] = (uint8_t)state;
    Send_Command(ODRIVE_CMD_SET_AXIS_STATE, cmd_data, 8);
}

/**
 * @brief 设置控制模式
 */
void Class_ODrive::Set_Control_Mode(Enum_ODrive_Control_Mode ctrl_mode,
                                  Enum_ODrive_Input_Mode input_mode)
{
    uint8_t cmd_data[8] = {0};
    cmd_data[0] = (uint8_t)ctrl_mode;    // 控制模式
    cmd_data[4] = (uint8_t)input_mode;   // 输入模式
    Send_Command(ODRIVE_CMD_SET_CONTROLLER_MODES, cmd_data, 8);
}

/**
 * @brief 设置位置控制
 */
void Class_ODrive::Set_Position(float position, float vel_ff, float torque_ff)
{
    uint8_t cmd_data[8];
    Float_To_Bytes(position, &cmd_data[0]);
    Float_To_Bytes(vel_ff, &cmd_data[4]);
    Send_Command(ODRIVE_CMD_SET_INPUT_POS, cmd_data, 8);

}

/**
 * @brief 设置速度控制
 */
void Class_ODrive::Set_Velocity(float velocity, float torque_ff)
{
    uint8_t cmd_data[8];
    Float_To_Bytes(velocity, &cmd_data[0]);
    Float_To_Bytes(torque_ff, &cmd_data[4]);
    Send_Command(ODRIVE_CMD_SET_INPUT_VEL, cmd_data, 8);
}

/**
 * @brief 设置扭矩控制
 */
void Class_ODrive::Set_Torque(float torque)
{
    uint8_t cmd_data[4];
    Float_To_Bytes(torque, cmd_data);
    Send_Command(ODRIVE_CMD_SET_INPUT_TORQUE, cmd_data, 4);
}

/**
 * @brief 设置限制参数
 */
void Class_ODrive::Set_Limits(float vel_limit, float current_limit)
{
    uint8_t cmd_data[8];
    Float_To_Bytes(vel_limit, &cmd_data[0]);
    Float_To_Bytes(current_limit, &cmd_data[4]);
    Send_Command(ODRIVE_CMD_SET_LIMITS, cmd_data, 8);
}

/**
 * @brief 闭环模式
 */
void Class_ODrive::Closed_loop()
{
    Set_Axis_State(ODRIVE_STATE_CLOSED_LOOP_CONTROL);
}

/**
 * @brief 紧急停止
 */
void Class_ODrive::Emergency_Stop()
{
    Set_Axis_State(ODRIVE_STATE_IDLE);
}

/**
 * @brief 清除错误
 */
void Class_ODrive::Clear_Errors()
{
    uint8_t cmd_data[8] = {0};
    Send_Command(ODRIVE_CMD_CLEAR_ERRORS, cmd_data, 8);
}

/**
 * @brief 请求编码器数据
 */
void Class_ODrive::Request_Encoder_Data()
{
    uint8_t cmd_data[8] = {0};
    Send_Command_RTR(ODRIVE_CMD_GET_ENCODER_ESTIMATES, cmd_data, 8);
}

/**
 * @brief 请求总线电压
 */
void Class_ODrive::Request_Bus_Voltage()
{
    uint8_t cmd_data[8] = {0x10,0x00,0x00,0x00,0x10,0x00,0x00,0x00};
    Send_Command(ODRIVE_CMD_GET_BUS_VOLTAGE, cmd_data, 8);
}

/**
 * @brief 请求IQ数据
 */
void Class_ODrive::Request_IQ_Data()
{
    uint8_t cmd_data[8] = {0};
    Send_Command(ODRIVE_CMD_GET_IQ, cmd_data, 8);
}

/**
 * @brief CAN接收回调
 */
void Class_ODrive::CAN_RxCpltCallback(uint8_t *rx_data)
{

    uint32_t can_id = can_handle->Rx_Buffer.Header.StdId;
    uint8_t node_id = (can_id >> 5) & 0x3F;
    // Enum_ODrive_Command cmd = (Enum_ODrive_Command)(can_id & 0x1F);
    Enum_ODrive_Command cmd = ODRIVE_CMD_GET_ENCODER_ESTIMATES;

    if (node_id == config.node_id) {
        Process_Response(cmd, rx_data);
        data.last_update = HAL_GetTick();
        data.is_connected = 1;
    }
}
/**
 * @brief TIM定时器中断计算回调函数, 计算周期取决于电机反馈周期
 *
 */
void Class_ODrive::TIM_Send_PeriodElapsedCallback() {


    if ( data.motor_mode== ODRIVE_CTRL_VOLTAGE) {

        Set_Velocity(data.target_omage,data.target_torque);
    }

    else if (data.motor_mode == ODRIVE_CTRL_TORQUE) {

    }

    else if(data.motor_mode == ODRIVE_CTRL_VELOCITY){

    }

    else if(data.motor_mode == ODRIVE_CTRL_POSITION){

    }
}

/**
 * @brief 定时存活检查
 */
void Class_ODrive::TIM_Alive_CheckCallback()
{
    uint32_t current_time = HAL_GetTick();

    if (current_time - data.last_update > ODRIVE_COMM_TIMEOUT_MS) {
        data.is_connected = 0;
    }
}

/**
 * @brief 发送CAN命令
 */
void Class_ODrive::Send_Command(Enum_ODrive_Command cmd, const uint8_t *data, uint8_t len)
{

    uint32_t can_id = ((config.node_id << 5) | cmd) & 0xFFF;
    memcpy(tx_buffer, data, len);
    CAN_Send_Data(can_handle->CAN_Handler, can_id, tx_buffer, 8);

}
/**
 * @brief 发送远程帧CAN命令
 */
void Class_ODrive::Send_Command_RTR(Enum_ODrive_Command cmd, const uint8_t *data, uint8_t len)
{

    uint32_t can_id = ((config.node_id << 5) | cmd) & 0xFFF;
    memcpy(tx_buffer, data, len);
    CAN_Send_Data_RTR(can_handle->CAN_Handler, can_id, tx_buffer, 8);

}
/**
 * @brief 处理响应数据
 */
void Class_ODrive::Process_Response(Enum_ODrive_Command cmd, const uint8_t *rx_data)
{
    switch (cmd) {
        case ODRIVE_CMD_GET_ENCODER_ESTIMATES:
            data.position = Bytes_To_Float(&rx_data[0]);
            data.omega = Bytes_To_Float(&rx_data[4]);
            break;

        case ODRIVE_CMD_GET_BUS_VOLTAGE:
            data.bus_voltage = Bytes_To_Float(rx_data);
            break;

        case ODRIVE_CMD_GET_IQ:
            data.iq_setpoint = Bytes_To_Float(&rx_data[0]);
            data.iq_measured = Bytes_To_Float(&rx_data[4]);
            break;

        case ODRIVE_CMD_SET_AXIS_STATE:
            data.axis_state = (Enum_ODrive_Axis_State)rx_data[0];
            break;

        default:
            break;
    }
}

/**
 * @brief 浮点数转字节数组
 */
void Class_ODrive::Float_To_Bytes(float value, uint8_t *bytes)
{
    union {
        float f;
        uint8_t b[4];
    } converter;

    converter.f = value;
    memcpy(bytes, converter.b, 4);
}

/**
 * @brief 字节数组转浮点数
 */
float Class_ODrive::Bytes_To_Float(const uint8_t *bytes)
{
    union {
        float f;
        uint8_t b[4];
    } converter;

    memcpy(converter.b, bytes, 4);
    return converter.f;
}