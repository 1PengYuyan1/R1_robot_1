#include "ActuatorDM_control.h"

Motor_DM_Control::Motor_DM_Control(CANDeviceManager *canManager) : canManager_DM(canManager)
{
    // 先初始化 CAN 管理器
    if (!canManager_DM)
    {
        std::cout << "警告: CANDeviceManager 指针为空，请在后续设置" << std::endl;
    }

    CANable_DM_CAN_Bus_Init();

    // 只有在 CAN 管理器有效时才使能电机
    if (canManager_DM)
    {
        // ENABLE_ALL_DM_Actuator(1000);
        std::cout << "DM电机使能完成" << std::endl;
    }
}
Motor_DM_Control::~Motor_DM_Control()
{
    if (canManager_DM)
    {
        // DISABLE_ALL_Actuator_DM(1000);
    }
}
void Motor_DM_Control::CANable_DM_CAN_Bus_inti_set(CANable_DM_CAN_Bus_Struct *CAN_DM_Bus)
{
    CAN_DM_Bus->ID_1_motor_DM_send.DM_id = 0X08;
}

void Motor_DM_Control::CANable_DM_CAN_Bus_Init()
{
    CANable_DM_CAN_Bus_inti_set(&CANable0_DM_CAN_Bus);
}
/// @brief 使能
/// @param dev
/// @param channel
/// @param Motor_DM_Data
void Motor_DM_Control::Actuator_DM_Enable(Motor_DM_CAN_Send_Struct *Motor_DM_Data)
{

    Data_DM_CAN[0] = 0xFF;
    Data_DM_CAN[1] = 0xFF;
    Data_DM_CAN[2] = 0xFF;
    Data_DM_CAN[3] = 0xFF;
    Data_DM_CAN[4] = 0xFF;
    Data_DM_CAN[5] = 0xFF;
    Data_DM_CAN[6] = 0xFF;
    Data_DM_CAN[7] = 0xFC;

    canManager_DM->CANable0_sendFrame(Motor_DM_Data->DM_id, Data_DM_CAN, 8);
}

/// @brief 电机失能
/// @param dev
/// @param channel
/// @param Motor_DM_Data
void Motor_DM_Control::Actuator_DM_Disable(Motor_DM_CAN_Send_Struct *Motor_DM_Data)
{

    Data_DM_CAN[0] = 0xFF;
    Data_DM_CAN[1] = 0xFF;
    Data_DM_CAN[2] = 0xFF;
    Data_DM_CAN[3] = 0xFF;
    Data_DM_CAN[4] = 0xFF;
    Data_DM_CAN[5] = 0xFF;
    Data_DM_CAN[6] = 0xFF;
    Data_DM_CAN[7] = 0xFD;

    canManager_DM->CANable0_sendFrame(Motor_DM_Data->DM_id, Data_DM_CAN, 8);
}

/// @brief 设置零点
/// @param dev
/// @param channel
/// @param Motor_DM_Data
void Motor_DM_Control::Actuator_DM_Zore(Motor_DM_CAN_Send_Struct *Motor_DM_Data)
{

    Data_DM_CAN[0] = 0xFF;
    Data_DM_CAN[1] = 0xFF;
    Data_DM_CAN[2] = 0xFF;
    Data_DM_CAN[3] = 0xFF;
    Data_DM_CAN[4] = 0xFF;
    Data_DM_CAN[5] = 0xFF;
    Data_DM_CAN[6] = 0xFF;
    Data_DM_CAN[7] = 0xFE;

    canManager_DM->CANable0_sendFrame(Motor_DM_Data->DM_id, Data_DM_CAN, 8);
}

/// @brief 电机控制
/// @param dev 模块设备号
/// @param channel can1或者can2
/// @param Motor_DM_Data 电机数据
// @brief 电机控制
// @param Motor_DM_Data 电机数据
// @param CAN设备管理器
void Motor_DM_Control::CANable_DM_Send_Control(Motor_DM_CAN_Send_Struct *Motor_DM_Data)
{
    // 限制范围
    if (Motor_DM_Data->DM_kp > KP_MAX)
        Motor_DM_Data->DM_kp = KP_MAX;
    else if (Motor_DM_Data->DM_kp < KP_MIN)
        Motor_DM_Data->DM_kp = KP_MIN;

    if (Motor_DM_Data->DM_kd > KD_MAX)
        Motor_DM_Data->DM_kd = KD_MAX;
    else if (Motor_DM_Data->DM_kd < KD_MIN)
        Motor_DM_Data->DM_kd = KD_MIN;

    if (Motor_DM_Data->DM_position > P_MAX)
        Motor_DM_Data->DM_position = P_MAX;
    else if (Motor_DM_Data->DM_position < P_MIN)
        Motor_DM_Data->DM_position = P_MIN;

    if (Motor_DM_Data->DM_speed > V_MAX)
        Motor_DM_Data->DM_speed = V_MAX;
    else if (Motor_DM_Data->DM_speed < V_MIN)
        Motor_DM_Data->DM_speed = V_MIN;

    if (Motor_DM_Data->DM_torque > T_MAX)
        Motor_DM_Data->DM_torque = T_MAX;
    else if (Motor_DM_Data->DM_torque < T_MIN)
        Motor_DM_Data->DM_torque = T_MIN;

    uint8_t Data_DM_CAN_Control[8];

    Data_DM_CAN_Control[0] = float_to_uint(Motor_DM_Data->DM_position, P_MIN, P_MAX, 16) >> 8;
    Data_DM_CAN_Control[1] = float_to_uint(Motor_DM_Data->DM_position, P_MIN, P_MAX, 16) & 0xFF;
    Data_DM_CAN_Control[2] = float_to_uint(Motor_DM_Data->DM_speed, V_MIN, V_MAX, 12) >> 4;
    Data_DM_CAN_Control[3] = ((float_to_uint(Motor_DM_Data->DM_speed, V_MIN, V_MAX, 12) & 0xF) << 4) |
                             (float_to_uint(Motor_DM_Data->DM_kp, KP_MIN, KP_MAX, 12) >> 8);
    Data_DM_CAN_Control[4] = float_to_uint(Motor_DM_Data->DM_kp, KP_MIN, KP_MAX, 12) & 0xFF;
    Data_DM_CAN_Control[5] = float_to_uint(Motor_DM_Data->DM_kd, KD_MIN, KD_MAX, 12) >> 4;
    Data_DM_CAN_Control[6] = ((float_to_uint(Motor_DM_Data->DM_kd, KD_MIN, KD_MAX, 12) & 0xF) << 4) |
                             (float_to_uint(Motor_DM_Data->DM_torque, T_MIN, T_MAX, 12) >> 8);
    Data_DM_CAN_Control[7] = float_to_uint(Motor_DM_Data->DM_torque, T_MIN, T_MAX, 12) & 0xFF;

    canManager_DM->CANable0_sendFrame(Motor_DM_Data->DM_id, Data_DM_CAN_Control, 8);
}

/// @brief 电机阻尼模式
/// @param dev
/// @param channel
/// @param Motor_DM_Data
void Motor_DM_Control::Actuator_DM_Passive_SET(Motor_DM_CAN_Send_Struct *Motor_DM_Data)
{
    Motor_DM_Data->DM_speed = 0;
    Motor_DM_Data->DM_kp = 0;
    Motor_DM_Data->DM_kd = 2.0;
    Motor_DM_Data->DM_torque = 0;

    canManager_DM->CANable0_sendFrame(Motor_DM_Data->DM_id, Data_DM_CAN, 8);
}

void Motor_DM_Control::ENABLE_ALL_DM_Actuator(int delay_us)
{
    auto t = std::chrono::high_resolution_clock::now(); // 这一句耗时50us
    Actuator_DM_Enable(&CANable0_DM_CAN_Bus.ID_1_motor_DM_send);
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);
}

void Motor_DM_Control::DISABLE_ALL_Actuator_DM(int delay_us)
{
    auto t = std::chrono::high_resolution_clock::now(); // 这一句耗时50us
    Actuator_DM_Disable(&CANable0_DM_CAN_Bus.ID_1_motor_DM_send);
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);
}

void Motor_DM_Control::ZERO_ALL_Actuator_DM(int delay_us)
{
    auto t = std::chrono::high_resolution_clock::now(); // 这一句耗时50us
    Actuator_DM_Zore(&CANable0_DM_CAN_Bus.ID_1_motor_DM_send);
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);
}

void Motor_DM_Control::PASSIVE_ALL_Actuator_DM(int delay_us)
{
    auto t = std::chrono::high_resolution_clock::now(); // 这一句耗时50us
    Actuator_DM_Passive_SET(&CANable0_DM_CAN_Bus.ID_1_motor_DM_send);
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);
}

/// @brief can控制发送，12个电机的数据
// 目前能达到1000hz的控制频率--------3000hz的总线发送频率---------同一路can的发送间隔在300us
void Motor_DM_Control::CANable_TX_ALL_Actuator_DM(int delay_us)
{
    auto t = std::chrono::high_resolution_clock::now(); // 这一句耗时50us
    CANable_DM_Send_Control(&CANable0_DM_CAN_Bus.ID_1_motor_DM_send);
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);
}

void Motor_DM_Control::Handle_DM_Information(struct can_frame &frame, Motor_DM_CAN_Recieve_Struct &motorData)
{
    // 解码
    motorData.DM_ERR = frame.data[0] >> 4 & 0X0F;
    motorData.DM_current_position = (frame.data[1] << 8) | frame.data[2];       // 电机位置数据
    motorData.DM_current_speed = (frame.data[3] << 4) | (frame.data[4] >> 4);   // 电机速度数据
    motorData.DM_current_torque = ((frame.data[4] & 0xF) << 8) | frame.data[5]; // 电机扭矩数据
    motorData.DM_current_temp_MOS = frame.data[6];
    motorData.DM_current_temp_Rotor = frame.data[7];
    // 转换
    motorData.DM_current_position_f = uint_to_float(motorData.DM_current_position, (P_MIN), (P_MAX), 16);
    motorData.DM_current_speed_f = uint_to_float(motorData.DM_current_speed, (V_MIN), (V_MAX), 12);
    motorData.DM_current_torque_f = uint_to_float(motorData.DM_current_torque, (T_MIN), (T_MAX), 12);
}
