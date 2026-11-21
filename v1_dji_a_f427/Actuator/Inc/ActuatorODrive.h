#ifndef ACTUATORODRIVE_H
#define ACTUATORODRIVE_H

#include "CANDeviceManager.h"
#include "MotorStructs.h"
#include "DataType.h"
#include <iostream>
#include <cstring>
#include <cstdint>
#include <thread>
#include <chrono>

class CANDeviceManager;

class ODrive_Control
{
private:
    CANDeviceManager *canManager_ODrive;
    uint8_t Data_ODrive_CAN[8];
    float current_velocity_ = 0.0f;
    float current_position_ = 0.0f;

public:
    ODrive_Control(CANDeviceManager *canManager = nullptr);
    ~ODrive_Control();

    // 初始化函数
    void ODrive_CAN_Bus_inti_set(CANable_ODrive_CAN_Bus_Struct *ODrive_Bus);
    void ODrive_CAN_Bus_Init();

    // 状态控制函数
    void ODrive_Set_Axis_State(CANable_ODrive_CAN_Bus_Struct *ODrive_Bus, ODriveAxisState_t state);
    void ODrive_Set_Control_Mode(CANable_ODrive_CAN_Bus_Struct *ODrive_Bus, ODriveControlMode_t control_mode, ODriveInputMode_t input_mode);

    // 运动控制函数
    void ODrive_Set_Velocity(CANable_ODrive_CAN_Bus_Struct *ODrive_Bus, float velocity, float torque_ff = 0.0f);
    void ODrive_Set_Position(CANable_ODrive_CAN_Bus_Struct *ODrive_Bus, float position, float velocity_ff = 0.0f);
    void ODrive_Set_Torque(CANable_ODrive_CAN_Bus_Struct *ODrive_Bus, float torque);

    // 数据获取函数
    void ODrive_Request_Velocity(CANable_ODrive_CAN_Bus_Struct *ODrive_Bus);
    float GetCurrentVelocity() const { return current_velocity_; }
    float GetCurrentPosition() const { return current_position_; }

    // 添加数据处理回调
    void UpdateMotorData(const ODrive_CAN_Recieve_Struct &data);
    
    // 数据处理函数
    void Handle_ODrive_Information(struct can_frame &frame, ODrive_CAN_Recieve_Struct &motorData);

    // 批量操作函数
    void ENABLE_ALL_ODrive_Actuator(int delay_us);
    void DISABLE_ALL_ODrive_Actuator(int delay_us);
    void REBOOT_ALL_ODrive_Actuator(int delay_us);

    // 发送函数
    void ODrive_CAN_TX_All_Actuator(int delay_us);
};

#endif