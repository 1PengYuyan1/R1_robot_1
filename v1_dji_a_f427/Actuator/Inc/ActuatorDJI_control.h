#ifndef ACTUATORDJI_CONTROL_H
#define ACTUATORDJI_CONTROL_H

#include "CANDeviceManager.h"
#include "MotorStructs.h"
#include "DataType.h"

class CANDeviceManager;

class Motor_DJI_Control
{
private:
    CANDeviceManager *canManager_DJI;

public:
    Motor_DJI_Control(CANDeviceManager *canManager = nullptr);
    ~Motor_DJI_Control();

    // 电机初始化
    void DJI_CAN_Bus_inti_set(CANable_DJI_CAN_Bus_Struct *Leg_Data);
    void DJI_CAN_Bus_Init();
    void Handle_DJI_Information(struct can_frame &frame, Motor_DJI_CAN_Recieve_Struct &motorData);
    void DJI_CANable_send(Motor_DJI_CAN_Send_Struct *Motor_DJI_Data, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
    void DJI_CAN_ALl_send(int delay_us);
};

#endif
