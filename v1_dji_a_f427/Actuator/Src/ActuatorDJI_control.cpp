#include "ActuatorDJI_control.h"

Motor_DJI_Control::Motor_DJI_Control(CANDeviceManager *canManager) : canManager_DJI(canManager)
{
    if (canManager_DJI)
    {
        DJI_CAN_Bus_Init();
        std::cout << "DJI电机初始化完成" << std::endl;
    }
}
Motor_DJI_Control::~Motor_DJI_Control()
{
}

void Motor_DJI_Control::DJI_CAN_Bus_inti_set(CANable_DJI_CAN_Bus_Struct *CAN_DJI_Bus)
{
    CAN_DJI_Bus->ID_1_motor_DJI_send.DJI_id = 0X200;
}

void Motor_DJI_Control::DJI_CAN_Bus_Init()
{
    DJI_CAN_Bus_inti_set(&CANable0_DJI_CAN_Bus);
}

void Motor_DJI_Control::DJI_CANable_send(Motor_DJI_CAN_Send_Struct *Motor_DJI_Data, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
    uint8_t Data_DJI_CAN_contiol[8];
    Data_DJI_CAN_contiol[0] = iq1 >> 8;
    Data_DJI_CAN_contiol[1] = iq1;
    Data_DJI_CAN_contiol[2] = iq2 >> 8;
    Data_DJI_CAN_contiol[3] = iq2;
    Data_DJI_CAN_contiol[4] = iq3 >> 8;
    Data_DJI_CAN_contiol[5] = iq3;
    Data_DJI_CAN_contiol[6] = iq4 >> 8;
    Data_DJI_CAN_contiol[7] = iq4;

    canManager_DJI->CANable0_sendFrame(Motor_DJI_Data->DJI_id, Data_DJI_CAN_contiol, 8);
}

void Motor_DJI_Control::Handle_DJI_Information(struct can_frame &frame, Motor_DJI_CAN_Recieve_Struct &motorData)
{
    motorData.DJI_last_position = motorData.DJI_current_position;
    // 解码
    motorData.DJI_current_position = (frame.data[0] << 8) | (frame.data[1]); // 电机位置数据
    motorData.DJI_current_speed = (frame.data[2] << 8) | (frame.data[3]);    // 电机速度数据
    motorData.DJI_current_I = (frame.data[2] << 8) | (frame.data[5]);        // 电机速度数据
    motorData.DJI_current_Temperature = (frame.data[6]);                     // 电机温度数据
    // 转换
    motorData.DJI_current_position_f = uint_to_float(motorData.DJI_current_position, (P_MIN), (P_MAX), 16);
    motorData.DJI_current_speed_f = uint_to_float(motorData.DJI_current_speed, (V_MIN), (V_MAX), 12);

    std::cout << std::endl
    << "DJI.current_position_f=  " << CANable0_DJI_CAN_Bus.ID_1_motor_DJI_recieve.DJI_current_position_f << "  rad/s" << std::endl
    << "DJI.current_speed_f=  " << CANable0_DJI_CAN_Bus.ID_1_motor_DJI_recieve.DJI_current_speed_f << "  rad/s" << std::endl;
}

void Motor_DJI_Control::DJI_CAN_ALl_send(int delay_us)
{
    auto t = std::chrono::high_resolution_clock::now(); // 这一句耗时50us
    DJI_CANable_send(&CANable0_DJI_CAN_Bus.ID_1_motor_DJI_send, 1000, 1000, 0, 0);
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);
}
