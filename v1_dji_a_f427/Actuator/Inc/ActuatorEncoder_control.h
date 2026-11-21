#ifndef ACTUATORENCODER_CONTROL_H
#define ACTUATORENCODER_CONTROL_H

#include "CANDeviceManager.h"
#include "MotorStructs.h"
#include "DataType.h"

class CANDeviceManager;

class Encoder_Control
{
private:
    uint8_t Data_Encoder_CAN[8]; // CAN数据缓冲区
    CANDeviceManager *canManager_Encoder;

public:
    Encoder_Control(CANDeviceManager* canManager = nullptr);
    ~Encoder_Control();


    // 编码器初始化
    void CANable_Encoder_CAN_Bus_inti_set(CANable_Encoder_CAN_Bus_Struct *Encoder_Data);
    void CANable_Encoder_CAN_Bus_Init();

    // 编码器基本操作
    void Encoder_Read_Position(Encoder_CAN_Send_Struct *Encoder_Data);
    void Encoder_Set_Zero(Encoder_CAN_Send_Struct *Encoder_Data);
    void Encoder_Set_ID(Encoder_CAN_Send_Struct *Encoder_Data, uint8_t new_id);
    void Encoder_Set_Baudrate(Encoder_CAN_Send_Struct *Encoder_Data, EncoderBaudrate_t baudrate);
    void Encoder_Restore_Factory(Encoder_CAN_Send_Struct *Encoder_Data);
    void Encoder_Read_Status(Encoder_CAN_Send_Struct *Encoder_Data);

    // 批量操作函数
    void READ_ALL_Encoder_Position(int delay_us);
    void ZERO_ALL_Encoder(int delay_us);
    void RESTORE_FACTORY_ALL_Encoder(int delay_us);

    // 处理接收到的编码器信息
    void Handle_Encoder_Information(struct can_frame &frame, Encoder_CAN_Recieve_Struct &recieve_data);
};

#endif