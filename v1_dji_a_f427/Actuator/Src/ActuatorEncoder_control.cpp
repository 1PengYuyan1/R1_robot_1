#include "ActuatorEncoder_control.h"

Encoder_Control::Encoder_Control(CANDeviceManager *canManager) : canManager_Encoder(canManager)
{
    if (canManager_Encoder)
    {
        CANable_Encoder_CAN_Bus_Init();
        std::cout << "编码器初始化完成" << std::endl;
    }
}

Encoder_Control::~Encoder_Control()
{
}

// 编码器CAN总线初始化设置
void Encoder_Control::CANable_Encoder_CAN_Bus_inti_set(CANable_Encoder_CAN_Bus_Struct *CAN_Encoder_Bus)
{
    CAN_Encoder_Bus->ID_1_Encoder_send.Encoder_id = 0x02;
    CAN_Encoder_Bus->ID_1_Encoder_send.Encoder_position = 0;
    CAN_Encoder_Bus->ID_1_Encoder_send.Encoder_multi_turn = 0;
    CAN_Encoder_Bus->ID_1_Encoder_send.Encoder_single_turn = 0;
    CAN_Encoder_Bus->ID_1_Encoder_send.Encoder_baudrate = ENCODER_BAUDRATE_1000000;
    CAN_Encoder_Bus->ID_1_Encoder_send.Encoder_resolution = 16; // 默认16位分辨率
}

void Encoder_Control::CANable_Encoder_CAN_Bus_Init()
{
    CANable_Encoder_CAN_Bus_inti_set(&CANable0_Encoder_CAN_Bus);
}

// 读取编码器位置
void Encoder_Control::Encoder_Read_Position(Encoder_CAN_Send_Struct *Encoder_Data)
{
    Data_Encoder_CAN[0] = 0x04;                      // 数据长度: 4字节
    Data_Encoder_CAN[1] = Encoder_Data->Encoder_id;  // 编码器地址
    Data_Encoder_CAN[2] = ENCODER_CMD_READ_POSITION; // 功能码: 读取位置
    Data_Encoder_CAN[3] = 0x00;                      // 数据域
    Data_Encoder_CAN[4] = 0x00;
    Data_Encoder_CAN[5] = 0x00;
    Data_Encoder_CAN[6] = 0x00;
    Data_Encoder_CAN[7] = 0x00;

    canManager_Encoder->CANable0_sendFrame(Encoder_Data->Encoder_id, Data_Encoder_CAN, 8);
}

// 设置编码器零点
void Encoder_Control::Encoder_Set_Zero(Encoder_CAN_Send_Struct *Encoder_Data)
{
    Data_Encoder_CAN[0] = 0x04; // 数据长度: 4字节
    Data_Encoder_CAN[1] = Encoder_Data->Encoder_id;
    Data_Encoder_CAN[2] = ENCODER_CMD_SET_ZERO; // 功能码: 设置零点
    Data_Encoder_CAN[3] = 0x00;
    canManager_Encoder->CANable0_sendFrame(Encoder_Data->Encoder_id, Data_Encoder_CAN, 4);
}

// 设置编码器ID地址
void Encoder_Control::Encoder_Set_ID(Encoder_CAN_Send_Struct *Encoder_Data, uint8_t new_id)
{
    Data_Encoder_CAN[0] = 0x05; // 数据长度: 5字节
    Data_Encoder_CAN[1] = Encoder_Data->Encoder_id;
    Data_Encoder_CAN[2] = ENCODER_CMD_SET_ID; // 功能码: 设置ID
    Data_Encoder_CAN[3] = new_id;             // 新的ID地址
    Data_Encoder_CAN[4] = 0x00;
    canManager_Encoder->CANable0_sendFrame(Encoder_Data->Encoder_id, Data_Encoder_CAN, 5);
}

// 设置编码器波特率
void Encoder_Control::Encoder_Set_Baudrate(Encoder_CAN_Send_Struct *Encoder_Data, EncoderBaudrate_t baudrate)
{
    Data_Encoder_CAN[0] = 0x05; // 数据长度: 5字节
    Data_Encoder_CAN[1] = Encoder_Data->Encoder_id;
    Data_Encoder_CAN[2] = ENCODER_CMD_SET_BAUDRATE; // 功能码: 设置波特率
    Data_Encoder_CAN[3] = (uint8_t)baudrate;        // 波特率值
    Data_Encoder_CAN[4] = 0x00;
    canManager_Encoder->CANable0_sendFrame(Encoder_Data->Encoder_id, Data_Encoder_CAN, 5);
}

// 恢复出厂设置
void Encoder_Control::Encoder_Restore_Factory(Encoder_CAN_Send_Struct *Encoder_Data)
{
    Data_Encoder_CAN[0] = 0x04; // 数据长度: 4字节
    Data_Encoder_CAN[1] = Encoder_Data->Encoder_id;
    Data_Encoder_CAN[2] = ENCODER_CMD_RESTORE_FACTORY; // 功能码: 恢复出厂
    Data_Encoder_CAN[3] = 0x00;
    canManager_Encoder->CANable0_sendFrame(Encoder_Data->Encoder_id, Data_Encoder_CAN, 4);
}

// 读取编码器状态信息
void Encoder_Control::Encoder_Read_Status(Encoder_CAN_Send_Struct *Encoder_Data)
{
    Data_Encoder_CAN[0] = 0x04; // 数据长度: 4字节
    Data_Encoder_CAN[1] = Encoder_Data->Encoder_id;
    Data_Encoder_CAN[2] = ENCODER_CMD_READ_STATUS; // 功能码: 读取状态
    Data_Encoder_CAN[3] = 0x00;
    canManager_Encoder->CANable0_sendFrame(Encoder_Data->Encoder_id, Data_Encoder_CAN, 4);
}

// 处理接收到的编码器信息
void Encoder_Control::Handle_Encoder_Information(struct can_frame &frame, Encoder_CAN_Recieve_Struct &recieve_data)
{
    if (frame.len >= 7)
    {
        recieve_data.Encoder_data_length = frame.data[0];
        recieve_data.Encoder_id = frame.data[1];
        recieve_data.Encoder_function_code = frame.data[2];

        if (recieve_data.Encoder_function_code == ENCODER_CMD_READ_POSITION && recieve_data.Encoder_data_length >= 7)
        {
            // 解析32位位置数据 (低字节在前)
            uint32_t position = (frame.data[6] << 24) | (frame.data[5] << 16) | (frame.data[4] << 8) | frame.data[3];

            float angle_deg = ConvertToDegrees(position);
            float angle_rad = ConvertToRadians(position);

            std::cout << "原始位置: " << position << " pulses, ";
            std::cout << "角度: " << angle_deg << "°, ";
            std::cout << "弧度: " << angle_rad << " rad";

            recieve_data.Encoder_current_position_f = Convert_Position_To_Float(position);
            std::cout << " | 转换后: " << recieve_data.Encoder_current_position_f << std::endl;

            // 可选：存储角度和弧度值
            recieve_data.Encoder_current_angle_deg = angle_deg;
            recieve_data.Encoder_current_angle_rad = angle_rad;
        }

        if (recieve_data.Encoder_function_code == ENCODER_CMD_READ_STATUS && recieve_data.Encoder_data_length >= 5)
        {
            recieve_data.Encoder_status = frame.data[3];
            recieve_data.Encoder_temperature = frame.data[4];
        }
    }
}

// 批量读取所有编码器位置
void Encoder_Control::READ_ALL_Encoder_Position(int delay_us)
{
    auto t = std::chrono::high_resolution_clock::now(); // 这一句耗时50us

    Encoder_Read_Position(&CANable0_Encoder_CAN_Bus.ID_1_Encoder_send);

    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);
}

// 批量设置所有编码器零点
void Encoder_Control::ZERO_ALL_Encoder(int delay_us)
{
    auto t = std::chrono::high_resolution_clock::now(); // 这一句耗时50us

    Encoder_Set_Zero(&CANable0_Encoder_CAN_Bus.ID_1_Encoder_send);

    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);
}

// 批量恢复所有编码器出厂设置
void Encoder_Control::RESTORE_FACTORY_ALL_Encoder(int delay_us)
{
    auto t = std::chrono::high_resolution_clock::now(); // 这一句耗时50us

    Encoder_Restore_Factory(&CANable0_Encoder_CAN_Bus.ID_1_Encoder_send);

    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);
}
