#include "ActuatorODrive.h"

ODrive_Control::ODrive_Control(CANDeviceManager *canManager) : canManager_ODrive(canManager)
{
    // 先初始化 CAN 管理器
    if (!canManager_ODrive)
    {
        std::cout << "警告: CANDeviceManager 指针为空，请在后续设置" << std::endl;
    }
    ODrive_CAN_Bus_Init();

    // 只有在 CAN 管理器有效时才初始化电机
    if (canManager_ODrive)
    {
        std::cout << "ODrive电机初始化完成" << std::endl;
    }
}

ODrive_Control::~ODrive_Control()
{
    if (canManager_ODrive)
    {
        DISABLE_ALL_ODrive_Actuator(1000);
    }
}

void ODrive_Control::ODrive_CAN_Bus_inti_set(CANable_ODrive_CAN_Bus_Struct *ODrive_Bus)
{
    ODrive_Bus->ID_1_ODrive_Send.node_id = 0x010;
    ODrive_Bus->ID_1_ODrive_Send.extended_id = false;
}

void ODrive_Control::ODrive_CAN_Bus_Init()
{
    ODrive_CAN_Bus_inti_set(&CANable0_ODrive_CAN_Bus);
    ODrive_Set_Axis_State(&CANable0_ODrive_CAN_Bus, AXIS_STATE_CLOSED_LOOP_CONTROL);
}

// 状态控制函数
void ODrive_Control::ODrive_Set_Axis_State(CANable_ODrive_CAN_Bus_Struct *ODrive_Bus, ODriveAxisState_t state)
{
    if (!canManager_ODrive)
        return;

    uint8_t data[8] = {0};
    data[0] = state & 0xFF;
    data[1] = (state >> 8) & 0xFF;
    data[2] = 0;
    data[3] = 0;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;

    uint32_t can_id = ((ODrive_Bus->ID_1_ODrive_Send.node_id << 5) | ODRIVE_CMD_SET_AXIS_STATE) & 0xFFF;
    canManager_ODrive->CANable0_sendFrame(can_id, data, 8, ODrive_Bus->ID_1_ODrive_Send.extended_id);
}

// 运动控制函数
void ODrive_Control::ODrive_Set_Velocity(CANable_ODrive_CAN_Bus_Struct *ODrive_Bus, float velocity, float torque_ff)
{
    if (!canManager_ODrive)
        return;

    // 参数限制
    if (velocity > V_MAX)
        velocity = V_MAX;
    else if (velocity < V_MIN)
        velocity = V_MIN;

    if (torque_ff > T_MAX)
        torque_ff = T_MAX;
    else if (torque_ff < T_MIN)
        torque_ff = T_MIN;

    uint8_t data[8] = {0};

    memcpy(&data[0], &velocity, 4);
    memcpy(&data[4], &torque_ff, 4);

    uint32_t can_id = ((ODrive_Bus->ID_1_ODrive_Send.node_id << 5) | ODRIVE_CMD_SET_INPUT_VEL) & 0xFFF;
    canManager_ODrive->CANable0_sendFrame(can_id, data, 8, ODrive_Bus->ID_1_ODrive_Send.extended_id);
}

void ODrive_Control::ODrive_Set_Position(CANable_ODrive_CAN_Bus_Struct *ODrive_Bus, float position, float velocity_ff)
{
    if (!canManager_ODrive)
        return;

    // 参数限制
    if (position > V_MAX)
        position = V_MAX;
    else if (position < V_MIN)
        position = V_MIN;

    if (velocity_ff > T_MAX)
        velocity_ff = T_MAX;
    else if (velocity_ff < T_MIN)
        velocity_ff = T_MIN;

    uint8_t data[8] = {0};

    memcpy(&data[0], &position, 4);
    memcpy(&data[4], &velocity_ff, 4);

    uint32_t can_id = ((ODrive_Bus->ID_1_ODrive_Send.node_id << 5) | ODRIVE_CMD_SET_INPUT_POS) & 0xFFF;
    canManager_ODrive->CANable0_sendFrame(can_id, data, 8, ODrive_Bus->ID_1_ODrive_Send.extended_id);
}

// 数据处理函数
void ODrive_Control::Handle_ODrive_Information(struct can_frame &frame, ODrive_CAN_Recieve_Struct &motorData)
{
    // 解码CAN帧数据
    uint32_t can_id = frame.can_id & 0xFFF;
    uint8_t node_id = (can_id >> 5) & 0x3F;
    uint8_t cmd_id = can_id & 0x1F;

    motorData.node_id = node_id;
    motorData.command_id = static_cast<ODriveCommandID_t>(cmd_id);

    switch (cmd_id)
    {
    case ODRIVE_CMD_GET_ENCODER_ESTIMATES:
        memcpy(&motorData.position, &frame.data[0], 4);
        memcpy(&motorData.velocity, &frame.data[4], 4);
        // 更新实时数据
        UpdateMotorData(motorData);

        // 打印实时速度（调试用）
        std::cout << "ODrive节点" << std::hex << (int)node_id << std::dec
                  << " - 位置: " << motorData.position
                  << " rad, 速度: " << motorData.velocity << " rad/s" << std::endl;
        break;
    case ODRIVE_CMD_GET_BUS_VOLTAGE:
        memcpy(&motorData.bus_voltage, &frame.data[0], 4);
        break;
    default:
        break;
    }
}

// 请求速度信息
void ODrive_Control::ODrive_Request_Velocity(CANable_ODrive_CAN_Bus_Struct *ODrive_Bus)
{
    if (!canManager_ODrive)
        return;

    uint8_t data[8] = {00, 00, 00, 00, 00, 00, 00, 00};
    uint32_t can_id = (ODrive_Bus->ID_1_ODrive_Send.node_id << 5) | ODRIVE_CMD_GET_ENCODER_ESTIMATES;
    can_id |= CAN_RTR_FLAG;
    canManager_ODrive->CANable0_sendFrame(can_id, data, 8, ODrive_Bus->ID_1_ODrive_Send.extended_id);
}

// 更新电机数据
void ODrive_Control::UpdateMotorData(const ODrive_CAN_Recieve_Struct &data)
{

    if (data.command_id == ODRIVE_CMD_GET_ENCODER_ESTIMATES)
    {
        current_position_ = data.position;
        current_velocity_ = data.velocity;
    }
}

// 批量操作函数
void ODrive_Control::ENABLE_ALL_ODrive_Actuator(int delay_us)
{
    auto t = std::chrono::high_resolution_clock::now(); // 这一句耗时50us
    ODrive_Set_Axis_State(&CANable0_ODrive_CAN_Bus, AXIS_STATE_CLOSED_LOOP_CONTROL);
    t += std::chrono::microseconds(delay_us);
}

void ODrive_Control::DISABLE_ALL_ODrive_Actuator(int delay_us)
{
    auto t = std::chrono::high_resolution_clock::now(); // 这一句耗时50us
    ODrive_Set_Axis_State(&CANable0_ODrive_CAN_Bus, AXIS_STATE_IDLE);
    std::this_thread::sleep_until(t);
}

void ODrive_Control::ODrive_CAN_TX_All_Actuator(int delay_us)
{
    auto t = std::chrono::high_resolution_clock::now();
    ODrive_Set_Velocity(&CANable0_ODrive_CAN_Bus, 10.0f, 0.0f);
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);
}