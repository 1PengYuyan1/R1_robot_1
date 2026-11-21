#ifndef DVC_MINIPC_H
#define DVC_MINIPC_H

#include "main.h"
#include "drv_minipc.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// 发送包结构体 (MCU 发给 ROS 的实际状态/里程计) -> 总长 16 字节
#pragma pack(push, 1)
typedef struct {
    uint8_t  header[2]; // 包头: 0x5A, 0xA5
    float    real_vx;   // 实际X轴速度
    float    real_vy;   // 实际Y轴速度
    float    real_vw;   // 实际Z轴旋转速度
    uint16_t checksum;  // CRC16校验码 (校验前面 14 个字节)
} MCU_To_ROS_Packet_t;
#pragma pack(pop)

class Class_Chassis_MiniPC {
public:
    void Init();

    // 解析与计算 (建议放入定时器循环调用)
    void Parse_Data();
    void TIM_1ms_Calculate_PeriodElapsedCallback(); // 效仿 F710 的软看门狗机制

    // 获取解析后的速度期望值
    inline float Get_Vx() { return target_vx; }
    inline float Get_Vy() { return target_vy; }
    inline float Get_Vw() { return target_vw; }

    // 判断 ROS 端是否连接/在线
    inline bool Is_Connected() { return Alive_Count > 0; }

    void Send_Data(float current_vx, float current_vy, float current_vw);

private:
    MCU_To_ROS_Packet_t tx_packet;

    float target_vx;
    float target_vy;
    float target_vw;

    // --- 为适配你的上位机，更新状态机定义 ---
    enum Enum_State {
        STATE_WAIT_HEAD0,
        STATE_WAIT_HEAD1,
        STATE_WAIT_LEN,
        STATE_WAIT_FUNC,
        STATE_WAIT_DATA,
        STATE_WAIT_CRC_H,
        STATE_WAIT_CRC_L
    } Cur_State;

    uint8_t Rx_Len;
    uint8_t Rx_Func;
    uint8_t Rx_Data_Buffer[32]; // 稍微放大一点防止溢出
    uint8_t Rx_Data_Index;
    uint16_t Rx_CRC_Rec;

    int16_t Alive_Count;
};

#endif // DVC_INIPC_H