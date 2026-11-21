//
// Created by Pzx on 26-3-9.
//

#ifndef DVC_OPS_H
#define DVC_OPS_H

#include "../../Core/Inc/main.h"
#include <cstring> // for memcpy

#if defined(__GNUC__)
    #define SECTION_RAM_D1 __attribute__((section(".ram_d1")))
#else
    #define SECTION_RAM_D1
#endif

struct Struct_OPS_Rx_Data
{
    float Yaw;      // 航向角
    float Pitch;        // 俯仰角
    float Roll;         // 横滚角
    float Pos_X;        // X坐标 (mm)
    float Pos_Y;        // Y坐标 (mm)
    float Omega_Z;      // 角速度 (w_z)
};

union Union_OPS_Raw {
    uint8_t bytes[24];
    Struct_OPS_Rx_Data Rx_Data; // 直接使用定义好的结构体
};

class Class_OPS {
public:

    // 初始化：启动DMA接收
    void init();
    // 解析缓冲数据（接收 drv_uart 传来的数据）
    void parseBuffer(uint8_t* buffer, uint16_t length);
    // 中断处理函数：需要在 HAL_UARTEx_RxEventCallback 中调用
    void onRxCircularEvent(uint8_t* buffer, uint16_t size, uint16_t buffer_capacity);
    // 获取最新数据 (返回副本，线程安全建议加锁，这里简单处理)
    inline Struct_OPS_Rx_Data getData();
    // 检查是否已连接 (根据最近是否有数据更新)
    bool isConnected();
private:
    uint32_t _last_update_tick = 0;
    uint16_t _last_read_pos = 0;
    Union_OPS_Raw OPS_Data;

    uint8_t parse_state = 0;
    uint8_t payload_idx = 0;
    uint8_t temp_buffer[24];

    // --- 状态机解析相关 ---
    void processByte(uint8_t byte);


};

// 获取数据
inline Struct_OPS_Rx_Data Class_OPS::getData() {
    // 直接返回联合体中的结构体部分
    return OPS_Data.Rx_Data;
}



#endif //DVC_OPS_H
