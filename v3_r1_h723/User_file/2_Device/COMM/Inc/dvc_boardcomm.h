#ifndef DVC_BOARDCOMM_H
#define DVC_BOARDCOMM_H

#include "usart.h"
#include <string.h>

// 【A板发送包】只传控制机械臂所需的 3 个状态，总共 5 字节
#pragma pack(push, 1)
struct Struct_Remote_Sync_Packet {
    uint8_t  header;        // 帧头 0xAA
    // uint8_t  dr16_status;   // 遥控器在线状态 (0:断开, 1:在线)
    // uint8_t  left_switch;   // 左拨杆状态
    // uint8_t  right_switch;  // 右拨杆状态
    uint8_t  data;
    uint8_t  tail;          // 帧尾 0x55
};
#pragma pack(pop)

class Class_BoardComm {
public:
    void Init(UART_HandleTypeDef *huart);
    void Send_Data(uint8_t data);

    // 留一个空的回调接口，防止串口被垃圾数据塞满溢出
    void UART_RxCpltCallback();
    Struct_Remote_Sync_Packet TX_Data;
private:
    UART_HandleTypeDef *UART_Handler;
    uint8_t Rx_Byte; // 接收垃圾桶
};

#endif // DVC_BOARDCOMM_H