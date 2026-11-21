#ifndef DRV_MINIPC_H
#define DRV_MINIPC_H

#include "main.h"
#include <stdint.h>
#include "usbd_cdc_if.h"

// 缓冲区大小，建议为2的幂次方
#define MINIPC_FIFO_SIZE 1024

class Class_Drv_MiniPC {
public:
    void Init();

    // 提供给 USB 中断调用的写入接口
    void Write_Data_To_FIFO(uint8_t* pData, uint16_t Length);

    // 提供给解算器调用的读取接口
    uint8_t Read_Byte_From_FIFO(uint8_t* pByte);

    //提供给上层调用的 USB 发送接口
    void Transmit_Data(uint8_t* pData, uint16_t Length);
private:
    uint8_t Rx_FIFO[MINIPC_FIFO_SIZE];
    volatile uint16_t Head; // 写入位置
    volatile uint16_t Tail; // 读取位置
};

extern Class_Drv_MiniPC Drv_MiniPC;

#endif