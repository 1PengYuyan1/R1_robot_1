#include "drv_minipc.h"
#include <string.h>

Class_Drv_MiniPC Drv_MiniPC;

// 外部 C 接口：对接 USB CDC 的接收回调
extern "C" {
    void MiniPC_USB_Receive_Wrapper(uint8_t *Buf, uint32_t Len) {
        Drv_MiniPC.Write_Data_To_FIFO(Buf, Len);
    }
}

void Class_Drv_MiniPC::Init() {
    Head = 0;
    Tail = 0;
    memset(Rx_FIFO, 0, MINIPC_FIFO_SIZE);
}

// 写入数据（在中断中调用）
void Class_Drv_MiniPC::Write_Data_To_FIFO(uint8_t* pData, uint16_t Length) {
    for (uint16_t i = 0; i < Length; i++) {
        uint16_t next_head = (Head + 1) % MINIPC_FIFO_SIZE;

        // 如果缓冲区没满，就写入
        if (next_head != Tail) {
            Rx_FIFO[Head] = pData[i];
            Head = next_head;
        } else {
            // 缓冲区溢出，丢弃数据或记录错误
            break;
        }
    }
}

// 读取一个字节（返回 1 表示读取成功，0 表示缓冲区空）
uint8_t Class_Drv_MiniPC::Read_Byte_From_FIFO(uint8_t* pByte) {
    if (Head == Tail) {
        return 0; // 空
    }

    *pByte = Rx_FIFO[Tail];
    Tail = (Tail + 1) % MINIPC_FIFO_SIZE;
    return 1;
}

void Class_Drv_MiniPC::Transmit_Data(uint8_t* pData, uint16_t Length) {
    // 调用 ST 官方生成的 HS 接口发送数据
    CDC_Transmit_HS(pData, Length);
}