#include "dvc_ops.h"


SECTION_RAM_D1 static uint8_t OPS_Circular_Buffer[128] __attribute__((aligned(32)));
// 初始化函数
void Class_OPS::init()
{
    memset(&OPS_Data, 0, sizeof(Union_OPS_Raw));
    _last_update_tick = 0;
    parse_state = 0;
    _last_read_pos = 0;

}

// 检查连接状态
bool Class_OPS::isConnected()
{
    // 超过 500ms 未更新则认为断连
    return (HAL_GetTick() - _last_update_tick) < 500;
}

// // 接收一整段 Buffer 并逐字节送入状态机
// void Class_OPS::parseBuffer(uint8_t* buffer, uint16_t length) {
//     if (buffer == nullptr || length == 0) return;
//
//     for (uint16_t i = 0; i < length; i++) {
//         processByte(buffer[i]);
//     }
// }

void Class_OPS::processByte(uint8_t byte)
{
    switch (parse_state)
    {
    case 0: // 等待帧头第一字节 0x0D
        if (byte == 0x0D) parse_state = 1;
        break;
    case 1: // 等待帧头第二字节 0x0A
        if (byte == 0x0A)
        {
            parse_state = 2;
            payload_idx = 0;
        }
        else if (byte != 0x0D)
        {
            parse_state = 0;
        }
        break;
    case 2: // 接收 24 字节数据
        temp_buffer[payload_idx++] = byte;
        if (payload_idx >= 24) parse_state = 3;
        break;
    case 3: // 等待帧尾第一字节 0x0A
        if (byte == 0x0A) parse_state = 4;
        else parse_state = 0;
        break;
    case 4: // 等待帧尾第二字节 0x0D
        if (byte == 0x0D)
        {
            memcpy(OPS_Data.bytes, temp_buffer, 24);
              _last_update_tick = HAL_GetTick();
        }
        parse_state = 0; // 重置状态机
        break;
    }
}

// 接收中断处理
void Class_OPS::onRxCircularEvent(uint8_t* buffer, uint16_t size, uint16_t buffer_capacity)
{
    if (buffer == nullptr || buffer_capacity == 0) return;

    // size 是 DMA 当前接收到的总字节数指针
    uint16_t curr_pos = size;

    if (curr_pos > _last_read_pos)
    {
        // 没有发生绕回，处理新的一段数据
        for (uint16_t i = _last_read_pos; i < curr_pos; i++)
        {
            processByte(buffer[i]);
        }
    }
    else if (curr_pos < _last_read_pos)
    {
        // 发生了绕回：先处理末尾的数据，再处理开头的数据
        for (uint16_t i = _last_read_pos; i < buffer_capacity; i++)
        {
            processByte(buffer[i]);
        }
        for (uint16_t i = 0; i < curr_pos; i++)
        {
            processByte(buffer[i]);
        }
    }
    // 更新读取指针
    _last_read_pos = curr_pos;
}
