#include "dvc_Chassis_minipc.h"
#include "crc.h" // 确保你的 crc.h 里有 Get_CRC16_Check_Sum 函数

void Class_Chassis_MiniPC::Init() {
    Cur_State = STATE_WAIT_HEAD0;
    Rx_Data_Index = 0;

    target_vx = 0.0f;
    target_vy = 0.0f;
    target_vw = 0.0f;

    Alive_Count = 0;
    memset(Rx_Data_Buffer, 0, sizeof(Rx_Data_Buffer));

    Drv_MiniPC.Init();
}

void Class_Chassis_MiniPC::TIM_1ms_Calculate_PeriodElapsedCallback() {
    if (Alive_Count > 0) {
        Alive_Count--;
    } else {
        target_vx = 0.0f;
        target_vy = 0.0f;
        target_vw = 0.0f;
    }
}

void Class_Chassis_MiniPC::Parse_Data() {
    uint8_t byte;

    while (Drv_MiniPC.Read_Byte_From_FIFO(&byte)) {
        switch (Cur_State) {
            case STATE_WAIT_HEAD0:
                if (byte == 0x5A) Cur_State = STATE_WAIT_HEAD1;
                break;

            case STATE_WAIT_HEAD1:
                if (byte == 0xA5) Cur_State = STATE_WAIT_LEN;
                else if (byte == 0x5A) Cur_State = STATE_WAIT_HEAD1; // 容错
                else Cur_State = STATE_WAIT_HEAD0;
                break;

            case STATE_WAIT_LEN:
                Rx_Len = byte;
                // 防止长度异常导致内存溢出
                if(Rx_Len > 32) Cur_State = STATE_WAIT_HEAD0;
                else Cur_State = STATE_WAIT_FUNC;
                break;

            case STATE_WAIT_FUNC:
                Rx_Func = byte;
                Rx_Data_Index = 0;
                Cur_State = STATE_WAIT_DATA;
                break;

            case STATE_WAIT_DATA:
                Rx_Data_Buffer[Rx_Data_Index++] = byte;
                if (Rx_Data_Index >= Rx_Len) {
                    Cur_State = STATE_WAIT_CRC_H;
                }
                break;

            case STATE_WAIT_CRC_H:
                Rx_CRC_Rec = byte << 8;
                Cur_State = STATE_WAIT_CRC_L;
                break;

            case STATE_WAIT_CRC_L:
                Rx_CRC_Rec |= byte;

                // --- 开始严谨的 CRC16 校验 ---
                {
                    uint8_t check_frame[64];
                    check_frame[0] = 0x5A;
                    check_frame[1] = 0xA5;
                    check_frame[2] = Rx_Len;
                    check_frame[3] = Rx_Func;
                    memcpy(&check_frame[4], Rx_Data_Buffer, Rx_Len);

                    // 计算包括帧头、长度、功能码、数据在内的全部内容的 CRC16
                    // 总计算长度 = 4 (头+Len+Func) + Rx_Len (数据长)
                    uint16_t cal_crc = Get_CRC16_Check_Sum(check_frame, 4 + Rx_Len, 0xFFFF);

                    // 校验对比
                    if (cal_crc == Rx_CRC_Rec) {
                        // 确认功能码和长度都是我们期望的速度指令
                        if (Rx_Func == 0x10 && Rx_Len == 12) {
                            memcpy(&target_vx, &Rx_Data_Buffer[0], 4);
                            memcpy(&target_vy, &Rx_Data_Buffer[4], 4);
                            memcpy(&target_vw, &Rx_Data_Buffer[8], 4);

                            Alive_Count = 500; // 成功解析，重置看门狗
                        }
                    }
                }

                // 无论成功与否，重置状态机寻找下一包
                Cur_State = STATE_WAIT_HEAD0;
                break;

            default:
                Cur_State = STATE_WAIT_HEAD0;
                break;
        }
    }
}