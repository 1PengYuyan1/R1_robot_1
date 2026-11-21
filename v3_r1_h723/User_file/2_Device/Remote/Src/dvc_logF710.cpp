#include "dvc_logF710.h"
#include "dvc_buzzer.h"


//
const uint16_t wCRC_Table[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
    0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
    0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
    0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
    0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
    0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
    0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
    0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
    0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
    0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
    0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
    0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
    0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
    0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
    0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
    0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
    0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
    0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
    0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
    0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
    0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
    0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
    0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
};

void Class_LogF710::Init()
{
    Cur_State = STATE_WAIT_HEAD0;
    Alive_Count = 0;
    crc_ok = 0;
    memset(&LogF710_UART_RX_Data, 0.0f, sizeof(Struct_LogF710_UART_Data));
    memset(&LogF710_Data, 0.0f, sizeof(Struct_LogF710_Data));
    // 初始化驱动
    Drv_MiniPC.Init();
}

/**
 * @brief TIM定时器中断定期检测遥控器LogF710是否存活
 *
 */
void Class_LogF710::TIM_100ms_Alive_PeriodElapsedCallback()
{
    // 喂狗逻辑：每次调用递减
    if (Alive_Count > 0) Alive_Count--;

    // 判断该时间段内是否接收过遥控器LohF710数据
    if (Is_Connected())
        LogF710_Status = LogF710_Status_ENABLE;
    else
    {
        LogF710_Status = LogF710_Status_DISABLE;
        // 如果断连，也把按键状态复位
        LogF710_Key_Status = LogF710_Key_Status_IDLE;
        // Buzzer_Play_SystemError();
    }
}

/**
 * @brief 定时器计算函数
 *
 */
void Class_LogF710::TIM_1ms_Calculate_PeriodElapsedCallback()
{
    USART_Data_Process();
}

uint16_t btn_raw;

void Class_LogF710::Judge_Status()
{
    btn_raw = Get_Button_Status();

    // --- 长按 START 开启逻辑 (500ms) ---
    if (btn_raw & LogF710_Key_Start)
    {
        Start_Press_Timer++;
        if (Start_Press_Timer >= 500)
        {
            LogF710_Status = LogF710_Status_ENABLE;
            Is_ROS_Control_Enabled = true;
            // Buzzer_Play_Startup();
            Start_Buzzer_Played = true;
        }
    }
    else
    {
        Start_Press_Timer = 0;
        Start_Buzzer_Played = false;
    }

    // --- 长按 BACK 关闭逻辑 (500ms) ---
    if (btn_raw & LogF710_Key_Back)
    {
        Back_Press_Timer++;
        if (Back_Press_Timer >= 100)
        {
            LogF710_Status = LogF710_Status_DISABLE;
            Is_ROS_Control_Enabled = false;
            // Buzzer_Play_DeviceLost();
            Back_Buzzer_Played = true;
        }
    }
    else
    {
        Back_Press_Timer = 0;
        Back_Buzzer_Played = false;
    }
}


/**
 * @brief 数据处理过程
 *
 */
void Class_LogF710::USART_Data_Process()
{
    uint8_t byte;

    // 循环从驱动 FIFO 中读取所有可用字节
    while (Drv_MiniPC.Read_Byte_From_FIFO(&byte))
    {
        switch (Cur_State)
        {
        case STATE_WAIT_HEAD0:
            if (byte == PROTOCOL_HEAD_0) Cur_State = STATE_WAIT_HEAD1;
            break;

        case STATE_WAIT_HEAD1:
            if (byte == PROTOCOL_HEAD_1) Cur_State = STATE_WAIT_LEN;
            else Cur_State = STATE_WAIT_HEAD0;
            break;

        case STATE_WAIT_LEN:
            Rx_Len = byte;
            if (Rx_Len > 100)
            {
                // 长度保护
                Cur_State = STATE_WAIT_HEAD0;
            }
            else
            {
                Cur_State = STATE_WAIT_FUNC;
                // CRC 初始化 (根据你的协议，CRC 通常包含 Len 和 Func)
                Rx_CRC_Cal = 0xFFFF; // 初始值取决于你的 CRC 实现，这里假设是 Append 方式
            }
            break;

        case STATE_WAIT_FUNC:
            Rx_Func = byte;
            Rx_Data_Index = 0;
            Cur_State = (Rx_Len > 0) ? STATE_WAIT_DATA : STATE_WAIT_CRC_H;
            break;

        case STATE_WAIT_DATA:
            Rx_Data_Buffer[Rx_Data_Index++] = byte;
            if (Rx_Data_Index >= Rx_Len)
            {
                Cur_State = STATE_WAIT_CRC_H;
            }
            break;

        case STATE_WAIT_CRC_H:
            Rx_CRC_Rec = byte << 8;
            Cur_State = STATE_WAIT_CRC_L;
            break;

        case STATE_WAIT_CRC_L:
            Rx_CRC_Rec |= byte;

            {
                uint8_t checkBuf[128];
                checkBuf[0] = Rx_Len;
                checkBuf[1] = Rx_Func;
                memcpy(&checkBuf[2], Rx_Data_Buffer, Rx_Len);

                // 假设校验通过：
                bool crc_ok = true; // 需替换为实际校验代码

                if (crc_ok)
                {
                    Alive_Count = 50; // 重置看门狗

                    if (Rx_Func == FUNC_CHASSIS)
                    {
                        if (Rx_Len == 12) {
                            memcpy(&LogF710_UART_RX_Data.Vx, Rx_Data_Buffer, 12);
                        }
                        else if (Rx_Len == 14) {
                            memcpy(&LogF710_UART_RX_Data.Vx, Rx_Data_Buffer, 14);
                        }
                    }
                    else if (Rx_Func == FUNC_BUTTONS && Rx_Len == 2)
                    {
                        LogF710_UART_RX_Data.Buttons = Rx_Data_Buffer[0] | (Rx_Data_Buffer[1] << 8);
                    }

                    // 2. 进一步加工逻辑 (模仿 DR16)
                    // 假设从 MiniPC 传来的 float 是 -1.0 到 1.0 的摇杆位置
                    // 我们将其映射为实际速度： 比例 * 链路增益 * 最大速度
                    LogF710_Data.Vx    = LogF710_UART_RX_Data.Vx     * Robot_VelocityMax;
                    LogF710_Data.Vy    = LogF710_UART_RX_Data.Vy     * Robot_VelocityMax;
                    LogF710_Data.Omega = LogF710_UART_RX_Data.Omega  * Robot_RotationMax; // 角速度

                    // 3. 按键处理
                    Resolve_Key_Status(LogF710_UART_RX_Data.Buttons);

                }
            }
            Cur_State = STATE_WAIT_HEAD0;
            break;

        default:
            Cur_State = STATE_WAIT_HEAD0;
            break;
        }
    }
    if (crc_ok) {
        Alive_Count = 10; // 只要 1 秒内（10*100ms）有包就算在线
    }
}


void Class_LogF710::Data_Process()
{


    LogF710_Data.Vx    = LogF710_UART_RX_Data.Vx     * Robot_VelocityMax;
    LogF710_Data.Vy    = LogF710_UART_RX_Data.Vy     * Robot_VelocityMax;
    LogF710_Data.Omega = LogF710_UART_RX_Data.Omega  * Robot_RotationMax;
    // a =  LogF710_UART_RX_Data.Vx ;
    // b =  LogF710_UART_RX_Data.Vy ;
    // c =  LogF710_UART_RX_Data.Omega ;
    // d =  LogF710_UART_RX_Data.Buttons ;

}
/**
 * @brief 解析按键优先级，将原始位数据转换为状态枚举
 * 封装成函数后，Data_Process 会非常干净
 */
void Class_LogF710::Resolve_Key_Status(uint16_t raw)
{
    switch (raw)
    {
    case LogF710_Key_A:
        LogF710_Key_Status = LogF710_Key_Status_A;
        break;

    case LogF710_Key_B:
        LogF710_Key_Status = LogF710_Key_Status_B;
        break;

    case LogF710_Key_X:
        LogF710_Key_Status = LogF710_Key_Status_X;
        break;

    case LogF710_Key_Y:
        LogF710_Key_Status = LogF710_Key_Status_Y;
        break;

    case LogF710_Key_LB:
        LogF710_Key_Status = LogF710_Key_Status_LB;
        break;

    case LogF710_Key_RB:
        LogF710_Key_Status = LogF710_Key_Status_RB;
        break;

    case LogF710_Key_LT:
        LogF710_Key_Status = LogF710_Key_Status_LT;
        break;

    case LogF710_Key_RT:
        LogF710_Key_Status = LogF710_Key_Status_RT;
        break;

    case LogF710_Key_Back:
        LogF710_Key_Status = LogF710_Key_Status_Back;
        break;

    case LogF710_Key_Start:
        LogF710_Key_Status = LogF710_Key_Status_Start;
        break;

    case LogF710_Key_Up:
        LogF710_Key_Status = LogF710_Key_Status_Up;
        break;

    case LogF710_Key_Down:
        LogF710_Key_Status = LogF710_Key_Status_Down;
        break;

    case LogF710_Key_Right:
        LogF710_Key_Status = LogF710_Key_Status_Right;
        break;

    case LogF710_Key_Left:
        LogF710_Key_Status = LogF710_Key_Status_Left;
        break;

    default:
        LogF710_Key_Status = LogF710_Key_Status_IDLE;
        break;
    }
}

