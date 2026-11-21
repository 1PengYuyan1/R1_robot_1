#ifndef DVC_LOGF710_H
#define DVC_LOGF710_H

#include "drv_minipc.h" // 需要引用驱动层来读取数据
#include "string.h"

// 协议常量
#define PROTOCOL_HEAD_0     0xAA
#define PROTOCOL_HEAD_1     0x55
#define FUNC_CHASSIS        0x10
#define FUNC_BUTTONS        0x11


#define LogF710_Key_IDLE  0x0000
#define LogF710_Key_X  0x0001
#define LogF710_Key_A  0x0002
#define LogF710_Key_B  0x0003
#define LogF710_Key_Y  0x0004
#define LogF710_Key_LB  0x0010
#define LogF710_Key_LT  0x0020
#define LogF710_Key_RB  0x0030
#define LogF710_Key_RT  0x0040
#define LogF710_Key_Back   0x0100
#define LogF710_Key_Start  0x0200
#define LogF710_Key_Right  0x1000
#define LogF710_Key_Left   0x2000
#define LogF710_Key_Up     0x3000
#define LogF710_Key_Down   0x4000
/**
 * @brief 遥控器开关
 *
 */
enum Enum_LogF710_Status {
    LogF710_Status_DISABLE = 0,
    LogF710_Status_ENABLE,
};

/**
 * @brief 按键状态
 *
 */
enum Enum_LogF710_Key_Status {
    LogF710_Key_Status_IDLE = 0,
    LogF710_Key_Status_X,
    LogF710_Key_Status_A,
    LogF710_Key_Status_B,
    LogF710_Key_Status_Y,
    LogF710_Key_Status_LB,
    LogF710_Key_Status_LT,
    LogF710_Key_Status_RB,
    LogF710_Key_Status_RT,
    LogF710_Key_Status_Start,
    LogF710_Key_Status_Back,
    LogF710_Key_Status_Up,
    LogF710_Key_Status_Down,
    LogF710_Key_Status_Left,
    LogF710_Key_Status_Right,
};


// F710 原数据数据
struct Struct_LogF710_UART_Data {
    float Vx;
    float Vy;
    float Omega;
    uint16_t Buttons;
};

// F710 数据
struct Struct_LogF710_Data {
    float Vx;
    float Vy;
    float Omega;
    uint16_t Buttons;
};

class Class_LogF710 {
public:

    bool Is_ROS_Control_Enabled = false; // 记录当前是否允许 ROS 控制的状态位

    void Init();

    void Judge_Status();

    void USART_Data_Process();

    void Data_Process();

    void Resolve_Key_Status(uint16_t raw) ;

    // 获取数据接口
    inline const Struct_LogF710_UART_Data *Get_Data() { return &LogF710_UART_RX_Data; }
    inline bool Is_Connected() { return Alive_Count > 0; }

    void TIM_100ms_Alive_PeriodElapsedCallback();
    void TIM_1ms_Calculate_PeriodElapsedCallback();

    inline Enum_LogF710_Status Get_Status() ;
    inline Enum_LogF710_Key_Status Get_Key_Status();

    inline uint16_t Get_Button_Status();
    inline float Get_Left_X() ;
    inline float Get_Left_Y() ;
    inline float Get_Right_X();

private:
    bool Start_Buzzer_Played = false;
    bool Back_Buzzer_Played = false;

    uint8_t Rx_Len;
    uint8_t Rx_Func;
    uint8_t Rx_Data_Buffer[128]; // 临时存 Payload
    uint8_t Rx_Data_Index;
    uint16_t Rx_CRC_Rec;
    uint16_t Rx_CRC_Cal;

    float Robot_RotationMax = 0.04; // 最大线速度
    float Robot_VelocityMax = 0.04; // 最大角速度

    uint32_t Start_Press_Timer = 0;  // START 键计时器
    uint32_t Back_Press_Timer = 0;   // BACK 键计时器

    Struct_LogF710_UART_Data LogF710_UART_RX_Data;
    Struct_LogF710_Data LogF710_Data;

    int16_t Alive_Count; // 看门狗计数
    int16_t crc_ok;

    Enum_LogF710_Status LogF710_Status = LogF710_Status_DISABLE;
    Enum_LogF710_Key_Status LogF710_Key_Status = LogF710_Key_Status_IDLE;


    // 状态机定义
    enum Enum_State {
        STATE_WAIT_HEAD0,
        STATE_WAIT_HEAD1,
        STATE_WAIT_LEN,
        STATE_WAIT_FUNC,
        STATE_WAIT_DATA,
        STATE_WAIT_CRC_H,
        STATE_WAIT_CRC_L
    } Cur_State;


};

inline Enum_LogF710_Status Class_LogF710::Get_Status() {
    return (LogF710_Status);
}
inline Enum_LogF710_Key_Status Class_LogF710::Get_Key_Status()
{
    return (LogF710_Key_Status);

}
inline uint16_t Class_LogF710::Get_Button_Status() {
    return (LogF710_UART_RX_Data.Buttons);
}

inline float Class_LogF710::Get_Left_X() {
    return (LogF710_Data.Vx);
}


inline float Class_LogF710::Get_Left_Y() {
    return (LogF710_Data.Vy);
}

inline float Class_LogF710::Get_Right_X() {
    return (LogF710_Data.Omega);
}



#endif
