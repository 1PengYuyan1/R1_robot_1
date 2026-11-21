/**
 * @file DR16.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief 遥控器DR16
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright USTC-RoboWalker (c) 2023
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_dr16.h"

SECTION_RAM_D1 static uint8_t DR16_Rx_Buffer[36] __attribute__((aligned(32)));
/**
 * @brief 遥控器DR16初始化
 *
 * @param huart 指定的UART
 */

void Class_DR16::Init(UART_HandleTypeDef *huart)
{
        Alive_Count = 0;
        if (huart->Instance == USART1)UART_Manage_Object = &UART1_Manage_Object;
        else if (huart->Instance == USART2)UART_Manage_Object = &UART2_Manage_Object;
        else if (huart->Instance == USART3)UART_Manage_Object = &UART3_Manage_Object;
        else if (huart->Instance == UART4)UART_Manage_Object = &UART4_Manage_Object;
        else if (huart->Instance == UART5)UART_Manage_Object = &UART5_Manage_Object;
        else if (huart->Instance == USART6)UART_Manage_Object = &UART6_Manage_Object;
        else if (huart->Instance == UART7)UART_Manage_Object = &UART7_Manage_Object;
        else if (huart->Instance == UART8)UART_Manage_Object = &UART8_Manage_Object;

}


bool Class_DR16::Is_Data_Valid(uint8_t *buf)
{
    uint16_t ch0 = (buf[0] | (buf[1] << 8)) & 0x07FF;
    uint16_t ch1 = ((buf[1] >> 3) | (buf[2] << 5)) & 0x07FF;
    uint16_t ch2 = ((buf[2] >> 6) | (buf[3] << 2) | (buf[4] << 10)) & 0x07FF;
    uint16_t ch3 = ((buf[4] >> 1) | (buf[5] << 7)) & 0x07FF;
    uint8_t sw2 = ((buf[5] >> 4) & 0x03);
    uint8_t sw1 = ((buf[5] >> 6) & 0x03);

    // 特征1：大疆拨杆只有 1(上), 3(中), 2(下)，绝对不可能解析出 0
    if (sw1 == 0 || sw2 == 0) return false;

    // 特征2：摇杆数值绝对在 [364, 1684] 之间 (中位是1024)
    // 如果发生错位，拼凑出来的数值会极大(>1700)或极小(<300)
    if (ch0 < 300 || ch0 > 1750) return false;
    if (ch1 < 300 || ch1 > 1750) return false;
    if (ch2 < 300 || ch2 > 1750) return false;
    if (ch3 < 300 || ch3 > 1750) return false;

    return true; // 完全吻合，是一帧完美对齐的数据！
}

/**
 * @brief UART通信接收回调函数
 *
 * @param Rx_Data 接收的数据
 */
void Class_DR16::UART_RxCpltCallback(uint8_t *Rx_Data, uint16_t Length)
{
    last_update_time = HAL_GetTick(); // 把它加在这里！只有收到数据才更新时间
    for (uint16_t i = 0; i < Length; i++) {
        DR16_FIFO[DR16_FIFO_Head] = Rx_Data[i];
        DR16_FIFO_Head = (DR16_FIFO_Head + 1) % 128; // 128为FIFO大小
    }
    Flag += 1;
}

/**
 * @brief TIM定时器中断定期检测遥控器DR16是否存活
 *
 */
void Class_DR16::TIM_100ms_Alive_PeriodElapsedCallback()
{
    // 喂狗逻辑：每次调用递减
    if (Alive_Count > 0) Alive_Count--;
    if (HAL_GetTick() - last_update_time > TIMEOUT_MS) {
        DR16_Status = DR16_Status_DISABLE;
    } else {
        DR16_Status = DR16_Status_ENABLE;
    }
}

/**
 * @brief 定时器计算函数
 *
 */
void Class_DR16::TIM_1ms_Calculate_PeriodElapsedCallback()
{

    Data_Process();
    {
        // uint8_t *buf = DR16_Rx_Buffer;
        //
        // // 提取拨杆状态 (大疆协议位于第6字节的高4位)
        // uint8_t current_sw2 = ((buf[5] >> 4) & 0x03); // 原 Switch_2 (右拨杆)
        // uint8_t current_sw1 = ((buf[5] >> 6) & 0x03); // 原 Switch_1 (左拨杆)
        //
        // // 判断拨码触发边沿
        // _Judge_Switch(&Data.Left_Switch, current_sw1, Pre_UART_Rx_Data.Switch_1);
        // _Judge_Switch(&Data.Right_Switch, current_sw2, Pre_UART_Rx_Data.Switch_2);
        //
        // // 保留状态供下一次边沿检测 (此处仅需更新这两个状态变量即可，无需memcpy)
        // Pre_UART_Rx_Data.Switch_1 = current_sw1;
        // Pre_UART_Rx_Data.Switch_2 = current_sw2;

        // // 数据处理过程
        // Struct_DR16_UART_Data *tmp_buffer = (Struct_DR16_UART_Data *) DR16_Rx_Buffer;
        //
        // // // 判断拨码触发
        // _Judge_Switch(&Data.Left_Switch, tmp_buffer->Switch_1, Pre_UART_Rx_Data.Switch_1);
        // _Judge_Switch(&Data.Right_Switch, tmp_buffer->Switch_2, Pre_UART_Rx_Data.Switch_2);
        //
        // // 保留数据
        // memcpy(&Pre_UART_Rx_Data, tmp_buffer, 18 * sizeof(uint8_t));
    }

    }

/**
 * @brief 数据处理过程
 *
 */
void Class_DR16::Data_Process()
{

while (DR16_FIFO_Tail != DR16_FIFO_Head)
    {
        // 从 FIFO 读取一个字节
        uint8_t byte = DR16_FIFO[DR16_FIFO_Tail];
        DR16_FIFO_Tail = (DR16_FIFO_Tail + 1) % 128;

        // 移位缓冲区 (相当于水管往前挪一步)
        for (int i = 0; i < 17; i++) {
            shift_buf[i] = shift_buf[i+1];
        }
        shift_buf[17] = byte;

        if (shift_count < 18) {
            shift_count++;
        }

        // 当移位缓冲区凑够 18 个字节时，进行特征验证
        if (shift_count >= 18)
        {
            if (Is_Data_Valid(shift_buf))
            {
                // 验证通过！拷贝到接收缓存
                memcpy(DR16_Rx_Buffer, shift_buf, 18);

                // --- 提取摇杆数据 ---
                uint16_t ch0 = (shift_buf[0] | (shift_buf[1] << 8)) & 0x07FF;
                uint16_t ch1 = ((shift_buf[1] >> 3) | (shift_buf[2] << 5)) & 0x07FF;
                uint16_t ch2 = ((shift_buf[2] >> 6) | (shift_buf[3] << 2) | (shift_buf[4] << 10)) & 0x07FF;
                uint16_t ch3 = ((shift_buf[4] >> 1) | (shift_buf[5] << 7)) & 0x07FF;
                uint16_t ch_yaw = (shift_buf[16] | (shift_buf[17] << 8)) & 0x07FF;

                // Data.Right_X = (ch0 - Rocker_Offset) / Rocker_Num * Remote_Link * Robot_VelocityMax;
                // Data.Right_Y = (ch1 - Rocker_Offset) / Rocker_Num  * Remote_Link * Robot_VelocityMax;
                // Data.Left_X  = (ch2 - Rocker_Offset) / Rocker_Num  * Remote_Link * Robot_VelocityMax;
                // Data.Left_Y  = (ch3 - Rocker_Offset) / Rocker_Num  * Remote_Link * Robot_VelocityMax;
                // Data.Yaw     = (ch_yaw - Rocker_Offset) / Rocker_Num  * Remote_Link * Robot_VelocityMax;

                Data.Right_X = (ch0 - Rocker_Offset) / Rocker_Num * Remote_Link;
                Data.Right_Y = (ch1 - Rocker_Offset) / Rocker_Num  * Remote_Link ;
                Data.Left_X  = (ch2 - Rocker_Offset) / Rocker_Num  * Remote_Link ;
                Data.Left_Y  = (ch3 - Rocker_Offset) / Rocker_Num  * Remote_Link ;
                Data.Yaw     = (ch_yaw - Rocker_Offset) / Rocker_Num  * Remote_Link ;
                // --- 提取拨杆边沿数据 ---
                uint8_t current_sw2 = ((shift_buf[5] >> 4) & 0x03);
                uint8_t current_sw1 = ((shift_buf[5] >> 6) & 0x03);
                _Judge_Switch(&Data.Left_Switch, current_sw1, Pre_UART_Rx_Data.Switch_1);
                _Judge_Switch(&Data.Right_Switch, current_sw2, Pre_UART_Rx_Data.Switch_2);
                Pre_UART_Rx_Data.Switch_1 = current_sw1;
                Pre_UART_Rx_Data.Switch_2 = current_sw2;

                // 核心：验证成功后，清空计数器！
                // 这样程序会重新等待 18 个全新的字节，而不会重复解析包含旧数据的帧。
                shift_count = 0;
            }
            // 如果 Is_Data_Valid 为 false，说明数据错位。
            // shift_count 依然是 18，下一轮循环会读入一个新字节，顶掉最旧的一个字节继续验证，实现无缝滑动对齐。
        }
    }

    {
        // uint8_t *buf = DR16_Rx_Buffer;
        //
        // // 1. 提取摇杆信息 (无视编译器差异)
        // uint16_t ch0 = (buf[0] | (buf[1] << 8)) & 0x07FF;
        // uint16_t ch1 = ((buf[1] >> 3) | (buf[2] << 5)) & 0x07FF;
        // uint16_t ch2 = ((buf[2] >> 6) | (buf[3] << 2) | (buf[4] << 10)) & 0x07FF;
        // uint16_t ch3 = ((buf[4] >> 1) | (buf[5] << 7)) & 0x07FF;
        //
        // // 提取左上角拨轮 (第17、18字节)
        // uint16_t ch_yaw = (buf[16] | (buf[17] << 8)) & 0x07FF;
        //
        // Data.Right_X = (ch0 - Rocker_Offset) / Rocker_Num * Robot_VelocityMax;
        // Data.Right_Y = (ch1 - Rocker_Offset) / Rocker_Num * Robot_VelocityMax;
        // Data.Left_X  = (ch2 - Rocker_Offset) / Rocker_Num * Robot_VelocityMax;
        // Data.Left_Y  = (ch3 - Rocker_Offset) / Rocker_Num * Robot_VelocityMax;
        // Data.Yaw     = (ch_yaw - Rocker_Offset) / Rocker_Num * Robot_VelocityMax;;

        // // 数据处理过程
        // Struct_DR16_UART_Data *tmp_buffer = (Struct_DR16_UART_Data *) DR16_Rx_Buffer;
        //
        // // 摇杆信息
        // Data.Right_X = (tmp_buffer->Channel_0 - Rocker_Offset) / Rocker_Num * Robot_VelocityMax ;
        // Data.Right_Y = (tmp_buffer->Channel_1 - Rocker_Offset) / Rocker_Num * Robot_VelocityMax ;
        // Data.Left_X = (tmp_buffer->Channel_2 - Rocker_Offset) / Rocker_Num  * Robot_VelocityMax ;
        // Data.Left_Y = (tmp_buffer->Channel_3 - Rocker_Offset) / Rocker_Num  * Robot_VelocityMax ;
        //
        // // 左前轮信息
        // Data.Yaw = (tmp_buffer->Channel_Yaw - Rocker_Offset) / Rocker_Num;
        //
    }
}


/**
 * @brief 判断拨动开关状态
 *
 */
void Class_DR16::_Judge_Switch(Enum_DR16_Switch_Status *Switch, uint8_t Status, uint8_t Pre_Status)
{
    // 带触发的判断
    switch (Pre_Status)
    {
    case (DR16_SWITCH_UP):
    {
        switch (Status)
        {
        case (DR16_SWITCH_UP):
        {
            *Switch = DR16_Switch_Status_UP;

            break;
        }
        case (DR16_SWITCH_DOWN):
        {
            *Switch = DR16_Switch_Status_TRIG_MIDDLE_DOWN;

            break;
        }
        case (DR16_SWITCH_MIDDLE):
        {
            *Switch = DR16_Switch_Status_TRIG_UP_MIDDLE;

            break;
        }
        }

        break;
    }
    case (DR16_SWITCH_DOWN):
    {
        switch (Status)
        {
        case (DR16_SWITCH_UP):
        {
            *Switch = DR16_Switch_Status_TRIG_MIDDLE_UP;

            break;
        }
        case (DR16_SWITCH_DOWN):
        {
            *Switch = DR16_Switch_Status_DOWN;

            break;
        }
        case (DR16_SWITCH_MIDDLE):
        {
            *Switch = DR16_Switch_Status_TRIG_DOWN_MIDDLE;

            break;
        }
        }

        break;
    }
    case (DR16_SWITCH_MIDDLE):
    {
        switch (Status)
        {
        case (DR16_SWITCH_UP):
        {
            *Switch = DR16_Switch_Status_TRIG_MIDDLE_UP;

            break;
        }
        case (DR16_SWITCH_DOWN):
        {
            *Switch = DR16_Switch_Status_TRIG_MIDDLE_DOWN;

            break;
        }
        case (DR16_SWITCH_MIDDLE):
        {
            *Switch = DR16_Switch_Status_MIDDLE;

            break;
        }
        }

        break;
    }
    }
}

/**
 * @brief 判断按键状态
 *
 */
void Class_DR16::_Judge_Key(Enum_DR16_Key_Status *Key, uint8_t Status, uint8_t Pre_Status)
{
    // 带触发的判断
    switch (Pre_Status)
    {
    case (DR16_KEY_FREE):
    {
        switch (Status)
        {
        case (DR16_KEY_FREE):
        {
            *Key = DR16_Key_Status_FREE;

            break;
        }
        case (DR16_KEY_PRESSED):
        {
            *Key = DR16_Key_Status_TRIG_FREE_PRESSED;

            break;
        }
        }

        break;
    }
    case (DR16_KEY_PRESSED):
    {
        switch (Status)
        {
        case (DR16_KEY_FREE):
        {
            *Key = DR16_Key_Status_TRIG_PRESSED_FREE;

            break;
        }
        case (DR16_KEY_PRESSED):
        {
            *Key = DR16_Key_Status_PRESSED;

            break;
        }
        }

        break;
    }
    }
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
