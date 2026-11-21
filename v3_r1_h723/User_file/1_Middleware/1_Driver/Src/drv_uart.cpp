/**
 * @file drv_uart.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief 仿照SCUT-Robotlab改写的UART通信初始化与配置流程
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 * @date 2023-11-18 1.1 修改成cpp
 * @date 2024-05-05 1.2 新增错误中断, 24赛季定稿
 * @date 2024-08-22 2.1 新增回调函数空指针判定
 *
 * @copyright USTC-RoboWalker (c) 2023-2024
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "drv_uart.h"

/* Private macros ------------------------------------------------------------*/
#if defined(__GNUC__)
#define SECTION_RAM_D1 __attribute__((section(".ram_d1"), aligned(32)))
#else
    #define SECTION_RAM_D1
#endif
/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
SECTION_RAM_D1 Struct_UART_Manage_Object UART1_Manage_Object = {0};
SECTION_RAM_D1 Struct_UART_Manage_Object UART2_Manage_Object = {0};
SECTION_RAM_D1 Struct_UART_Manage_Object UART3_Manage_Object = {0};
SECTION_RAM_D1 Struct_UART_Manage_Object UART4_Manage_Object = {0};
SECTION_RAM_D1 Struct_UART_Manage_Object UART5_Manage_Object = {0};
SECTION_RAM_D1 Struct_UART_Manage_Object UART6_Manage_Object = {0};
SECTION_RAM_D1 Struct_UART_Manage_Object UART7_Manage_Object = {0};
SECTION_RAM_D1 Struct_UART_Manage_Object UART8_Manage_Object = {0};
SECTION_RAM_D1 Struct_UART_Manage_Object UART9_Manage_Object = {0};
SECTION_RAM_D1 Struct_UART_Manage_Object UART10_Manage_Object = {0};
/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化UART
 *
 * @param huart UART编号
 * @param Callback_Function 处理回调函数
 * @param Rx_Buffer_Length 接收缓冲区长度
 */
void UART_Init(UART_HandleTypeDef *huart, UART_Call_Back Callback_Function, uint16_t Rx_Buffer_Length)
{
    if (huart->Instance == USART1)
    {
        memset(&UART1_Manage_Object, 0, sizeof(Struct_UART_Manage_Object));
        UART1_Manage_Object.UART_Handler = huart;
        UART1_Manage_Object.Callback_Function = Callback_Function;
        UART1_Manage_Object.Rx_Buffer_Length = Rx_Buffer_Length;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART1_Manage_Object.Rx_Buffer, UART1_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == USART2)
    {
        memset(&UART2_Manage_Object, 0, sizeof(Struct_UART_Manage_Object));
        UART2_Manage_Object.UART_Handler = huart;
        UART2_Manage_Object.Callback_Function = Callback_Function;
        UART2_Manage_Object.Rx_Buffer_Length = Rx_Buffer_Length;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART2_Manage_Object.Rx_Buffer, UART2_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == USART3)
    {
        memset(&UART3_Manage_Object, 0, sizeof(Struct_UART_Manage_Object));
        UART3_Manage_Object.UART_Handler = huart;
        UART3_Manage_Object.Callback_Function = Callback_Function;
        UART3_Manage_Object.Rx_Buffer_Length = Rx_Buffer_Length;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART3_Manage_Object.Rx_Buffer, UART3_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == UART4)
    {
        memset(&UART4_Manage_Object, 0, sizeof(Struct_UART_Manage_Object));
        UART4_Manage_Object.UART_Handler = huart;
        UART4_Manage_Object.Callback_Function = Callback_Function;
        UART4_Manage_Object.Rx_Buffer_Length = Rx_Buffer_Length;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART4_Manage_Object.Rx_Buffer, UART4_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == UART5)
    {
        memset(&UART5_Manage_Object, 0, sizeof(Struct_UART_Manage_Object));
        UART5_Manage_Object.UART_Handler = huart;
        UART5_Manage_Object.Callback_Function = Callback_Function;
        UART5_Manage_Object.Rx_Buffer_Length = Rx_Buffer_Length;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART5_Manage_Object.Rx_Buffer, UART5_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == USART6)
    {
        memset(&UART6_Manage_Object, 0, sizeof(Struct_UART_Manage_Object));
        UART6_Manage_Object.UART_Handler = huart;
        UART6_Manage_Object.Callback_Function = Callback_Function;
        UART6_Manage_Object.Rx_Buffer_Length = Rx_Buffer_Length;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART6_Manage_Object.Rx_Buffer, UART6_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == UART7)
    {
        memset(&UART7_Manage_Object, 0, sizeof(Struct_UART_Manage_Object));
        UART7_Manage_Object.UART_Handler = huart;
        UART7_Manage_Object.Callback_Function = Callback_Function;
        UART7_Manage_Object.Rx_Buffer_Length = Rx_Buffer_Length;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART7_Manage_Object.Rx_Buffer, UART7_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == UART8)
    {
        memset(&UART8_Manage_Object, 0, sizeof(Struct_UART_Manage_Object));
        UART8_Manage_Object.UART_Handler = huart;
        UART8_Manage_Object.Callback_Function = Callback_Function;
        UART8_Manage_Object.Rx_Buffer_Length = Rx_Buffer_Length;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART8_Manage_Object.Rx_Buffer, UART8_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == UART9)
    {
        memset(&UART9_Manage_Object, 0, sizeof(Struct_UART_Manage_Object));
        UART9_Manage_Object.UART_Handler = huart;
        UART9_Manage_Object.Callback_Function = Callback_Function;
        UART9_Manage_Object.Rx_Buffer_Length = Rx_Buffer_Length;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART9_Manage_Object.Rx_Buffer, UART9_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == USART10)
    {
        memset(&UART10_Manage_Object, 0, sizeof(Struct_UART_Manage_Object));
        UART10_Manage_Object.UART_Handler = huart;
        UART10_Manage_Object.Callback_Function = Callback_Function;
        UART10_Manage_Object.Rx_Buffer_Length = Rx_Buffer_Length;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART10_Manage_Object.Rx_Buffer, UART10_Manage_Object.Rx_Buffer_Length);
    }
}

/**
 * @brief 掉线重新初始化UART
 *
 * @param huart UART编号
 */
void UART_Reinit(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART1_Manage_Object.Rx_Buffer, UART1_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == USART2)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART2_Manage_Object.Rx_Buffer, UART2_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == USART3)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART3_Manage_Object.Rx_Buffer, UART3_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == UART4)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART4_Manage_Object.Rx_Buffer, UART4_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == UART5)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART5_Manage_Object.Rx_Buffer, UART5_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == USART6)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART6_Manage_Object.Rx_Buffer, UART6_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == UART7)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART7_Manage_Object.Rx_Buffer, UART7_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == UART8)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART8_Manage_Object.Rx_Buffer, UART8_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == UART9)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART9_Manage_Object.Rx_Buffer, UART9_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == USART10)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART10_Manage_Object.Rx_Buffer, UART10_Manage_Object.Rx_Buffer_Length);
    }
}

/**
 * @brief 发送数据帧
 *
 * @param huart UART编号
 * @param Data 被发送的数据指针
 * @param Length 长度
 * @return uint8_t 执行状态
 */
uint8_t UART_Send_Data(UART_HandleTypeDef *huart, uint8_t *Data, uint16_t Length)
{
    // SCB_CleanDCache_by_Addr((uint32_t *)Data, Length);

    return (HAL_UART_Transmit_DMA(huart, Data, Length));
}

/**
 * @brief UART的TIM定时器中断发送回调函数
 *
 */
void TIM_1ms_UART_PeriodElapsedCallback()
{

}

/**
 * @brief HAL库UART接收DMA空闲中断
 *
 * @param huart UART编号
 * @param Size 长度
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    // 判断程序初始化完成
    if (init_finished == false)
    {
        return;
    }

    if (huart->Instance == USART1)
    {
        // SCB_InvalidateDCache_by_Addr((uint32_t *)UART1_Manage_Object.Rx_Buffer, UART_BUFFER_SIZE);
        if(UART1_Manage_Object.Callback_Function != nullptr)
        {
            UART1_Manage_Object.Callback_Function(UART1_Manage_Object.Rx_Buffer, Size);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART1_Manage_Object.Rx_Buffer, UART1_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == USART2)
    {
        // SCB_InvalidateDCache_by_Addr((uint32_t *)UART2_Manage_Object.Rx_Buffer, UART_BUFFER_SIZE);

        if(UART2_Manage_Object.Callback_Function != nullptr)
        {
            UART2_Manage_Object.Callback_Function(UART2_Manage_Object.Rx_Buffer, Size);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART2_Manage_Object.Rx_Buffer, UART2_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == USART3)
    {
        // SCB_InvalidateDCache_by_Addr((uint32_t *)UART3_Manage_Object.Rx_Buffer, UART_BUFFER_SIZE);

        if(UART3_Manage_Object.Callback_Function != nullptr)
        {
            UART3_Manage_Object.Callback_Function(UART3_Manage_Object.Rx_Buffer, Size);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART3_Manage_Object.Rx_Buffer, UART3_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == UART4)
    {
        // SCB_InvalidateDCache_by_Addr((uint32_t *)UART4_Manage_Object.Rx_Buffer, UART_BUFFER_SIZE);

        if(UART4_Manage_Object.Callback_Function != nullptr)
        {
            UART4_Manage_Object.Callback_Function(UART4_Manage_Object.Rx_Buffer, Size);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART4_Manage_Object.Rx_Buffer, UART4_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == UART5)
    {
        // SCB_InvalidateDCache_by_Addr((uint32_t *)UART5_Manage_Object.Rx_Buffer, UART_BUFFER_SIZE);

        if(UART5_Manage_Object.Callback_Function != nullptr)
        {
            UART5_Manage_Object.Callback_Function(UART5_Manage_Object.Rx_Buffer, Size);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART5_Manage_Object.Rx_Buffer, UART5_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == USART6)
    {
        // SCB_InvalidateDCache_by_Addr((uint32_t *)UART6_Manage_Object.Rx_Buffer, UART_BUFFER_SIZE);

        if(UART6_Manage_Object.Callback_Function != nullptr)
        {
            UART6_Manage_Object.Callback_Function(UART6_Manage_Object.Rx_Buffer, Size);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART6_Manage_Object.Rx_Buffer, UART6_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == UART7)
    {
        // SCB_InvalidateDCache_by_Addr((uint32_t *)UART7_Manage_Object.Rx_Buffer, UART_BUFFER_SIZE);

        if(UART7_Manage_Object.Callback_Function != nullptr)
        {
            UART7_Manage_Object.Callback_Function(UART7_Manage_Object.Rx_Buffer, Size);
        }
        // HAL_UARTEx_ReceiveToIdle_DMA(huart, UART7_Manage_Object.Rx_Buffer, UART7_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == UART8)
    {
        // SCB_InvalidateDCache_by_Addr((uint32_t *)UART8_Manage_Object.Rx_Buffer, UART_BUFFER_SIZE);

        if(UART8_Manage_Object.Callback_Function != nullptr)
        {
            UART8_Manage_Object.Callback_Function(UART8_Manage_Object.Rx_Buffer, Size);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART8_Manage_Object.Rx_Buffer, UART8_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == UART9)
    {
        // SCB_InvalidateDCache_by_Addr((uint32_t *)UART9_Manage_Object.Rx_Buffer, UART_BUFFER_SIZE);

        if(UART9_Manage_Object.Callback_Function != nullptr)
        {
            UART9_Manage_Object.Callback_Function(UART9_Manage_Object.Rx_Buffer, Size);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART9_Manage_Object.Rx_Buffer, UART9_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == USART10)
    {
        // SCB_InvalidateDCache_by_Addr((uint32_t *)UART10_Manage_Object.Rx_Buffer, UART_BUFFER_SIZE);

        if(UART10_Manage_Object.Callback_Function != nullptr)
        {
            UART10_Manage_Object.Callback_Function(UART10_Manage_Object.Rx_Buffer, Size);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART10_Manage_Object.Rx_Buffer, UART10_Manage_Object.Rx_Buffer_Length);
    }
}

/**
 * @brief HAL库UART错误中断
 *
 * @param huart UART编号
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (init_finished == false) {
        return;
    }
    if (huart->Instance == USART1)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART1_Manage_Object.Rx_Buffer, UART1_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == USART2)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART2_Manage_Object.Rx_Buffer, UART2_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == USART3)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART3_Manage_Object.Rx_Buffer, UART3_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == UART4)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART4_Manage_Object.Rx_Buffer, UART4_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == UART5)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART5_Manage_Object.Rx_Buffer, UART5_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == USART6)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART6_Manage_Object.Rx_Buffer, UART6_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == UART7)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART7_Manage_Object.Rx_Buffer, UART7_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == UART8)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART8_Manage_Object.Rx_Buffer, UART8_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == UART9)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART9_Manage_Object.Rx_Buffer, UART9_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == USART10)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART10_Manage_Object.Rx_Buffer, UART10_Manage_Object.Rx_Buffer_Length);
    }
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
