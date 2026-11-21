#ifndef DRV_CAN_H
#define DRV_CAN_H

#include "stm32h7xx_hal.h"
#include "fdcan.h"
#include <string.h>


struct Struct_FDCAN_Rx_Buffer
{
    FDCAN_RxHeaderTypeDef Header; // H7 FDCAN 头类型
    uint8_t Data[64];
};

/**
 * @brief CAN通信接收回调函数数据类型
 *
 */
typedef void (*CAN_Call_Back)(Struct_FDCAN_Rx_Buffer *);

/**
 * @brief CAN通信处理结构体
 *
 */
struct Struct_FDCAN_Manage_Object
{
    FDCAN_HandleTypeDef *CAN_Handler; // H7 FDCAN 句柄
    Struct_FDCAN_Rx_Buffer Rx_Buffer;
    CAN_Call_Back Callback_Function;
};


extern bool init_finished; // 需要你在 main.c 或其他地方定义这个变量

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

extern Struct_FDCAN_Manage_Object CAN1_Manage_Object;
extern Struct_FDCAN_Manage_Object CAN2_Manage_Object;
extern Struct_FDCAN_Manage_Object CAN3_Manage_Object;


void CAN_Init(FDCAN_HandleTypeDef *hfdcan, CAN_Call_Back Callback_Function);

uint8_t CAN_Send_Data(FDCAN_HandleTypeDef *hfdcan, uint16_t ID, uint8_t *Data, uint16_t Length);

void TIM_1ms_CAN_PeriodElapsedCallback();

#endif
