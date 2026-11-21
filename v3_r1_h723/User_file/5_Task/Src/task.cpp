//
// Created by pzx on 2025/12/16.
//
#include "task.h"

#include "arm_math.h"
#include "crt_chassis.h"
#include "drv_fdcan.h"
#include "drv_tim.h"
#include "dvc_buzzer.h"
#include "dvc_dr16.h"
#include "dvc_encoder.h"
#include "dvc_motor_dm.h"
#include "dvc_odrive.h"
#include "fdcan.h"
#include "main.h"
#include "math.h"
#include "robot.h"

#if defined(__GNUC__)
#define SECTION_RAM_D1 __attribute__((section(".ram_d1"), aligned(32)))
#else
#define SECTION_RAM_D1
#endif

Class_Robot robot;
// Class_Motor_DM_Normal Motor;

bool init_finished = false;

void DR16_UART5_Callback(uint8_t* Buffer, uint16_t Length) {
  if (!init_finished) return;
  robot.DR16.UART_RxCpltCallback(Buffer, Length);
}

void OPS_UART7_Callback(uint8_t* Buffer, uint16_t Size) {
  if (!init_finished) return;
  robot.ops.onRxCircularEvent(Buffer, Size, 28);
}

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
  if (!init_finished) return;
  if (huart->Instance == USART10) robot.BoardComm.UART_RxCpltCallback();
}
/**
 * @brief TIM5任务回调函数
 *
 */
void Task1ms_TIM5_Callback() {
  if (!init_finished) return;

  // 模块保持存活
  static int interaction_mod2 = 0;
  static int alive_mod100 = 0;

  robot.TIM_1ms_Calculate_Callback();

  // Struct_Remote_Sync_Packet tx_pack;
  // static uint8_t gantry_sync_cnt = 0;
  // gantry_sync_cnt++;
  // if (gantry_sync_cnt >= 10)
  // {
  //
  //     gantry_sync_cnt = 0;
  //     tx_pack.header = 0xAA;
  //     tx_pack.right_switch = robot.gantry_right_switch;
  //     tx_pack.left_switch = robot.gantry_left_switch;
  //     tx_pack.tail = 0x55;
  //     // 判读底盘是否处于使能状态
  //     if (robot.LogF710.Is_ROS_Control_Enabled && robot.LogF710.Get_Status()
  //     != LogF710_Status_DISABLE) {
  //         tx_pack.dr16_status = 1;
  //     } else {
  //         tx_pack.dr16_status = 0;
  //     }
  //     robot.BoardComm.Send_Data((uint8_t*)&tx_pack, sizeof(tx_pack));
  // }

  interaction_mod2++;
  if (interaction_mod2 % 2 == 0) {
    interaction_mod2 = 0;

    robot.TIM_2ms_Calculate_PeriodElapsedCallback();
  }

  alive_mod100++;
  if (alive_mod100 % 100 == 0) {
    alive_mod100 = 0;

    robot.TIM_100ms_Alive_PeriodElapsedCallback();
  }
}

void Device_CAN1_Callback(Struct_FDCAN_Rx_Buffer* CAN_RxMessage) {
  switch (CAN_RxMessage->Header.Identifier) {
    case 0x11:
      robot.Chassis.Motor_Steer[0].CAN_RxCpltCallback(CAN_RxMessage->Data);
      // Motor.CAN_RxCpltCallback(CAN_RxMessage->Data);

      break;

    case 0x12:
      robot.Chassis.Motor_Steer[1].CAN_RxCpltCallback(CAN_RxMessage->Data);
      break;

    case 0x13:
      robot.Chassis.Motor_Steer[2].CAN_RxCpltCallback(CAN_RxMessage->Data);
      break;

    case 0x14:
      robot.Chassis.Motor_Steer[3].CAN_RxCpltCallback(CAN_RxMessage->Data);
      break;

    default:
      break;
  }
}

void Device_CAN2_Callback(Struct_FDCAN_Rx_Buffer* CAN_RxMessage) {
  switch (CAN_RxMessage->Header.Identifier) {
    case 0x201:
      robot.Chassis.ODrive_Motor_Steer[0].CAN_RxCpltCallback(
          CAN_RxMessage->Data);
      break;

    case 0x301:
      robot.Chassis.ODrive_Motor_Steer[1].CAN_RxCpltCallback(
          CAN_RxMessage->Data);
      break;

    case 0x401:
      robot.Chassis.ODrive_Motor_Steer[2].CAN_RxCpltCallback(
          CAN_RxMessage->Data);
      break;

    case 0x501:
      robot.Chassis.ODrive_Motor_Steer[3].CAN_RxCpltCallback(
          CAN_RxMessage->Data);
      break;

      // 位置速度信息
    case 0x209:
      robot.Chassis.ODrive_Motor_Steer[0].CAN_RxCpltCallback(
          CAN_RxMessage->Data);
      break;

    case 0x309:
      robot.Chassis.ODrive_Motor_Steer[1].CAN_RxCpltCallback(
          CAN_RxMessage->Data);
      break;

    case 0x409:
      robot.Chassis.ODrive_Motor_Steer[2].CAN_RxCpltCallback(
          CAN_RxMessage->Data);
      break;

    case 0x509:
      robot.Chassis.ODrive_Motor_Steer[3].CAN_RxCpltCallback(
          CAN_RxMessage->Data);
      break;

    default:
      break;
  }
}

void Device_CAN3_Callback(Struct_FDCAN_Rx_Buffer* CAN_RxMessage) {
  switch (CAN_RxMessage->Header.Identifier) {
    case 0x05:
      robot.Chassis.Encoder_Steer[0].CAN_RxCpltCallback(CAN_RxMessage->Data);
      break;

    case 0x06:
      robot.Chassis.Encoder_Steer[1].CAN_RxCpltCallback(CAN_RxMessage->Data);
      break;

    case 0x07:
      robot.Chassis.Encoder_Steer[2].CAN_RxCpltCallback(CAN_RxMessage->Data);
      break;

    case 0x08:
      robot.Chassis.Encoder_Steer[3].CAN_RxCpltCallback(CAN_RxMessage->Data);
      break;
    default:
      break;
  }
}

void Init_Task(void) {
  // 整车使能
  robot.Init();
  // 使能调度时钟
  TIM_Init(&htim5, Task1ms_TIM5_Callback);
  // 蜂鸣器使能
  Buzzer_Init();

  HAL_Delay(100);

  // CAN总线初始化
  CAN_Init(&hfdcan1, Device_CAN1_Callback);
  CAN_Init(&hfdcan2, Device_CAN2_Callback);
  CAN_Init(&hfdcan3, Device_CAN3_Callback);
  // UART初始化
  // UART_Init(&huart5, DR16_UART5_Callback, 36);
  UART_Init(&huart7, OPS_UART7_Callback, 128);
  // 使能调度时钟
  __HAL_TIM_CLEAR_IT(&htim5, TIM_IT_UPDATE);  // 清除更新中断标志
  HAL_TIM_Base_Start_IT(&htim5);
  init_finished = true;
  // 等待系统
  HAL_Delay(2000);
  // 标记初始化完成
  Buzzer_Play_Startup();

  // Motor.Init(&hfdcan1, 0x11, 0x01,
  // Motor_DM_Control_Method_NORMAL_MIT, 21.92f, 150.0f, 5.0f);
}

// uint8_t bb[] ={0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xfc};
// volatile uint32_t bb = 0;
void Start_Task(void) {
  // robot.BoardComm.Send_Data(bb,8);
  // bb++;
  // HAL_Delay(10);
  // Motor.CAN_Send_Enter();
  // HAL_Delay(2);
  // CAN_Send_Data(&hfdcan1, 0x01 , bb, 8 );
}
