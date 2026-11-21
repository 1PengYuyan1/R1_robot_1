//
// Created by pzx on 2025/12/16.
//
#include "task.h"
#include "main.h"
#include "can.h"

#include "dvc_motor_dm.h"
#include "dvc_encoder.h"
#include "dvc_odrive.h"
#include "drv_can.h"
#include "drv_tim.h"
#include "dvc_dr16.h"
#include "math.h"
#include "arm_math.h"
#include "robot.h"

Class_Robot robot;

Class_Motor_DM_Normal motor1;
Class_Encoder_BRT encoder1;
Class_ODrive odrive1;
Class_DR16 DR16;

uint8_t b[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
int a;


bool init_finished = false;


float m;
/**
 * @brief TIM4任务回调函数
 *
 */
void Task100us_TIM4_Callback()
{

}

/**
 * @brief TIM5任务回调函数
 *
 */
void Task1ms_TIM5_Callback()
{

    // TIM_1ms_CAN_PeriodElapsedCallback();

}
void Device_CAN1_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.StdId)
    {
        // case (0x201):
        // {
        //     odrive1.CAN_RxCpltCallback(CAN_RxMessage->Data);
        //     break;
        // }
            case (0x02):
            {
            encoder1.CAN_RxCpltCallback(CAN_RxMessage->Data);
                break;
            }
        case (0x11):
        {
            motor1.CAN_RxCpltCallback(CAN_RxMessage->Data);
            break;
        }
        case (0x209):
        {
            odrive1.CAN_RxCpltCallback(CAN_RxMessage->Data);

            break;
        }
            // case (0x205):
            // {
            //     robot.Gimbal.Motor_Yaw.CAN_RxCpltCallback(CAN_RxMessage->Data);
            //
            //     break;
            // }
            // case (0x206):
            // {
            //     robot.Gimbal.Motor_Pitch.CAN_RxCpltCallback(CAN_RxMessage->Data);
            //
            //     break;
            // }
    }
}
void Init_Task(void) {

    // CAN总线初始化
    CAN_Init(&hcan1,Device_CAN1_Callback);

    // 使能调度时钟
    // TIM_Init(&htim4, Task100us_TIM4_Callback);
    // TIM_Init(&htim5, Task1ms_TIM5_Callback);

    //
    // robot.Init();

    //
    DR16.Init(&huart1);

    // 标记初始化完成

    //
       motor1.Init(&hcan1,
                0x11,     // 接收ID（与上位机Master_ID一致）
                0x01,     // 发送基础ID（上位机CAN_ID）
                Motor_DM_Control_Method_NORMAL_MIT,  // MIT模式
                12.5f,    // 最大角度（rad）
                100.0f,   // 最大速度（度/秒）
                10.0f,     // 最大扭矩（Nm）
                10.0f);
    HAL_Delay(50);

    //
    encoder1.Init(&hcan1, 0x02, 1000000, 4096);
    HAL_Delay(50);

    //
    odrive1.Init(&hcan1, 0x010, 1000000);
    HAL_Delay(50);

    // encoder1.CAN_Send_SetZero();
    init_finished = true;

}


void Start_Task(void) {
    // motor1.Set_Control_Angle(0.0f);    // 目标角度
    // motor1.Set_Control_Omega(0.0f);    // 目标角速度
    // motor1.Set_Control_Torque(0.0f);   // 目标扭矩
    // motor1.Set_K_P(0.0f);           // 比例系数
    // motor1.Set_K_D(1.0f);
    // motor1.TIM_Send_PeriodElapsedCallback();

    // CAN_Send_Data_RTR(&hcan1,0x209,b,8);
    // CAN_Send_Data(&hcan1,0x02,b,8);
    a++;
     // HAL_Delay(100);

    motor1.TIM_Send_PeriodElapsedCallback();
    HAL_Delay(100);

    encoder1.TIM_Query_PeriodElapsedCallback();
    HAL_Delay(100);

    odrive1.Set_Velocity(m, 0);
    HAL_Delay(50);
    odrive1.Request_Encoder_Data();
    HAL_Delay(50);

}