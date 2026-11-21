//
// Created by pzx on 2025/12/13.
//

#ifndef ROBOT_H
#define ROBOT_H
#include "dvc_dr16.h"
#include "crt_chassis.h"
#include "dvc_logF710.h"
#include "drv_minipc.h"
#include "dvc_logF710.h"
#include "dvc_ops.h"
#include "dvc_Chassis_minipc.h"
#include "crt_navigation.h"
#include "dvc_boardcomm.h"

class Class_Robot {
public:

    Class_Chassis Chassis;
    Class_Chassis_MiniPC Dvc_MiniPC;
    Class_Navigation Navigation;

    Class_Drv_MiniPC Drv_MiniPC;
    Class_BoardComm BoardComm;

    uint8_t Usart_data  = 0;

    Class_DR16 DR16;
    Class_LogF710 LogF710;

    Class_OPS ops;

    void Init();

    void Loop();

    void TIM_1000ms_Alive_PeriodElapsedCallback();

    void TIM_100ms_Alive_PeriodElapsedCallback();

    void TIM_100ms_Calculate_Callback();

    void TIM_10ms_Calculate_PeriodElapsedCallback();

    void TIM_2ms_Calculate_PeriodElapsedCallback();

    void TIM_1ms_Calculate_Callback();



protected:

    // 底盘解算方向对底盘速度的前馈
    float AHRS_Chassis_Omega_Feedforward = 0.07f;
    float Chassis_Omega_Feedforward = 0.10f;

    // DR16摇杆死区
    float DR16_Rocker_Dead_Zone = 0.03f;
    // DR16摇杆对云台yaw灵敏度系数(0.002 * PI表示yaw速度最大时为1rpm)
    float DR16_Rocker_Yaw_Resolution = 0.001f * PI;
    // DR16摇杆对云台pitch灵敏度系数(0.002 * PI表示pitch速度最大时为1rpm)
    float DR16_Rocker_Pitch_Resolution = 0.0005f * PI;


    void _Chassis_Control();
    void _Chassis_Navigation_Control();

};


#endif //USTC_STREETING_ROBOT_H