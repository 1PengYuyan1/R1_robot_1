//
// Created by pzx on 2025/12/13.
//
#include "robot.h"
#include "stm32f4xx.h"

/**
 * @brief 控制交互端初始化
 *
 */
void Class_Robot::Init() {
    Chassis.Init();

    DR16.Init(&huart1);
}

/**
 * @brief 定时器计算函数
 *
 */
void Class_Robot::TIM_100ms_Alive_PeriodElapsedCallback() {
    DR16.TIM_100ms_Alive_PeriodElapsedCallback();

    Chassis.TIM_100ms_Alive_PeriodElapsedCallback();
}

/**
 * @brief 定时器计算函数
 *
 */
void Class_Robot::TIM_2ms_Calculate_PeriodElapsedCallback() {
    Chassis.TIM_2ms_Resolution_PeriodElapsedCallback();
    Chassis.TIM_2ms_Control_PeriodElapsedCallback();
}

/**
 * @brief 底盘控制逻辑
 *
 */
void Class_Robot::_Chassis_Control() {
    // 判断遥控器状态是否正常, 不在线或急停直接断控
    if (DR16.Get_Status() == DR16_Status_DISABLE || DR16.Get_Left_Switch() == DR16_Switch_Status_DOWN) {
        Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
        return;
    } else {
        // 底盘速度期望值
        float tmp_expect_direction_velocity_x, tmp_expect_direction_velocity_y, tmp_expect_direction_omega;
        // 底盘速度传参值
        float tmp_chassis_velocity_x, tmp_chassis_velocity_y, tmp_chassis_omega;

        // 遥控器摇杆值
        float dr16_left_x = DR16.Get_Left_X();
        float dr16_left_y = DR16.Get_Left_Y();
        float dr16_yaw = DR16.Get_Yaw();
        // 排除死区
        dr16_left_x = Math_Abs(dr16_left_x) > DR16_Rocker_Dead_Zone ? dr16_left_x : 0.0f;
        dr16_left_y = Math_Abs(dr16_left_y) > DR16_Rocker_Dead_Zone ? dr16_left_y : 0.0f;
        dr16_yaw = Math_Abs(dr16_yaw) > DR16_Rocker_Dead_Zone ? dr16_yaw : 0.0f;

        // 摇杆控制

        tmp_expect_direction_velocity_x = dr16_left_y;
        tmp_expect_direction_velocity_y = -dr16_left_x;
        tmp_expect_direction_omega = dr16_yaw;

        Chassis.Set_Target_Velocity_X(tmp_chassis_velocity_x);
        Chassis.Set_Target_Velocity_Y(tmp_chassis_velocity_y);
        Chassis.Set_Target_Omega(tmp_chassis_omega);
    }
}
