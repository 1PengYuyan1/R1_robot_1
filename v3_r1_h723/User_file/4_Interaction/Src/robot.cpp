//
// Created by pzx on 2025/12/13.
//
#include "robot.h"

#include "stm32h7xx_hal.h"

// const Struct_Waypoint Route_Get_Ball[] = {
//   // x,     y,    yaw, speed, radius
//   {100.0f, -100.0f, 0.0f, 2.5f, 100.0f},
//   {200.0f, -200.0f, 0.0f, 2.5f, 100.0f},
//   {300.0f, -300.0f, 0.0f, 2.5f, 100.0f},
//
//   {400.0f, -310.0f, 0.0f, 2.3f, 100.0f},
//   {500.0f, -310.0f, 0.0f, 2.5f, 100.0f},
//   {600.0f, -310.0f, 0.0f, 2.5f, 100.0f},
//   {700.0f, -310.0f, 0.0f, 2.5f, 100.0f},
//   {800.0f, -310.0f, 0.0f, 2.5f, 100.0f},
//   {900.0f, -310.0f, 0.0f, 2.5f, 100.0f},
//   {1000.0f, -310.0f, 0.0f, 2.5f, 100.0f},
//   {1100.0f, -310.0f, 0.0f, 2.5f, 100.0f},
//   {1200.0f, -310.0f, 0.0f, 2.5f, 100.0f},
//   {1300.0f, -310.0f, 0.0f, 2.5f, 100.0f},
//   {1400.0f, -310.0f, 0.0f, 2.5f, 100.0f},
//   {1500.0f, -310.0f, 0.0f, 2.5f, 100.0f},
//
//   {1600.0f, -310.0f, 0.0f, 2.5f, 100.0f},
//   {1700.0f, -310.0f, 0.0f, 2.5f, 100.0f},
//   {1800.0f, -310.0f, 0.0f, 2.5f, 100.0f},
//   {1900.0f, -310.0f, 0.0f, 2.5f, 100.0f},
//   {2000.0f, -310.0f, 0.0f, 2.5f, 50.0f},
//   {2075.0f, -310.0f, 0.0f, 2.0f, 50.0f}
//
// };

const Struct_Waypoint Route_Get_Ball[] = {
  // x,     y,    yaw, speed, radius
  {100.0f, -100.0f, 0.0f, 2.5f, 100.0f},
  {200.0f, -200.0f, 0.0f, 2.5f, 100.0f},
  {300.0f, -300.0f, 0.0f, 2.5f, 100.0f},

  {400.0f, -300.0f, 0.0f, 2.3f, 100.0f},

  {700.0f, -300.0f, 0.0f, 2.5f, 100.0f},
  {1000.0f, -300.0f, 0.0f, 2.5f, 100.0f},
  {1300.0f, -300.0f, 0.0f, 2.5f, 100.0f},
  {1600.0f, -300.0f, 0.0f, 2.5f, 100.0f},
  {1900.0f, -300.0f, 0.0f, 2.5f, 50.0f},
  {2075.0f, -300.0f, 0.0f, 2.0f, 50.0f}

};
// const Struct_Waypoint Route_Get_Ball[] = {
//     // x,    y,    yaw, speed, radius
//     {-100.0f, -150.0f, 0.0f, 1.5f, 50.0f},
//     {-200.0f, -300.0f, 0.0f, 2.0f, 50.0f},
//     {-300.0f, -500.0f, 0.0f, 2.5f, 50.0f},
//     {-390.0f, -750.0f, 0.0f, 2.5f, 100.0f},
//     {-390.0f, -1050.0f, 0.0f, 2.5f, 100.0f},
//     {-390.0f, -1400.0f, 0.0f, 2.0f, 100.0f},
//     {-390.0f, -1800.0f, 0.0f, 1.5f, 100.0f},
//     {-390.0f, -2000.0f, 0.0f, 1.0f, 100.0f}
// };

/**
 * @brief 控制交互端初始化
 *
 */
void Class_Robot::Init()
{
  Usart_data = 0;

  Chassis.Init();
  Chassis.Steer_Calibration_Init();
  Chassis.Init_Motor_Params();

  // DR16.Init(&huart5);
  LogF710.Init();
  BoardComm.Init(&huart10);

  ops.init();
  Navigation.Init();
}

/**
 * @brief 定时器执行器存活
 *
 */
void Class_Robot::TIM_100ms_Alive_PeriodElapsedCallback()
{
  // DR16.TIM_100ms_Alive_PeriodElapsedCallback();
  LogF710.TIM_100ms_Alive_PeriodElapsedCallback();
  Chassis.TIM_100ms_Alive_PeriodElapsedCallback();
}

/**
 * @brief 定时器遥控控制
 *
 */
void Class_Robot::TIM_2ms_Calculate_PeriodElapsedCallback()
{
  Chassis.TIM_2ms_Resolution_PeriodElapsedCallback();

  Chassis.TIM_2ms_Control_PeriodElapsedCallback();
}

void Class_Robot::TIM_1ms_Calculate_Callback()
{
  // Dvc_MiniPC.Parse_Data();
  // DR16.TIM_1ms_Calculate_PeriodElapsedCallback();
  LogF710.TIM_1ms_Calculate_PeriodElapsedCallback();

  ops.getData();

  if (Navigation.Get_Status() == Nav_Status_NAVIGATING)
  {
    Navigation.Calculate(ops.getData());
  }

  _Chassis_Control();
}

/**
 * @brief 底盘控制逻辑
 *
 */
float a, b, c;
uint16_t d;

void Class_Robot::_Chassis_Control()
{
  // 统一声明目标速度
  float target_vx = 0.0f;
  float target_vy = 0.0f;
  float target_omega = 0.0f;

  {
    // float current_yaw = ops.getData().Yaw;

    // if (!Chassis.Is_Calibration_Complete()) {
    //     Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
    //     return;
    // }

    // // 优先级 1: DR16 遥控器 (半自动导航 + 纯手动控制)
    // if (DR16.Is_Connected())
    // {
    //     if (DR16.Get_Status() == DR16_Status_DISABLE ||
    //     DR16.Get_Left_Switch() == DR16_Switch_Status_DOWN)
    //     {
    //         Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
    //
    //         if (Navigation.Get_Status() != Nav_Status_IDLE)
    //         {
    //             Navigation.Stop_Navigation();
    //         }
    //         return;
    //     }
    //
    //     // DR16 状态正常，使能底盘
    //     Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_ENABLE);
    //
    //     Enum_DR16_Switch_Status left_sw = DR16.Get_Left_Switch();
    //     Enum_DR16_Switch_Status right_sw = DR16.Get_Right_Switch();
    //
    //     // 停止导航，回到手动模式 (左 MID + 右 MID)
    //     if (left_sw == DR16_Switch_Status_MIDDLE && right_sw ==
    //     DR16_Switch_Status_MIDDLE)
    //     {
    //         if (Navigation.Get_Status() != Nav_Status_IDLE)
    //         {
    //             Navigation.Stop_Navigation();
    //         }
    //     }
    //     // // 开启“自动模式准备状态” (左 MID + 右 DOWN)
    //     // else if (left_sw == DR16_Switch_Status_MIDDLE && right_sw ==
    //     DR16_Switch_Status_DOWN)
    //     // {
    //     //     // 如果你的 Navigation 类有“预备”接口可以在此调用
    //     //     // 这里主要起到占位作用，确保从 MID/MID 切换过来时不会立刻跑车
    //     // }
    //     // 触发具体导航路径 (左 UP + 右 DOWN)
    //     else if (left_sw == DR16_Switch_Status_UP && right_sw ==
    //     DR16_Switch_Status_DOWN)
    //     {
    //         // 只有当前处于空闲时才触发，防止重复触发导致路径重置
    //         if (Navigation.Get_Status() == Nav_Status_IDLE)
    //         {
    //             // 这里触发你指定的路径，例如 Route_Get_Ball
    //             Navigation.Set_Route(Route_Get_Ball, sizeof(Route_Get_Ball) /
    //             sizeof(Struct_Waypoint)); Navigation.Start_Navigation();
    //         }
    //     }
    //
    //     // 导航接管底盘 (拦截)
    //     if (Navigation.Get_Status() == Nav_Status_NAVIGATING)
    //     {
    //         target_vx = Navigation.Get_Target_Vx();
    //         target_vy = Navigation.Get_Target_Vy();
    //         target_omega = Navigation.Get_Target_Omega();
    //
    //         Chassis.Set_Target_Velocity_X(target_vx);
    //         Chassis.Set_Target_Velocity_Y(target_vy);
    //         Chassis.Set_Target_Omega(target_omega);
    //
    //         return; // 导航中，跳过下方摇杆读取
    //     }
    //
    //     // 纯手动接管逻辑 (当导航处于 IDLE 状态时)
    //     float dr16_left_x = DR16.Get_Left_X();
    //     float dr16_left_y = DR16.Get_Left_Y();
    //     float dr16_yaw = DR16.Get_Yaw();
    //
    //     // 排除死区
    //     dr16_left_x = Math_Abs(dr16_left_x) > DR16_Rocker_Dead_Zone ?
    //     dr16_left_x : 0.0f; dr16_left_y = Math_Abs(dr16_left_y) >
    //     DR16_Rocker_Dead_Zone ? dr16_left_y : 0.0f; dr16_yaw =
    //     Math_Abs(dr16_yaw) > DR16_Rocker_Dead_Zone ? dr16_yaw : 0.0f;
    //
    //     target_vx = dr16_left_y;
    //     target_vy = -dr16_left_x;
    //     target_omega = dr16_yaw;
    //
    //     // Chassis.Apply_Heading_Correction(target_omega, target_vx,
    //     target_vy, current_yaw);
    //
    //     Chassis.Set_Target_Velocity_X(target_vx);
    //     Chassis.Set_Target_Velocity_Y(target_vy);
    //     Chassis.Set_Target_Omega(target_omega);
    //
    //     return;
    // }
    //
    // Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
    //
    // if (Navigation.Get_Status() != Nav_Status_IDLE)
    // {
    //     Navigation.Stop_Navigation();
    // }

    // // 优先级 2: ROS 上位机控制
    // else if (Dvc_MiniPC.Is_Connected())
    // {
    //     Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_ENABLE);
    //
    //     target_vx = Dvc_MiniPC.Get_Vx();
    //     target_vy = Dvc_MiniPC.Get_Vy();
    //     target_omega = Dvc_MiniPC.Get_Vw();
    //
    //     Chassis.Set_Target_Velocity_X(target_vx);
    //     Chassis.Set_Target_Velocity_Y(target_vy);
    //     Chassis.Set_Target_Omega(target_omega);
    //
    //     return;
    // }

    // 优先级 3: LogF710 无线手柄控制
    // a =  LogF710.Get_Left_X();
    // b =  LogF710.Get_Left_Y();
    // c =  LogF710.Get_Right_X();
    // d =  LogF710.Get_Button_Status();
  }

  if (LogF710.Is_Connected())
  {
    LogF710.Judge_Status();

    if (LogF710.Is_ROS_Control_Enabled &&
      LogF710.Get_Status() != LogF710_Status_DISABLE)
    {
      Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_ENABLE);

      Usart_data = 1;
      BoardComm.Send_Data(Usart_data);

      target_vx = LogF710.Get_Left_X();
      target_vy = LogF710.Get_Left_Y();
      target_omega = LogF710.Get_Right_X();

      Chassis.Set_Target_Velocity_X(target_vx);
      Chassis.Set_Target_Velocity_Y(target_vy);
      Chassis.Set_Target_Omega(target_omega);

      // 停止导航，回到手动模式 (左 MID + 右 MID)
      if (LogF710.Get_Key_Status() == LogF710_Key_Status_X)
      {
        if (Navigation.Get_Status() != Nav_Status_IDLE)
        {
          Navigation.Stop_Navigation();
        }
      }

      if (LogF710.Get_Key_Status() == LogF710_Key_Status_Y)
      {
        if (Navigation.Get_Status() == Nav_Status_IDLE)
        {
          // 这里触发你指定的路径，例如 Route_Get_Ball
          Navigation.Set_Route(
            Route_Get_Ball, sizeof(Route_Get_Ball) / sizeof(Struct_Waypoint));
          Navigation.Start_Navigation(ops.getData().Pos_X, ops.getData().Pos_Y);
        }
      }

      if (LogF710.Get_Key_Status() == LogF710_Key_Status_A)
      {
        Usart_data = 2;
        BoardComm.Send_Data(Usart_data);
        Usart_data = 0;
      }

      if (LogF710.Get_Key_Status() == LogF710_Key_Status_LB)
      {
        Usart_data = 3;
        BoardComm.Send_Data(Usart_data);
        Usart_data = 0;
      }

      if (LogF710.Get_Key_Status() == LogF710_Key_Status_RB)
      {
        Usart_data = 4;
        BoardComm.Send_Data(Usart_data);
        Usart_data = 0;
      }

      // 导航接管底盘 (拦截)
      if (Navigation.Get_Status() == Nav_Status_NAVIGATING)
      {
        target_vx = Navigation.Get_Target_Vx();
        target_vy = Navigation.Get_Target_Vy();
        target_omega = Navigation.Get_Target_Omega();

        Chassis.Set_Target_Velocity_X(target_vx);
        Chassis.Set_Target_Velocity_Y(target_vy);
        Chassis.Set_Target_Omega(target_omega);

        return;
      }
    }
    else
    {
      Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
      Usart_data = 5;
      BoardComm.Send_Data(Usart_data);
    }

    return;
  }
}
