//
// Created by pzx on 2025/12/20.
//

#ifndef USTC_STEERING_1_CRT_CHASSIS_H
#define USTC_STEERING_1_CRT_CHASSIS_H

#include "dvc_encoder.h"
#include "dvc_odrive.h"
#include "dvc_motor_dm.h"

#include "drv_math.h"
#include "arm_math.h"

#include "drv_can.h"
#include "can.h"
/**
 * @brief 底盘控制类型
 *
 */
enum Enum_Chassis_Control_Type {
    Chassis_Control_Type_DISABLE = 0,
    Chassis_Control_Type_ENABLE,
};

/**
 * @brief Specialized, 舵轮底盘类
 *
 */
class Class_Chassis {


public:

    //舵向电机
    Class_Motor_DM_Normal Motor_Steer[4];

    //舵向编码器
    Class_Encoder_BRT Encoder_Steer[4];

    //轮向电机
    Class_ODrive ODrive_Motor_Steer[4];

    //

    void Init();

    bool Is_Steer_Angle_Reached(int index);  // 判断角度是否到达

    inline float Get_Now_Velocity_X();

    inline float Get_Now_Velocity_Y();

    inline float Get_Now_Omega();

    inline float Get_Angle_Pitch();

    inline float Get_Angle_Roll();

    inline Enum_Chassis_Control_Type Get_Chassis_Control_Type();

    inline float Get_Target_Velocity_X();

    inline float Get_Target_Velocity_Y();

    inline float Get_Target_Omega();

    inline void Set_Chassis_Control_Type(Enum_Chassis_Control_Type __Chassis_Control_Type);

    inline void Set_Target_Velocity_X(float __Target_Velocity_X);

    inline void Set_Target_Velocity_Y(float __Target_Velocity_Y);

    inline void Set_Target_Omega(float __Target_Omega);

    void TIM_100ms_Alive_PeriodElapsedCallback();

    void TIM_2ms_Resolution_PeriodElapsedCallback();

    void TIM_2ms_Control_PeriodElapsedCallback();

protected:

    bool steer_angle_reached[4] = {false};  // 记录每个舵向是否到达目标角度
    const float angle_tolerance = 0.05f;     // 角度容差，单位弧度（约2.86度）

//常量

    // 轮组半径
    const float Wheel_Radius = 0.058f;
    // 轮距中心长度
    const float Wheel_To_Core_Distance[4] = {0.207f,
                                             0.207f,
                                             0.207f,
                                             0.207f,};
    // 轮组方位角
    const float Wheel_Azimuth[4] = {PI / 4.0f,
                                    3.0f * PI / 4.0f,
                                    5.0f * PI / 4.0f,
                                    7.0f * PI / 4.0f,};

    // 重心高度
    const float Center_Height = 0.15f;

    // 底盘质量
    const float Class_Mass = 10.0f;

    // 转动惯量
    const float Class_Inertia = 2.0f;


//内部变量


    // 舵向电机角度目标值
    float Target_Steer_Angle[4];
    // 舵向电机角速度目标值
    float Target_Steer_Omage[4];
    // 舵向电机力矩目标值
    float Target_Steer_Torque[4];

    // 轮向电机角速度目标值
    float Target_Wheel_Omega[4];
    // 轮向电机力矩
    float Target_Wheel_torque[4];

    // 轮向电机静摩擦阻力电流值
    float Static_Resistance_Wheel_Current[4] = {0.0f,
                                                0.0f,
                                                0.0f,
                                                0.0f};

    float Dynamic_Resistance_Wheel_Current[4] = {0.0f,
                                                 0.0f,
                                                 0.0f,
                                                 0.0f};
    // 轮向电机摩擦阻力连续化的角速度阈值
    float Wheel_Resistance_Omega_Threshold = 1.0f;
    // 防单轮超速系数
    float Wheel_Speed_Limit_Factor = 0.5f;


// 读变量


    // 当前速度X
    float Now_Velocity_X = 0.0f;
    // 当前速度Y
    float Now_Velocity_Y = 0.0f;
    // 当前角速度
    float Now_Omega = 0.0f;

    // 底盘相对Odom角度
    float Angle_Pitch = 0.0f;
    float Angle_Roll = 0.0f;

// 写变量

    Enum_Chassis_Control_Type Chassis_Control_Type = Chassis_Control_Type_DISABLE;

    // 目标速度X
    float Target_Velocity_X = 0.0f;
    // 目标速度Y
    float Target_Velocity_Y = 0.0f;
    // 目标角速度
    float Target_Omega = 0.0f;

    //自身姿态，速度解算
    void Self_Resolution();
    //舵轮运动学逆结算
    void Kinematics_Inverse_Resolution();
    //舵向电机就近解算
    void _Steer_Motor_Kinematics_Nearest_Transposition();
    //输出动力学
    void Output_To_Dynamics();
    //动力学逆解算
    void Dynamics_Inverse_Resolution();
    //输出到电机
    void Output_To_Motor();

};

/**
 * @brief 获取当前速度X
 *
 * @return float 当前速度X
 */
inline float Class_Chassis::Get_Now_Velocity_X()
{
    return (Now_Velocity_X);
}

/**
 * @brief 获取当前速度Y
 *
 * @return float 当前速度Y
 */
inline float Class_Chassis::Get_Now_Velocity_Y()
{
    return (Now_Velocity_Y);
}

/**
 * @brief 获取当前角速度
 *
 * @return float 当前角速度
 */
inline float Class_Chassis::Get_Now_Omega()
{
    return (Now_Omega);
}
/**
 * @brief 获取底盘相对Odom角度
 *
 * @return float 底盘相对Odom角度
 */
inline float Class_Chassis::Get_Angle_Pitch()
{
    return (Angle_Pitch);
}

/**
 * @brief 获取底盘相对Odom角度
 *
 * @return float 底盘相对Odom角度
 */
inline float Class_Chassis::Get_Angle_Roll()
{
    return (Angle_Roll);
}
/**
 * @brief 获取底盘控制方法
 *
 * @return Enum_Chassis_Control_Type 底盘控制方法
 */
inline Enum_Chassis_Control_Type Class_Chassis::Get_Chassis_Control_Type()
{
    return (Chassis_Control_Type);
}
/**
 * @brief 获取目标速度X
 *
 * @return float 目标速度X
 */
inline float Class_Chassis::Get_Target_Velocity_X()
{
    return (Target_Velocity_X);
}

/**
 * @brief 获取目标速度Y
 *
 * @return float 目标速度Y
 */
inline float Class_Chassis::Get_Target_Velocity_Y()
{
    return (Target_Velocity_Y);
}

/**
 * @brief 获取目标速度方向
 *
 * @return float 目标速度方向
 */
inline float Class_Chassis::Get_Target_Omega()
{
    return (Target_Omega);
}

/**
 * @brief 设定底盘控制方法
 *
 * @param __Chassis_Control_Type 底盘控制方法
 */
inline void Class_Chassis::Set_Chassis_Control_Type(Enum_Chassis_Control_Type __Chassis_Control_Type)
{
    Chassis_Control_Type = __Chassis_Control_Type;
}
/**
 * @brief 设定目标速度X
 *
 * @param __Target_Velocity_X 目标速度X
 */
inline void Class_Chassis::Set_Target_Velocity_X(float __Target_Velocity_X)
{
    Target_Velocity_X = __Target_Velocity_X;
}

/**
 * @brief 设定目标速度Y
 *
 * @param __Target_Velocity_Y 目标速度Y
 */
inline void Class_Chassis::Set_Target_Velocity_Y(float __Target_Velocity_Y)
{
    Target_Velocity_Y = __Target_Velocity_Y;
}

/**
 * @brief 设定目标角速度
 *
 * @param __Target_Omega 目标角速度
 */
inline void Class_Chassis::Set_Target_Omega(float __Target_Omega)
{
    Target_Omega = __Target_Omega;
}


#endif //USTC_STEERING_1_CRT_CHASSIS_H