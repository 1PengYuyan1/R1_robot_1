//
// Created by pzx on 2025/12/20.
//
#include "crt_chassis.h"

void Class_Chassis::Init() {
    //舵向初始化

    //舵向电机初始化
    Motor_Steer[0].Init(&hcan1, 0x11, 0x01, Motor_DM_Control_Method_NORMAL_MIT,
                        360.0f, 100.0f, 10.0f);

    Motor_Steer[1].Init(&hcan1, 0x12, 0x02, Motor_DM_Control_Method_NORMAL_MIT,
                        360.0f, 100.0f, 10.0f);

    Motor_Steer[2].Init(&hcan1, 0x13, 0x03, Motor_DM_Control_Method_NORMAL_MIT,
                        360.0f, 100.0f, 10.0f);

    Motor_Steer[3].Init(&hcan1, 0x14, 0x04, Motor_DM_Control_Method_NORMAL_MIT,
                        360.0f, 100.0f, 10.0f);

    //舵向编码器初始化
    Encoder_Steer[0].Init(&hcan1, 0x01, BRT_BAUDRATE_100K, 16);
    Encoder_Steer[1].Init(&hcan1, 0x01, BRT_BAUDRATE_100K, 16);
    Encoder_Steer[2].Init(&hcan1, 0x01, BRT_BAUDRATE_100K, 16);
    Encoder_Steer[3].Init(&hcan1, 0x01, BRT_BAUDRATE_100K, 16);

    //轮向电机

    //轮向电机初始化
    ODrive_Motor_Steer[0].Init(&hcan1, 0x010,ODRIVE_DEFAULT_BAUDRATE);
    ODrive_Motor_Steer[1].Init(&hcan1, 0x010,ODRIVE_DEFAULT_BAUDRATE);
    ODrive_Motor_Steer[2].Init(&hcan1, 0x010,ODRIVE_DEFAULT_BAUDRATE);
    ODrive_Motor_Steer[3].Init(&hcan1, 0x010,ODRIVE_DEFAULT_BAUDRATE);
}

/**
 * @brief 判断角度是否到达
 *
 */
bool Class_Chassis::Is_Steer_Angle_Reached(int index) {
    float current_angle = Encoder_Steer[index].Get_Wheel_Posture();
    float target_angle = Target_Steer_Angle[index];
    float error = fabsf(Math_Modulus_Normalization(current_angle - target_angle, 2.0f * PI));

    // 如果误差小于容差，认为到达目标角度
    return (error < angle_tolerance || error > (2.0f * PI - angle_tolerance));
}

/**
 * @brief TIM定时器中断定期检测电机是否存活
 *
 */
void Class_Chassis::TIM_100ms_Alive_PeriodElapsedCallback() {
    for (int i = 0; i < 4; i++) {
        Motor_Steer[i].TIM_Alive_PeriodElapsedCallback();
        Encoder_Steer[i].TIM_Alive_PeriodElapsedCallback();
        ODrive_Motor_Steer[i].TIM_Alive_CheckCallback();
    }
}

/**
 * @brief TIM定时器中断自身姿态，速度解算回调函数
 *
 */
void Class_Chassis::TIM_2ms_Resolution_PeriodElapsedCallback() {
    Self_Resolution();
}

/**
 * @brief TIM定时器中断底盘控制回调函数
 *
 */
void Class_Chassis::TIM_2ms_Control_PeriodElapsedCallback() {
    Kinematics_Inverse_Resolution();

    Output_To_Dynamics();

    Dynamics_Inverse_Resolution();

    Output_To_Motor();
}

/**
 * @brief 自身姿态，速度解算
 *
 */
void Class_Chassis::Self_Resolution() {
    //计算整车速度

    Now_Omega = 0.0f;
    Now_Velocity_X = 0.0f;
    Now_Velocity_Y = 0.0f;

    for (int i = 0; i < 4; i++) {
        Now_Velocity_X += (ODrive_Motor_Steer[i].Get_Omega() * arm_cos_f32(
                               Encoder_Steer[i].Get_Wheel_Posture() * Wheel_Radius) / 4.0f);
        Now_Velocity_Y += (ODrive_Motor_Steer[i].Get_Omega() * arm_sin_f32(
                               Encoder_Steer[i].Get_Wheel_Posture() * Wheel_Radius) / 4.0f);
        Now_Omega += (ODrive_Motor_Steer[i].Get_Omega() * arm_sin_f32(
                          Encoder_Steer[i].Get_Wheel_Posture() - Wheel_Azimuth[i])
                      * Wheel_Radius / Wheel_To_Core_Distance[i]) / 4.0f;
    }
}

/**
 * @brief 运动学逆解算
 *
 */
void Class_Chassis::Kinematics_Inverse_Resolution() {
    for (int i = 0; i < 4; i++) {
        float tmp_velocity_x, tmp_velocity_y, tmp_velocity_modulus;

        //解算到每个轮组的具体线速度
        tmp_velocity_x = Target_Velocity_X - Target_Omega * Wheel_To_Core_Distance[i] / arm_sin_f32(Wheel_Azimuth[i]);
        tmp_velocity_y = Target_Velocity_Y + Target_Omega * Wheel_To_Core_Distance[i] / arm_cos_f32(Wheel_Azimuth[i]);
        arm_sqrt_f32(tmp_velocity_x * tmp_velocity_x + tmp_velocity_y * tmp_velocity_y, &tmp_velocity_modulus);

        //根据以上解算解算出每个轮组的角速度
        Target_Wheel_Omega[i] = tmp_velocity_modulus / Wheel_Radius;

        //根据速度的X，Y分量分别决定轮向电机的角度

        if (tmp_velocity_modulus == 0.0f) {
            Target_Steer_Angle[i] = Encoder_Steer[i].Get_Wheel_Posture();
        } else {
            Target_Steer_Angle[i] = atan2f(tmp_velocity_y, tmp_velocity_x);
        }
    }
    _Steer_Motor_Kinematics_Nearest_Transposition();
}

/**
 * @brief 舵向电机依照轮向电机目标角速度就近转位解算
 *
 */
void Class_Chassis::_Steer_Motor_Kinematics_Nearest_Transposition() {
    for (int i = 0; i < 4; i++) {
        float tmp_delta_angle = Math_Modulus_Normalization(Target_Steer_Angle[i] - Encoder_Steer[i].Get_Wheel_Posture(),
                                                           2.0f * PI);

        //根据转动角范围判断是否需要就近转位
        if (-PI / 2.0f <= tmp_delta_angle && tmp_delta_angle <= PI / 2.0f) {
            Target_Steer_Angle[i] = Encoder_Steer[i].Get_Wheel_Posture() + tmp_delta_angle;
        } else {
            //需要反转扣圈情况
            Target_Steer_Angle[i] = Math_Modulus_Normalization(tmp_delta_angle, 2.0f * PI) + Encoder_Steer[i].
                                    Get_Wheel_Posture();
            Target_Steer_Angle[i] *= -1.0f;
        }
    }
}

/**
 * @brief 动力学解算
 * @prif 输出转换到动力学状态
 */
void Class_Chassis::Output_To_Dynamics() {
    switch (Chassis_Control_Type) {
        case (Chassis_Control_Type_DISABLE): {
            // 底盘失能
            for (int i = 0; i < 4; i++) {
                Motor_Steer[i].Set_Control_Status(Motor_DM_Status_DISABLE);
                ODrive_Motor_Steer[i].Set_Axis_State(ODRIVE_STATE_IDLE);
            }
            // PID_Velocity_X.Set_Integral_Error(0.0f);
            // PID_Velocity_Y.Set_Integral_Error(0.0f);
            // PID_Omega.Set_Integral_Error(0.0f);

            // 目标速度X
            Target_Velocity_X = 0.0f;
            // 目标速度Y
            Target_Velocity_Y = 0.0f;
            // 目标角速度
            Target_Omega = 0.0f;
        }

        break;

        case (Chassis_Control_Type_ENABLE): {
            for (int i = 0; i < 4; i++) {
                Motor_Steer[i].Set_Control_Method(Motor_DM_Control_Method_NORMAL_MIT);
                ODrive_Motor_Steer[i].Set_Axis_State(ODRIVE_STATE_CLOSED_LOOP_CONTROL);
            }

            // PID_Velocity_X.Set_Target(Target_Velocity_X);
            // PID_Velocity_X.Set_Now(Now_Velocity_X);
            // PID_Velocity_X.TIM_Calculate_PeriodElapsedCallback();
            //
            // PID_Velocity_Y.Set_Target(Target_Velocity_Y);
            // PID_Velocity_Y.Set_Now(Now_Velocity_Y);
            // PID_Velocity_Y.TIM_Calculate_PeriodElapsedCallback();
            //
            // PID_Omega.Set_Target(Target_Omega);
            // PID_Omega.Set_Now(Now_Omega);
            // PID_Omega.TIM_Calculate_PeriodElapsedCallback();


            break;
        }
    }
}

/**
 * @brief 动力学逆解算
 *
 */
void Class_Chassis::Dynamics_Inverse_Resolution() {
    float force_x, force_y, torque_omega;

    force_x = Target_Velocity_X * Class_Mass;     // F = m * a，近似为 m * v
    force_y = Target_Velocity_Y * Class_Mass;
    torque_omega = Target_Omega * Class_Inertia;  // τ = I * α

    // 每个轮的扭力
    float tmp_force[4];

    for (int i = 0; i < 4; i++) {
        float steer_angle = Encoder_Steer[i].Get_Wheel_Posture();  // 使用编码器角度

        // 修正力分解公式

       //     tmp_force[i] = Fx·cosθ + Fy·sinθ +     // 平移力在轮子方向的分量
                           // (τ/L)·sin(φ-θ)         // 旋转力在轮子方向的分量

        tmp_force[i] = force_x * arm_cos_f32(steer_angle) +
                       force_y * arm_sin_f32(steer_angle) +
                       (torque_omega / Wheel_To_Core_Distance[i]) *
                       arm_sin_f32(Wheel_Azimuth[i] - steer_angle);
    }

    for (int i = 0; i < 4; i++) {
        // 获取轮子实际角速度（不是角度！）
        float current_wheel_omega = ODrive_Motor_Steer[i].Get_Omega();

        // 摩擦力转换为轮子扭矩 + 速度前馈

    //    Target_Wheel_Torque = tmp_force[i] * Wheel_Radius +          // 静力学部分：克服外力所需的扭矩
    //                          Kp * (Target_Omega - Current_Omega)    // 动力学部分：速度跟踪的前馈补偿
        Target_Wheel_torque[i] = tmp_force[i] * Wheel_Radius +
                                Wheel_Speed_Limit_Factor * (Target_Wheel_Omega[i] - current_wheel_omega);

        // 动摩擦阻力前馈修正
        if (fabsf(Target_Wheel_Omega[i]) > Wheel_Resistance_Omega_Threshold) {

            float sign = (Target_Wheel_Omega[i] > 0) ? 1.0f : -1.0f;

            Target_Wheel_torque[i] += sign * Dynamic_Resistance_Wheel_Current[i];

        } else {

            float ratio = fabsf(Target_Wheel_Omega[i]) / Wheel_Resistance_Omega_Threshold;

            Target_Wheel_torque[i] += ratio * Dynamic_Resistance_Wheel_Current[i] *
                                     (Target_Wheel_Omega[i] > 0 ? 1.0f : -1.0f);

        }
    }

}

/**
 * @brief 输出到电机
 *
 */
void Class_Chassis::Output_To_Motor() {
    switch (Chassis_Control_Type) {
        case (Chassis_Control_Type_DISABLE): {
            // 底盘失能
            for (int i = 0; i < 4; i++) {
                Motor_Steer[i].Set_Control_Status(Motor_DM_Status_DISABLE);
                ODrive_Motor_Steer[i].Set_Axis_State(ODRIVE_STATE_IDLE);
            }

            for (int i = 0; i < 4; i++) {
                Motor_Steer[i].Set_Control_Angle(0.0f);
                Motor_Steer[i].Set_Control_Omega(0.0f);
                Motor_Steer[i].Set_Control_Torque(0.0f);

                ODrive_Motor_Steer[i].Set_Velocity(0.0f);
                ODrive_Motor_Steer[i].Set_Torque(0.0f);
            }

            break;
        }
        case (Chassis_Control_Type_ENABLE): {
            // 舵轮模型

            // 设置执行器控制模式
            for (int i = 0; i < 4; i++) {
                Motor_Steer[i].Set_Control_Method(Motor_DM_Control_Method_NORMAL_MIT);
                ODrive_Motor_Steer[i].Set_Axis_State(ODRIVE_STATE_CLOSED_LOOP_CONTROL);
            }

            // 传入目标值
            for (int i = 0; i < 4; i++) {
                // 检查角度是否到达
                steer_angle_reached[i] = Is_Steer_Angle_Reached(i);

                if (!steer_angle_reached[i]) {
                    // 角度未到达：正常控制舵向电机
                    Motor_Steer[i].Set_Control_Angle(Target_Steer_Angle[i]);
                    Motor_Steer[i].Set_Control_Omega(2.0f); // 较小的转向速度
                    Motor_Steer[i].Set_Control_Torque(1.0f); // 适当的扭矩

                    // 轮向电机低速或停止，等待转向完成
                    ODrive_Motor_Steer[i].Set_Velocity(0.1f, 0.1f);
                } else {
                    // 角度已到达：停止舵向电机，启动轮向电机
                    Motor_Steer[i].Set_Control_Omega(0.0f); // 停止转动
                    Motor_Steer[i].Set_Control_Torque(0.5f); // 保持位置的扭矩

                    // 轮向电机正常运转
                    ODrive_Motor_Steer[i].Set_Velocity(Target_Wheel_Omega[i], Target_Wheel_torque[i]);
                }
            }

            break;
        }
    }

    // 中断回调通讯函数
    for (int i = 0; i < 4; i++) {
        Encoder_Steer[i].TIM_Query_PeriodElapsedCallback();
        Motor_Steer[i].TIM_Send_PeriodElapsedCallback();
        ODrive_Motor_Steer[i].TIM_Send_PeriodElapsedCallback();
    }
}
