//
// Created by pzx on 2025/11/28.
//

#ifndef STEERING_WHEEL_TRY_CONTROL_CHASSIS_H
#define STEERING_WHEEL_TRY_CONTROL_CHASSIS_H
#pragma diag_suppress 550

/* Exported types ------------------------------------------------------------*/
#include "struct_typedef.h"


#define RUD_OPSI       1
#define RUD_NOT_OPSI   0
#define RUD_RESET      1
#define RUD_NOT_RESET  0

#define CHASSIS_MAX_SPEED 9000  // 底盘驱动轮最大速度
#define CHASSIS_MAX_VW    9000  // 底盘旋转最大速度
#define CHASSIS_SPEED_L   5500
#define CHASSIS_SPEED_M   6500
#define CHASSIS_SPEED_H   9000

#define RF_INITANGLE 149.32f
#define LF_INITANGLE -95.48f
#define LB_INITANGLE 34.89f
#define RB_INITANGLE -33.55f

extern float test_Vz;
extern int16_t ssssspeed[3];

extern float Ramp_Vy, Ramp_Vx;
extern CHAS_Power_classdef CHAS_Power;

/* --- 底盘驱动电机 ID --------------------------------------------------------*/
enum CHAS_DrvMotorID_e
{
    RF_201 = 0,
    LF_202 = 1,
    LB_203 = 2,
    RB_204 = 3
};
/* --- 底盘转向电机 ID --------------------------------------------------------*/
enum CHAS_RudMororID_e
{
    RF_205 = 0,
    LF_206 = 1,
    LB_207 = 2,
    RB_208 = 3
};

/* --- 底盘控制模式 ------------------------------------------------------------*/
enum CHAS_CtrlMode_e
{
    CHAS_DisableMode,	    // 失能模式
    CHAS_SpinMode ,	        // 小陀螺模式
    CHAS_SentryPCMode,     //radar控制模式

    CHAS_InitMode,          // 初始化阶段
    };


/* --- 转向轮电机相关参数 -------------------------------------------------------*/
typedef struct
{
    float Init_angle;   // 初始化校准角度
    float Target_angle; // 目标角度
    float PreTar_angle; // 前一次目标角度
    float Total_angle;  // 当前总角度
    int32_t Turns_cnt;//当前圈数
    int32_t TarTurns_cnt;//目标圈数
    int32_t Turns_flag;
}RUD_Param_t;

/* 滤波后的目标值 */
typedef struct
{
    float Vx;
    float Vy;
    float Vw;
}FilterTarget_t;

/* 加速曲线相关参数 */
typedef struct
{
    uint8_t accelerating;
    uint8_t decelerating;
    float linnerSpeed;
    float linnerSpeedLast;
    float accCnt;
    float accK = 0.25;
    float accKp;
    float deceleRecodeSpeed[2];
}AccelerateParam_t;
typedef struct
{
    int16_t Target_Vx;
    int16_t Target_Vy;
    int16_t Target_Vw;
}Target_Low;
typedef struct
{
    int16_t Target_Vx;
    int16_t Target_Vy;
    int16_t Target_Vw;
}Target_Up;
RUD_Param_t RUD_Param[4]; /*<! 转向轮相关参数 */

void Rudder_Solve(int16_t Vx, int16_t Vy, int16_t Vz, int16_t *cal_speed);   /*<！舵轮解算 */
void RudAngle_Calc(int16_t Vx, int16_t Vy, int16_t Vz);   /*<! 三轴速度解算舵角度 */
void RUDTotalAngle_Calc(Motor_GM6020* rudder_motor , int8_t motor_num , int8_t reset , uint8_t opposite);  /*<! 转向轮总角度计算 */
void RUDTargetAngle_Calc(int8_t motor_num , int8_t reset , uint8_t opposite);                             /*<! 转向轮目标角度计算 */
void DRV_PIDCalc(uint8_t motor, float target, float current);
void RUD_PIDCalc(uint8_t motor, float target, float current);
void Judge_DRV_Dir(uint8_t motor, float target, float current);   /*<! 判断驱动轮是否需要反向 */
void Angle_Cnt();
void Set_RudMaxOut(uint16_t maxout); /*<! 设置 转向轮最大输出 */


#endif //STEERING_WHEEL_TRY_CONTROL_CHASSIS_H