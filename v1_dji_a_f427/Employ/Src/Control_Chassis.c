#include ".h“
/**
 * @brief      舵轮解算
 * @param[in]  Vx
 * @param[in]  Vy
 * @param[in]  Vw
 * @param[in]  cal_speed
 * @retval     None
 */
void Rudder_Solve(int16_t Vx, int16_t Vy, int16_t Vw, int16_t *cal_speed) {
    float Param = 1.0f;
    float MaxSpeed = 0.0f;

    static float const theta = atan(1.0 / 1.0);
    static uint8_t compare_flag = true;
    static uint16_t No_move_cnt = 0;

    VxVy_Coe = 1.0f;

    ssssspeed[0] = Vx;
    ssssspeed[1] = Vy;
    ssssspeed[2] = Vw;

    /* 三轴速度解算转向轮角度 ---------------------------------------------------------------------------*/
    RudAngle_Calc(Vx, Vy, Vw);

    if (Vx == 0 && Vy == 0) {
        if (abs(Vw) < 70) //---IMU零漂产生的自旋速度
        {
            if (No_move_cnt < 500/* 500 */) //--- 静止后1000ms期间不允许底盘跟随
            {
                No_move_cnt++;
                Vw = 0;
            } else {
            }
        }
    } else {
        No_move_cnt = 0;
    }

    /* 驱动轮 速度解算 ---------------------------------------------------------------------------------*/
    cal_speed[RF_201] = sqrt(pow(Vx - Vw * arm_sin_f32(theta), 2) + pow(Vy + Vw * arm_cos_f32(theta), 2));
    cal_speed[LF_202] = -sqrt(pow(Vx - Vw * arm_sin_f32(theta), 2) + pow(Vy - Vw * arm_cos_f32(theta), 2));
    cal_speed[LB_203] = sqrt(pow(Vx + Vw * arm_sin_f32(theta), 2) + pow(Vy - Vw * arm_cos_f32(theta), 2));
    cal_speed[RB_204] = -sqrt(pow(Vx + Vw * arm_sin_f32(theta), 2) + pow(Vy + Vw * arm_cos_f32(theta), 2));

    if (Infantry.Write_Msg[Uphill_Mode] == true) {
        cal_speed[RF_201] *= 0.5f;
        cal_speed[LF_202] *= 0.5f;
    }

    // 寻找最大速度
    for (uint8_t i = 0; i < 4; i++) {
        if (abs(cal_speed[i]) > MaxSpeed) {
            MaxSpeed = abs(cal_speed[i]);
        }
    }

    // 速度分配
    if (MaxSpeed > VxVy_Limit) {
        Param = (float) VxVy_Limit / MaxSpeed;
    }

    cal_speed[RF_201] *= Param;
    cal_speed[LF_202] *= Param;
    cal_speed[LB_203] *= Param;
    cal_speed[RB_204] *= Param;


    /* 计算转向轮角度当前值 计算当前舵轮的总角度 换算为角度制----------------------------------------------------------------------------*/
    RUDTotalAngle_Calc(RUD_Motor + RF_205, RF_205, RUD_NOT_RESET, RUD_NOT_OPSI);
    RUDTotalAngle_Calc(RUD_Motor + LF_206, LF_206, RUD_NOT_RESET, RUD_NOT_OPSI);
    RUDTotalAngle_Calc(RUD_Motor + LB_207, LB_207, RUD_NOT_RESET, RUD_NOT_OPSI);
    RUDTotalAngle_Calc(RUD_Motor + RB_208, RB_208, RUD_NOT_RESET, RUD_NOT_OPSI);

    /* 计算转向轮角度目标值 ----------------------------------------------------------------------------*/
    RUDTargetAngle_Calc(RF_205, RUD_NOT_RESET, RUD_NOT_OPSI);
    RUDTargetAngle_Calc(LF_206, RUD_NOT_RESET, RUD_NOT_OPSI);
    RUDTargetAngle_Calc(LB_207, RUD_NOT_RESET, RUD_NOT_OPSI);
    RUDTargetAngle_Calc(RB_208, RUD_NOT_RESET, RUD_NOT_OPSI);

    for (uint8_t i = 0; i < 4; i++) {
        //--- 转向轮 劣弧旋转
        RUD_Param[i].Target_angle = Turn_InferiorArc(i, RUD_Param[i].Target_angle, RUD_Param[i].Total_angle);
    }
}

/**
 * @brief      三轴速度解算舵角度
 * @param[in]  None
 * @retval     None
 */
void RudAngle_Calc(int16_t Vx, int16_t Vy, int16_t Vw) {
    static float const theta = atan(1.0 / 1.0);
    static uint16_t No_move_cnt = 0;

    static float last_vx = 0, last_vy = 0, last_vw = 0;

    //--- TODO 因为陀螺仪零漂会产生小幅度的Vw速度导致这里的InitAngle会来回跳变(暂时用个低级的方法解决)√

    //--- TODO 还有就是45度刹车可能由于车的重心较高，高速运动的时候会刹的比较猛会出现前倾
    //---      一个思路是不同的功率给不同的斜坡
    //---      另一个是刹车的时候先保持原本的角度，先让前轮失能，等车稳定后再呈45度归中
    //---      再一个是用高级点的加速曲线？

    if (Vx == 0 && Vy == 0) {
        No_move_flag = true;
        if (abs(Vw) < 70) //---IMU零漂产生的自旋速度
        {
            if (No_move_cnt < 500) //--- 静止后1000ms期间不允许底盘跟随
            {
                No_move_cnt++;
                Vw = 0;
            } else {
            }
        }
    } else {
        spin_flag = false;
        No_move_flag = false;
        No_move_cnt = 0;
    }

    if (Vx == 0 && Vy == 0 && Vw == 0) {
        Move_flag = false;

        if (Brake_cnt < 300) {
            Brake_cnt++;

            //--- 在上一帧不是静止的时候产生一个Vw的速度，为了不让他目标角度有一个归零的动作
            last_vw = (spin_flag == true ? 50 : 0);

            //--- 使用上一次的目标速度来保存上一次的目标角度
            RUD_Param[RF_205].Target_angle = atan2(last_vx - last_vw * (Radius * arm_sin_f32(theta)),
                                                   last_vy + last_vw * Radius * arm_cos_f32(theta)) * (180 / PI);
            RUD_Param[LF_206].Target_angle = atan2(last_vx - last_vw * (Radius * arm_sin_f32(theta)),
                                                   last_vy - last_vw * Radius * arm_cos_f32(theta)) * (180 / PI);
            RUD_Param[LB_207].Target_angle = atan2(last_vx + last_vw * (Radius * arm_sin_f32(theta)),
                                                   last_vy - last_vw * Radius * arm_cos_f32(theta)) * (180 / PI);
            RUD_Param[RB_208].Target_angle = atan2(last_vx + last_vw * (Radius * arm_sin_f32(theta)),
                                                   last_vy + last_vw * Radius * arm_cos_f32(theta)) * (180 / PI);
        } else {
            spin_flag = false;
            //--- 45度归中
            RUD_Param[RF_205].Init_angle = RUD_InitAngle[RF_205] - 45;
            RUD_Param[LF_206].Init_angle = RUD_InitAngle[LF_206] + 45;
            RUD_Param[LB_207].Init_angle = RUD_InitAngle[LB_207] - 45;
            RUD_Param[RB_208].Init_angle = RUD_InitAngle[RB_208] + 45;

            //--- 目标角度归零
            for (uint8_t i = 0; i < 4; i++) {
                RUD_Param[i].Target_angle = 0;
            }
        }
    } else {
        if (No_move_flag != true) {
            Brake_cnt = 0;
        }
        Move_flag = true;

        //--- 解除45度归中
        RUD_Param[RF_205].Init_angle = RUD_InitAngle[RF_205];
        RUD_Param[LF_206].Init_angle = RUD_InitAngle[LF_206];
        RUD_Param[LB_207].Init_angle = RUD_InitAngle[LB_207];
        RUD_Param[RB_208].Init_angle = RUD_InitAngle[RB_208];

        //--- 有目标速度的时候才进行舵轮解算的计算
        RUD_Param[RF_205].Target_angle = atan2(Vx - Vw * (Radius * arm_sin_f32(theta)),
                                               Vy + Vw * Radius * arm_cos_f32(theta)) * (180 / PI);
        RUD_Param[LF_206].Target_angle = atan2(Vx - Vw * (Radius * arm_sin_f32(theta)),
                                               Vy - Vw * Radius * arm_cos_f32(theta)) * (180 / PI);
        RUD_Param[LB_207].Target_angle = atan2(Vx + Vw * (Radius * arm_sin_f32(theta)),
                                               Vy - Vw * Radius * arm_cos_f32(theta)) * (180 / PI);
        RUD_Param[RB_208].Target_angle = atan2(Vx + Vw * (Radius * arm_sin_f32(theta)),
                                               Vy + Vw * Radius * arm_cos_f32(theta)) * (180 / PI);

        if (abs(Vw) > 100) {
            spin_flag = true;
        }

        //--- 无目标速度的时候不使用上一次角度来保存是因为跟随模式下IMU静止的瞬间会产生轻微的Vw速度
        last_vx = Vx;
        last_vy = 0;
        last_vw = 0;
    }

    //--- 更改为保存经过劣弧处理的目标角度(现在暂时不用这个上一帧的角度)
    // for(uint8_t i = 0 ; i < 4 ; i++)
    // {
    //     RUD_Param[i].PreTar_angle = RUD_Param[i].Target_angle;
    // }
}

/**
 * @brief      转向轮当前总角度计算
 * @param[in]  rudder_motor
 * @param[in]  motor_num
 * @param[in]  reset
 * @param[in]  opposite
 * @retval     None
 */
void RUDTotalAngle_Calc(Motor_GM6020 *rudder_motor, int8_t motor_num, int8_t reset, uint8_t opposite) {
    float Cur_encoder = rudder_motor->getencoder() / 8192 * 360; //换算乘当前角度，0~360°
    static float Pre_encoder[4] = {0};

    if (Cur_encoder - Pre_encoder[motor_num] > 180) //过零处理
    {
        RUD_Param[motor_num].Turns_cnt--;
    } else if (Cur_encoder - Pre_encoder[motor_num] < -180) //过零处理
    {
        RUD_Param[motor_num].Turns_cnt++;
    }
    Pre_encoder[motor_num] = Cur_encoder;

    if (reset == true) //---圈数清零
    {
        RUD_Param[motor_num].Turns_cnt = 0;
    }

    if (opposite == false) //---是否反向->指的是舵机极性是否反转
    {
        RUD_Param[motor_num].Total_angle = Cur_encoder + RUD_Param[motor_num].Turns_cnt * 360; //计算舵轮的总角度
    } else if (opposite == true) {
        RUD_Param[motor_num].Total_angle = -Cur_encoder - RUD_Param[motor_num].Turns_cnt * 360;
    }
}

float error;

void RUDTargetAngle_Calc(int8_t motor_num, int8_t reset, uint8_t opposite) {
    float Cur_Target = RUD_Param[motor_num].Target_angle;
    static float Pre_Target[4] = {
        RUD_Param[0].Init_angle, RUD_Param[1].Init_angle, RUD_Param[2].Init_angle, RUD_Param[3].Init_angle
    };
    static float Pre_TempTarget[4] = {
        RUD_Param[0].Init_angle, RUD_Param[1].Init_angle, RUD_Param[2].Init_angle, RUD_Param[3].Init_angle
    };

    static float temp_pre = 0; //--- 保存劣弧旋转的上一次未经过零处理的目标角度

    //--- 将目标角度与当前角度换算到同一个周期中
    RUD_Param[motor_num].TarTurns_cnt = (int32_t)(RUD_Param[motor_num].Total_angle / 180.0f) - (int32_t)(
                                            RUD_Param[motor_num].Total_angle / 360.0f);

    RUD_Param[motor_num].Target_angle = RUD_Param[motor_num].TarTurns_cnt * 360.0f + Cur_Target + RUD_Param[motor_num].
                                        Init_angle;

    temp_pre = Cur_Target;

    //---- 当前目标角度和当前角度的误差
    error = RUD_Param[motor_num].Target_angle - RUD_Param[motor_num].Total_angle;

    //--- 误差限幅
    // Constrain(&error, -180.0f, 180.0f); //--- 用这种限幅方法好像不太行
    AngleLimit(&error);

    //--- 如果角度误差大于90度则将速度反向并将目标角度叠加180度
    if (fabs(error) > 90.0f && (Move_flag == true || No_move_flag == false || Brake_cnt < 500)
        /* && Mode != CHAS_SpinMode */) {
        //--- 目标值叠加半个周期
        RUD_Param[motor_num].Target_angle += 180.0f;

        temp_pre += 180.0f;

        //--- 驱动轮反转
        Cal_Speed[motor_num] = -Cal_Speed[motor_num];
        //--- 确保目标角度和当前角度处于同一个周期
        if (RUD_Param[motor_num].Target_angle > RUD_Param[motor_num].TarTurns_cnt * 360.0f + 180.0f) {
            RUD_Param[motor_num].Target_angle -= 360.0f;
        } else if (RUD_Param[motor_num].Target_angle<RUD_Param[motor_num].TarTurns_cnt * 360.0f - 180.0f) {
            RUD_Param[motor_num].Target_angle += 360.0f;
        }
    }
    /*--------------------------------*/
    Pre_TempTarget[motor_num] = RUD_Param[motor_num].Target_angle;

    RUD_Param[motor_num].PreTar_angle = temp_pre; //--- 保存劣弧旋转的上一次未经过零处理的目标角度
}
