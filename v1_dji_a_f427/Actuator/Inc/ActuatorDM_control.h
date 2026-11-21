#ifndef ACTUATORDM_CONTROL_H
#define ACTUATORDM_CONTROL_H


#include "MotorStructs.h"
#include "CANDeviceManager.h"
#include "DataType.h"

class CANDeviceManager;

class Motor_DM_Control
{
private:
	uint8_t Data_DM_CAN[8];
	CANDeviceManager *canManager_DM;
	
public:
    Motor_DM_Control(CANDeviceManager* canManager = nullptr);
	~Motor_DM_Control();

	// 电机初始化
	void Handle_DM_Information(struct can_frame &frame, Motor_DM_CAN_Recieve_Struct &motorData);
	void CANable_DM_CAN_Bus_inti_set(CANable_DM_CAN_Bus_Struct *Leg_Data);
	void CANable_DM_CAN_Bus_Init();
	void Actuator_DM_Enable(Motor_DM_CAN_Send_Struct *Motor_Data);
	void Actuator_DM_Disable(Motor_DM_CAN_Send_Struct *Motor_Data);
	void Actuator_DM_Zore(Motor_DM_CAN_Send_Struct *Motor_Data);
	void CANable_DM_Send_Control(Motor_DM_CAN_Send_Struct *Motor_Data);
	void Actuator_DM_Passive_SET(Motor_DM_CAN_Send_Struct *Motor_Data);
	void ENABLE_ALL_DM_Actuator(int delay_us);
	void DISABLE_ALL_Actuator_DM(int delay_us);
	void ZERO_ALL_Actuator_DM(int delay_us);
	void PASSIVE_ALL_Actuator_DM(int delay_us);
	void CANable_TX_ALL_Actuator_DM(int delay_us);


};

#endif
