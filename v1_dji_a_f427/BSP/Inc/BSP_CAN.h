#ifndef BSP_CAN_H
#define BSP_CAN_H
#include "main.h"
#include "can.h"

#define CAN_LINE_BUSY  0
#define CAN_SUCCESS    1

extern uint8_t TxData[8];
extern uint8_t RxData[8];
extern CAN_RxHeaderTypeDef RXHeader;
extern CAN_TxHeaderTypeDef TXHeader;

typedef struct
{
		int16_t rh1;
	  int16_t	rh2;
	  int16_t rh3;
	  int16_t rh4;
}DR16;
extern DR16 DR16_num;

void CAN_Init(void);
void can_filter_init(void);
void HAL_CAN_RxFifo0MgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo0MsgPendingCallback_C(CAN_HandleTypeDef *hcan);

#endif
