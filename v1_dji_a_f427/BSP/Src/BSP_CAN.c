#include "BSP_CAN.h"

extern CAN_HandleTypeDef hcan1;
uint8_t TxData[8];
uint8_t RxData[8];

CAN_RxHeaderTypeDef RXHeader;
CAN_TxHeaderTypeDef TXHeader;
DR16 DR16_num;

void CAN_Init(void){
    can_filter_init();
    __HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}
//??????
void can_filter_init(void){
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
    HAL_CAN_GetRxMessage(hcan,CAN_FILTER_FIFO0,&RXHeader,RxData);

    HAL_CAN_RxFifo0MsgPendingCallback_C(&hcan1);
}

void HAL_CAN_RxFifo0MsgPendingCallback_C(CAN_HandleTypeDef *hcan){
    if(hcan==&hcan1){
        switch(RXHeader.StdId){
            case 0x300:{

                break;
            }
            default:{
            break;
            }
        }
    }
}


