#include "dvc_boardcomm.h"

void Class_BoardComm::Init(UART_HandleTypeDef *huart) {
    UART_Handler = huart;

    if (UART_Handler != nullptr) {
        HAL_UART_Receive_IT(UART_Handler, &Rx_Byte, 1);
    }
}

void Class_BoardComm::Send_Data(uint8_t data) {
if (UART_Handler == nullptr) return;    // Struct_Remote_Sync_Packet tx_pack;
    TX_Data.header = 0xAA;
    TX_Data.data = data ;
    TX_Data.tail = 0x55;
    HAL_UART_Transmit_IT(UART_Handler, (uint8_t*)&TX_Data, sizeof(Struct_Remote_Sync_Packet));
}

void Class_BoardComm::UART_RxCpltCallback() {
    HAL_UART_Receive_IT(UART_Handler, &Rx_Byte, 1);
}