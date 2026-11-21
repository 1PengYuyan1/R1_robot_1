#include "drv_fdcan.h"

#if defined(__GNUC__)
#define ALIGN_32 __attribute__((aligned(32)))
#else
#define ALIGN_32
#endif

ALIGN_32 Struct_FDCAN_Manage_Object CAN1_Manage_Object = {0};
ALIGN_32 Struct_FDCAN_Manage_Object CAN2_Manage_Object = {0};
ALIGN_32 Struct_FDCAN_Manage_Object CAN3_Manage_Object = {0};

/**
 * @brief 内部辅助函数：配置单个标准帧 ID 到专用 Rx Buffer
 * @param hfdcan FDCAN句柄
 * @param FilterIndex 过滤器索引 (0-127)
 * @param ID 要匹配的 ID (例如 0x201)
 * @param RxBufferIndex 目标 Buffer 索引 (0-63)
 */
static void fdcan_filter_to_buffer_config(FDCAN_HandleTypeDef* hfdcan, uint32_t FilterIndex, uint32_t ID,
                                          uint32_t RxBufferIndex)
{
    FDCAN_FilterTypeDef sFilterConfig = {0};

    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = FilterIndex;
    sFilterConfig.FilterType = FDCAN_FILTER_RANGE; // Range 模式下，ID1=ID2 表示精准匹配
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXBUFFER; // 关键：指向专用 Buffer
    sFilterConfig.FilterID1 = ID;
    sFilterConfig.FilterID2 = ID; // 精准匹配
    sFilterConfig.RxBufferIndex = RxBufferIndex;

    if (HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig) != HAL_OK)
    {
        // Error_Handler(); // 可以在这里添加错误处理
    }
}

/**
 * @brief 内部辅助函数：配置范围/掩码过滤到 FIFO0 (适配 H7, 用于非电机数据)
 */
static void fdcan_filter_to_fifo0_config(FDCAN_HandleTypeDef* hfdcan, uint32_t FilterIndex, uint32_t ID, uint32_t Mask)
{
    FDCAN_FilterTypeDef sFilterConfig = {0};

    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = FilterIndex;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // 指向 FIFO0
    sFilterConfig.FilterID1 = ID;
    sFilterConfig.FilterID2 = Mask;

    HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig);
}

/**
 * @brief 初始化CAN总线 (适配 FDCAN)
 *
 * @param hfdcan FDCAN编号
 * @param Callback_Function 处理回调函数
 */
void CAN_Init(FDCAN_HandleTypeDef* hfdcan, CAN_Call_Back Callback_Function)
{
    // 1. 配置全局过滤器 (处理漏网之鱼)
    // 这里的策略是：不匹配专用 Buffer 的标准帧，放入 FIFO0；拒绝远程帧
    HAL_FDCAN_ConfigGlobalFilter(hfdcan,
                                 FDCAN_ACCEPT_IN_RX_FIFO0,
                                 FDCAN_ACCEPT_IN_RX_FIFO0,
                                 FDCAN_REJECT_REMOTE,
                                 FDCAN_REJECT_REMOTE);

    // 2. 根据实例进行不同的过滤器配置
    if (hfdcan->Instance == FDCAN1)
    {
        memset(&CAN1_Manage_Object, 0, sizeof(Struct_FDCAN_Manage_Object));
        CAN1_Manage_Object.CAN_Handler = hfdcan;
        CAN1_Manage_Object.Callback_Function = Callback_Function;

        // CAN1 的配置
        fdcan_filter_to_buffer_config(hfdcan, 0, 0x11, 0);
        fdcan_filter_to_buffer_config(hfdcan, 1, 0x12, 1);
        fdcan_filter_to_buffer_config(hfdcan, 2, 0x13, 2);
        fdcan_filter_to_buffer_config(hfdcan, 3, 0x14, 3);
        // (可选) 配置其他 ID 进 FIFO0, 使用索引 4
        // fdcan_filter_to_fifo0_config(hfdcan, 0, 0x000, 0x000);
    }
    else if (hfdcan->Instance == FDCAN2)
    {
        memset(&CAN2_Manage_Object, 0, sizeof(Struct_FDCAN_Manage_Object));
        CAN2_Manage_Object.CAN_Handler = hfdcan;
        CAN2_Manage_Object.Callback_Function = Callback_Function;

        // CAN2 的配置
        fdcan_filter_to_buffer_config(hfdcan, 0, 0x201, 0);
        fdcan_filter_to_buffer_config(hfdcan, 1, 0x301, 1);
        fdcan_filter_to_buffer_config(hfdcan, 2, 0x401, 2);
        fdcan_filter_to_buffer_config(hfdcan, 3, 0x501, 3);
        // (可选) 配置其他 ID 进 FIFO0, 使用索引 4
        // fdcan_filter_to_fifo0_config(hfdcan, 0, 0x000, 0x000);
    }
    else if (hfdcan->Instance == FDCAN3)
    {
        memset(&CAN3_Manage_Object, 0, sizeof(Struct_FDCAN_Manage_Object));
        CAN3_Manage_Object.CAN_Handler = hfdcan;
        CAN3_Manage_Object.Callback_Function = Callback_Function;

        // CAN3 的配置
        fdcan_filter_to_buffer_config(hfdcan, 0, 0x05, 0);
        fdcan_filter_to_buffer_config(hfdcan, 1, 0x06, 1);
        fdcan_filter_to_buffer_config(hfdcan, 2, 0x07, 2);
        fdcan_filter_to_buffer_config(hfdcan, 3, 0x08, 3);
        // (可选) 配置其他 ID 进 FIFO0, 使用索引 4
        // fdcan_filter_to_fifo0_config(hfdcan, 0, 0x000, 0x000);
    }
    // 3. 开启 FDCAN 模块
    HAL_FDCAN_Start(hfdcan);

    // 4. 开启中断
    // H7 FDCAN 需要分别开启 "Rx Buffer New Message" (针对专用Buffer) 和 "Rx FIFO0" (针对FIFO)
    HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_BUFFER_NEW_MESSAGE, 0);
    HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

/**
 * @brief 发送数据帧 (适配 FDCAN FIFO 发送)
 *
 * @param hfdcan FDCAN编号
 * @param ID ID
 * @param Data 被发送的数据指针
 * @param Length 长度
 * @return uint8_t 执行状态
 */
uint8_t CAN_Send_Data(FDCAN_HandleTypeDef* hfdcan, uint16_t ID, uint8_t* Data, uint16_t Length)
{
    FDCAN_TxHeaderTypeDef tx_header;

    //检测传参是否正确
    assert_param(hfdcan != NULL);

    tx_header.Identifier = ID;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = FDCAN_DLC_BYTES_8; // 经典 CAN 固定 8 字节
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF; // 经典模式不加速
    tx_header.FDFormat = FDCAN_CLASSIC_CAN; // 经典模式
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    // 使用 AddMessageToTxFifoQ 替代 F4 的 AddTxMessage
    return (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, Data));
}

/**
 * @brief CAN的TIM定时器中断发送回调函数
 *
 */
void TIM_1ms_CAN_PeriodElapsedCallback()
{
    // DJI电机 / 达妙电机发送逻辑
    // 逻辑保持不变，只需注意 hcan1 -> hfdcan1

    static int mod2 = 0;
    mod2++;
    if (mod2 == 2)
    {
        mod2 = 0;
    }
}

/**
 * @brief HAL库 FDCAN 专用 Buffer 接收中断回调 (H7 新增)
 * 用于处理电机数据 (ID 0x201-0x204)
 */
void HAL_FDCAN_RxBufferNewMessageCallback(FDCAN_HandleTypeDef* hfdcan)
{
    Struct_FDCAN_Manage_Object* Manage_Obj = NULL;

    if (hfdcan->Instance == FDCAN1) Manage_Obj = &CAN1_Manage_Object;
    else if (hfdcan->Instance == FDCAN2) Manage_Obj = &CAN2_Manage_Object;
    else if (hfdcan->Instance == FDCAN3) Manage_Obj = &CAN3_Manage_Object;
    else return;

    // if (init_finished == false) return;

    for (uint32_t i = 0; i < 4; i++)
    {
        if (HAL_FDCAN_IsRxBufferMessageAvailable(hfdcan, i))
        {
            HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_BUFFER0 + i, &Manage_Obj->Rx_Buffer.Header,
                                   Manage_Obj->Rx_Buffer.Data);
            if (init_finished && Manage_Obj->Callback_Function != NULL)
                Manage_Obj->Callback_Function(&Manage_Obj->Rx_Buffer);
        }
    }
}

/**
 * @brief HAL库 FDCAN FIFO0 接收中断回调 (兼容旧逻辑)
 * 用于处理漏网之鱼或其他非电机数据
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs)
{
    Struct_FDCAN_Manage_Object* Manage_Obj = NULL;

    if (hfdcan->Instance == FDCAN1) Manage_Obj = &CAN1_Manage_Object;
    else if (hfdcan->Instance == FDCAN2) Manage_Obj = &CAN2_Manage_Object;
    else if (hfdcan->Instance == FDCAN3) Manage_Obj = &CAN3_Manage_Object;
    else return;

    if (init_finished == false) return;

    // 从 FIFO0 读取数据
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &Manage_Obj->Rx_Buffer.Header, Manage_Obj->Rx_Buffer.Data);
        if (Manage_Obj->Callback_Function != NULL)
        {
            Manage_Obj->Callback_Function(&Manage_Obj->Rx_Buffer);
        }
    }
}

