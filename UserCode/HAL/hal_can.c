//
// Created by ye on 2026/8/22.
//

#include "hal_can.h"

#include "fdcan.h"


static uint16_t CAN1_Err_Ticker;
static uint16_t CAN2_Err_Ticker;
static uint16_t CAN3_Err_Ticker;

static uint16_t CAN1_ErrorCount;
static uint16_t CAN2_ErrorCount;
static uint16_t CAN3_ErrorCount;

//总线离线计时: 每周期递增(收到数据在RX回调中清零)
void CAN_Bus_Tick(void)
{
        CAN1_Err_Ticker++;
        CAN2_Err_Ticker++;
        CAN3_Err_Ticker++;
}

//查询总线在线状态
uint8_t CAN_Get_Bus_Online(FDCAN_HandleTypeDef *hfdcan)
{
        if (hfdcan == &hfdcan1) return CAN1_Err_Ticker < CAN_OFFLINE_TICK;
        if (hfdcan == &hfdcan2) return CAN2_Err_Ticker < CAN_OFFLINE_TICK;
        if (hfdcan == &hfdcan3) return CAN3_Err_Ticker < CAN_OFFLINE_TICK;
        return 0;
}

//查询总线硬件错误计数
uint16_t CAN_Get_Bus_ErrorCount(FDCAN_HandleTypeDef *hfdcan)
{
        if (hfdcan == &hfdcan1) return CAN1_ErrorCount;
        if (hfdcan == &hfdcan2) return CAN2_ErrorCount;
        if (hfdcan == &hfdcan3) return CAN3_ErrorCount;
        return 0;
}

//FDCAN硬件错误中断回调(错误警告/被动错误/总线关闭)
void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
        if (hfdcan == &hfdcan1) CAN1_ErrorCount++;
        if (hfdcan == &hfdcan2) CAN2_ErrorCount++;
        if (hfdcan == &hfdcan3) CAN3_ErrorCount++;
}

//CAN总线重启: 停止→去初始化→重新初始化→启动→重配过滤器/中断, 并清零硬件错误计数
HAL_StatusTypeDef CAN_Bus_Restart(FDCAN_HandleTypeDef *hfdcan)
{
        HAL_StatusTypeDef Status;

        HAL_FDCAN_Stop(hfdcan);
        HAL_FDCAN_DeInit(hfdcan);

        //复用 hfdcan->Init(MX_FDCANx_Init 已配置, DeInit 不清空); Init内部会重挂Rx/Error回调并重跑MspInit
        Status = HAL_FDCAN_Init(hfdcan);
        if (Status == HAL_OK)
        {
                HAL_FDCAN_Start(hfdcan);
                CAN_Filter_Init(hfdcan); //重配过滤器 + 重新激活接收/错误中断
        }

        //清零硬件错误计数(恢复健康后重新累计)
        if (hfdcan == &hfdcan1) CAN1_ErrorCount = 0;
        if (hfdcan == &hfdcan2) CAN2_ErrorCount = 0;
        if (hfdcan == &hfdcan3) CAN3_ErrorCount = 0;

        return Status;
}

/*===| CAN节点分发框架(基于 can_node 基类) |===*/
#define CAN_NODE_MAX 32

typedef struct
{
        CAN_Node_HandleTypeDef *node;    //节点(内含总线+反馈ID)
        CAN_Node_Handler        handler; //数据回调
} CAN_Node_Entry;

static CAN_Node_Entry CAN_Node_Table[CAN_NODE_MAX];
static uint8_t        CAN_Node_Count = 0;

//CAN节点构造: 绑定FDCAN句柄与收发ID
void CAN_Node_Ctor(CAN_Node_HandleTypeDef *node, FDCAN_HandleTypeDef *hfdcan, uint16_t CAN_Send_ID, uint16_t CAN_Feedback_ID)
{
        node->hfdcan          = hfdcan;
        node->CAN_Send_ID     = CAN_Send_ID;
        node->CAN_Feedback_ID = CAN_Feedback_ID;
}

//注册节点(重复注册则更新回调)
void CAN_Node_Register(CAN_Node_HandleTypeDef *node, CAN_Node_Handler handler)
{
        if (node == NULL || handler == NULL) return;

        for (uint8_t i = 0; i < CAN_Node_Count; i++)
        {
                if (CAN_Node_Table[i].node == node)
                {
                        CAN_Node_Table[i].handler = handler;
                        return;
                }
        }

        if (CAN_Node_Count >= CAN_NODE_MAX) return;

        CAN_Node_Table[CAN_Node_Count].node    = node;
        CAN_Node_Table[CAN_Node_Count].handler = handler;
        CAN_Node_Count++;
}

//注销节点
void CAN_Node_UnRegister(CAN_Node_HandleTypeDef *node)
{
        for (uint8_t i = 0; i < CAN_Node_Count; i++)
        {
                if (CAN_Node_Table[i].node == node)
                {
                        CAN_Node_Count--;
                        CAN_Node_Table[i] = CAN_Node_Table[CAN_Node_Count];
                        return;
                }
        }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
        FDCAN_RxHeaderTypeDef RxHeader;
        uint8_t CAN_RX_Data[8];

        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, CAN_RX_Data);

        //清零对应CAN的错误计时
        if (hfdcan == &hfdcan1) CAN1_Err_Ticker = 0;
        if (hfdcan == &hfdcan2) CAN2_Err_Ticker = 0;
        if (hfdcan == &hfdcan3) CAN3_Err_Ticker = 0;

        uint16_t CAN_RX_ID = RxHeader.Identifier;

        //基于can_node的通用分发: 按(总线, 反馈ID)匹配节点, 调用节点数据回调
        for (uint8_t i = 0; i < CAN_Node_Count; i++)
        {
                if (CAN_Node_Table[i].node->hfdcan          == hfdcan &&
                    CAN_Node_Table[i].node->CAN_Feedback_ID == CAN_RX_ID)
                {
                        CAN_Node_Table[i].handler(CAN_Node_Table[i].node, CAN_RX_Data);
                }
        }
}

//CAN-发送标准帧(FDCAN, CANID，发送数据数组（八字节））
void CAN_Send_Data_STD(FDCAN_HandleTypeDef *hfdcan, const uint16_t ID, const uint8_t *TX_Data)
{
        FDCAN_TxHeaderTypeDef TxHeader;
        TxHeader.Identifier          = ID;
        TxHeader.IdType              = FDCAN_STANDARD_ID;
        TxHeader.TxFrameType         = FDCAN_DATA_FRAME;
        TxHeader.DataLength          = FDCAN_DLC_BYTES_8;
        TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        TxHeader.BitRateSwitch       = FDCAN_BRS_OFF;
        TxHeader.FDFormat            = FDCAN_CLASSIC_CAN;
        TxHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
        TxHeader.MessageMarker       = 0;

        uint32_t OverTick = 0;
        while (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) == 0)
        {
                OverTick++;
                if (OverTick > 1000) break;
        }
        HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, TX_Data);
}

//CAN-发送拓展帧(FDCAN, CANID，发送数据数组，发送数据长度）
void CAN_Send_Data_EXD(FDCAN_HandleTypeDef *hfdcan, uint32_t ID, uint8_t *TX_Data, uint8_t Length)
{
        FDCAN_TxHeaderTypeDef TxHeader;
        TxHeader.Identifier          = ID;
        TxHeader.IdType              = FDCAN_EXTENDED_ID;
        TxHeader.TxFrameType         = FDCAN_DATA_FRAME;
        TxHeader.DataLength          = Length;
        TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        TxHeader.BitRateSwitch       = FDCAN_BRS_OFF;
        TxHeader.FDFormat            = FDCAN_CLASSIC_CAN;
        TxHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
        TxHeader.MessageMarker       = 0;

        uint32_t OverTick = 0;
        while (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) == 0)
        {
                OverTick++;
                if (OverTick > 1000) break;
        }
        HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, TX_Data);


}

//CAN初始化
void CAN_Init(void)
{
        HAL_FDCAN_Start(&hfdcan1);
        CAN_Filter_Init(&hfdcan1);

        HAL_FDCAN_Start(&hfdcan2);
        CAN_Filter_Init(&hfdcan2);

        HAL_FDCAN_Start(&hfdcan3);
        CAN_Filter_Init(&hfdcan3);
}

//CAN过滤器初始化
void CAN_Filter_Init(FDCAN_HandleTypeDef *hfdcan)
{
        FDCAN_FilterTypeDef sFilterConfig;

        sFilterConfig.IdType       = FDCAN_STANDARD_ID;
        sFilterConfig.FilterIndex  = 0;
        sFilterConfig.FilterType   = FDCAN_FILTER_RANGE;
        sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
        sFilterConfig.FilterID1    = 0x00;
        sFilterConfig.FilterID2    = 0x7FF;

        HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig);
        HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
        //激活接收 + 错误中断(错误警告/被动错误/总线关闭)
        HAL_FDCAN_ActivateNotification(hfdcan,
                                       FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                                       FDCAN_IT_ERROR_WARNING |
                                       FDCAN_IT_ERROR_PASSIVE |
                                       FDCAN_IT_BUS_OFF,
                                       0);
}
