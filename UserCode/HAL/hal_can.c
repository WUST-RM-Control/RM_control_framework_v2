//
// Created by ye on 2026/8/22.
//

#include "hal_can.h"

#include "fdcan.h"
#include "utils.h"


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
        // if (hfdcan == &hfdcan1) CAN1_ErrorCount = 0;
        // if (hfdcan == &hfdcan2) CAN2_ErrorCount = 0;
        // if (hfdcan == &hfdcan3) CAN3_ErrorCount = 0;

        return Status;
}

/*===| CAN节点分发框架(基于 can_node 基类) |===*/
#define CAN_NODE_MAX 32

static CAN_Node_HandleTypeDef *hcan_node_table[CAN_NODE_MAX];
static uint8_t                 CAN_Node_Count = 0;

//注册节点(重复注册则更新回调)
void CAN_Node_Register(CAN_Node_HandleTypeDef *hcan_node, CAN_Node_Handler handler)
{
        if (hcan_node == NULL || handler == NULL) return;

        for (uint8_t i = 0; i < CAN_Node_Count; i++)
        {
                if (hcan_node_table[i] == hcan_node)
                {
                        hcan_node_table[i]->handler = handler;
                        return;
                }
        }

        if (CAN_Node_Count >= CAN_NODE_MAX) return;

        hcan_node_table[CAN_Node_Count]         = hcan_node;
        hcan_node_table[CAN_Node_Count]->handler = handler;
        CAN_Node_Count++;
}

//注销节点
void CAN_Node_UnRegister(CAN_Node_HandleTypeDef *hcan_node)
{
        for (uint8_t i = 0; i < CAN_Node_Count; i++)
        {
                if (hcan_node_table[i] == hcan_node)
                {
                        CAN_Node_Count--;
                        hcan_node_table[i] = hcan_node_table[CAN_Node_Count];
                        return;
                }
        }
}

//CAN节点构造: 绑定FDCAN句柄与收发ID与回调函数
void CAN_Node_Ctor(CAN_Node_HandleTypeDef *hcan_node, FDCAN_HandleTypeDef *hfdcan, uint16_t CAN_Send_ID, uint16_t CAN_Feedback_ID, CAN_Node_Handler node_handler, uint16_t err_tick_Timeout, uint16_t err_count_maximum, Err_Handler err_handler)
{
        Err_Ctor(&hcan_node->herr, err_tick_Timeout, err_count_maximum, err_handler);

        hcan_node->hfdcan          = hfdcan;
        hcan_node->CAN_Send_ID     = CAN_Send_ID;
        hcan_node->CAN_Feedback_ID = CAN_Feedback_ID;

        CAN_Node_Register(hcan_node, node_handler);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
        FDCAN_RxHeaderTypeDef RxHeader;
        uint8_t CAN_RX_Data[8];

        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, CAN_RX_Data);

        //清零对应CAN的错误计时
        // if (hfdcan == &hfdcan1) CAN1_Err_Ticker = 0;
        // if (hfdcan == &hfdcan2) CAN2_Err_Ticker = 0;
        // if (hfdcan == &hfdcan3) CAN3_Err_Ticker = 0;

        uint16_t CAN_RX_ID = RxHeader.Identifier;

        //基于can_node的通用分发: 按(总线, 反馈ID)匹配节点, 调用节点数据回调
        for (uint8_t i = 0; i < CAN_Node_Count; i++)
        {
                if (hcan_node_table[i]->hfdcan          == hfdcan &&
                    hcan_node_table[i]->CAN_Feedback_ID == CAN_RX_ID)
                {
                        hcan_node_table[i]->handler(hcan_node_table[i], CAN_RX_Data);
                }
        }
}

//CAN-发送标准帧(FDCAN, CANID，发送数据数组（八字节））
void CAN_Send_Data_STD(CAN_Node_HandleTypeDef *hcan_node, const uint8_t *TX_Data)
{
        FDCAN_TxHeaderTypeDef TxHeader;
        TxHeader.Identifier          = hcan_node->CAN_Send_ID;
        TxHeader.IdType              = FDCAN_STANDARD_ID;
        TxHeader.TxFrameType         = FDCAN_DATA_FRAME;
        TxHeader.DataLength          = FDCAN_DLC_BYTES_8;
        TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        TxHeader.BitRateSwitch       = FDCAN_BRS_OFF;
        TxHeader.FDFormat            = FDCAN_CLASSIC_CAN;
        TxHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
        TxHeader.MessageMarker       = 0;

        uint32_t OverTick = 0;
        while (HAL_FDCAN_GetTxFifoFreeLevel(hcan_node->hfdcan) == 0)
        {
                OverTick++;
                if (OverTick > 1000) break;
        }
        HAL_FDCAN_AddMessageToTxFifoQ(hcan_node->hfdcan, &TxHeader, TX_Data);
}

//CAN-发送拓展帧(FDCAN, CANID，发送数据数组，发送数据长度）
void CAN_Send_Data_EXD(CAN_Node_HandleTypeDef *hcan_node, uint8_t *TX_Data, uint8_t Length)
{
        FDCAN_TxHeaderTypeDef TxHeader;
        TxHeader.Identifier          = hcan_node->CAN_Send_ID;
        TxHeader.IdType              = FDCAN_EXTENDED_ID;
        TxHeader.TxFrameType         = FDCAN_DATA_FRAME;
        TxHeader.DataLength          = Length;
        TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        TxHeader.BitRateSwitch       = FDCAN_BRS_OFF;
        TxHeader.FDFormat            = FDCAN_CLASSIC_CAN;
        TxHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
        TxHeader.MessageMarker       = 0;

        uint32_t OverTick = 0;
        while (HAL_FDCAN_GetTxFifoFreeLevel(hcan_node->hfdcan) == 0)
        {
                OverTick++;
                if (OverTick > 1000) break;
        }
        HAL_FDCAN_AddMessageToTxFifoQ(hcan_node->hfdcan, &TxHeader, TX_Data);
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
