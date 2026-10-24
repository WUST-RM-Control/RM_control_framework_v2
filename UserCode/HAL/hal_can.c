//
// Created by ye on 2026/8/22.
//

#include "hal_can.h"

#include <string.h>

#include "task.h"

#define CAN_BUS_MAX       3
#define CAN_TX_DEPTH      32
#define CAN_TX_TIMEOUT_MS 10
#define CAN_TX_TASK_PRIO  10

//每条总线一套发送缓冲+任务: 一条总线堵塞不影响其他总线
typedef struct
{
        FDCAN_HandleTypeDef *hfdcan;
        MessageBufferHandle_t TxBuffer;
        TaskHandle_t          TxTask;
} CAN_Bus_StructTypeDef;

static CAN_Bus_StructTypeDef CAN_Bus_Table[CAN_BUS_MAX];
static uint8_t               CAN_Bus_Count = 0;

volatile uint32_t CAN_Tx_Drop_Count = 0;

static CAN_Bus_StructTypeDef *CAN_Bus_Find(FDCAN_HandleTypeDef *hfdcan)
{
        for (uint8_t i = 0; i < CAN_Bus_Count; i++)
        {
                if (CAN_Bus_Table[i].hfdcan == hfdcan) return &CAN_Bus_Table[i];
        }

        return NULL;
}



//CAN总线重启: 停止→去初始化→重新初始化→启动→重配过滤器/中断, 并清零硬件错误计数
HAL_StatusTypeDef CAN_Restart(FDCAN_HandleTypeDef *hfdcan)
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
void CAN_Node_Ctor(CAN_Node_HandleTypeDef *hcan_node, FDCAN_HandleTypeDef *hfdcan, uint16_t CAN_Send_ID, uint16_t CAN_Feedback_ID, CAN_Node_Handler node_handler, uint16_t err_tick_Timeout, uint16_t err_count_maximum, err_handler err_handler)
{
        Err_Ctor(&hcan_node->herr, err_tick_Timeout, err_count_maximum, err_handler);

        hcan_node->hfdcan          = hfdcan;
        hcan_node->CAN_Send_ID     = CAN_Send_ID;
        hcan_node->CAN_Feedback_ID = CAN_Feedback_ID;

        CAN_Node_Register(hcan_node, node_handler);
}

//CAN接收回调
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
        FDCAN_RxHeaderTypeDef RxHeader;
        uint8_t CAN_RX_Data[8];

        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, CAN_RX_Data);

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

//CAN-发送标准帧
void CAN_Send_Data_STD(CAN_Node_HandleTypeDef *hcan_node, const uint8_t *TX_Data)
{
        CAN_Send_Frame_StructTypeDef CAN_Send_Frame_Struct = {};

        CAN_Send_Frame_Struct.hfdcan = hcan_node->hfdcan;

        CAN_Send_Frame_Struct.CAN_TxMsg.Identifier          = hcan_node->CAN_Send_ID;
        CAN_Send_Frame_Struct.CAN_TxMsg.IdType              = FDCAN_STANDARD_ID;
        CAN_Send_Frame_Struct.CAN_TxMsg.TxFrameType         = FDCAN_DATA_FRAME;
        CAN_Send_Frame_Struct.CAN_TxMsg.DataLength          = FDCAN_DLC_BYTES_8;
        CAN_Send_Frame_Struct.CAN_TxMsg.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        CAN_Send_Frame_Struct.CAN_TxMsg.BitRateSwitch       = FDCAN_BRS_OFF;
        CAN_Send_Frame_Struct.CAN_TxMsg.FDFormat            = FDCAN_CLASSIC_CAN;
        CAN_Send_Frame_Struct.CAN_TxMsg.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
        CAN_Send_Frame_Struct.CAN_TxMsg.MessageMarker       = 0;

        memcpy(CAN_Send_Frame_Struct.txData, TX_Data, sizeof(CAN_Send_Frame_Struct.txData));

        CAN_Send_Data_Buffer(&CAN_Send_Frame_Struct);

}

//CAN-发送拓展帧
void CAN_Send_Data_EXD(CAN_Node_HandleTypeDef *hcan_node, uint8_t *TX_Data, uint8_t Length)
{
        CAN_Send_Frame_StructTypeDef CAN_Send_Frame_Struct = {};

        CAN_Send_Frame_Struct.hfdcan = hcan_node->hfdcan;

        CAN_Send_Frame_Struct.CAN_TxMsg.Identifier          = hcan_node->CAN_Send_ID;
        CAN_Send_Frame_Struct.CAN_TxMsg.IdType              = FDCAN_EXTENDED_ID;
        CAN_Send_Frame_Struct.CAN_TxMsg.TxFrameType         = FDCAN_DATA_FRAME;
        CAN_Send_Frame_Struct.CAN_TxMsg.DataLength          = Length;
        CAN_Send_Frame_Struct.CAN_TxMsg.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        CAN_Send_Frame_Struct.CAN_TxMsg.BitRateSwitch       = FDCAN_BRS_OFF;
        CAN_Send_Frame_Struct.CAN_TxMsg.FDFormat            = FDCAN_CLASSIC_CAN;
        CAN_Send_Frame_Struct.CAN_TxMsg.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
        CAN_Send_Frame_Struct.CAN_TxMsg.MessageMarker       = 0;

        memcpy(CAN_Send_Frame_Struct.txData, TX_Data, sizeof(CAN_Send_Frame_Struct.txData));

        CAN_Send_Data_Buffer(&CAN_Send_Frame_Struct);

}


//入队待发: 按帧内 hfdcan 选对应总线缓冲; 不阻塞, 满则丢帧(避免反压控制任务)
void CAN_Send_Data_Buffer(CAN_Send_Frame_StructTypeDef *CAN_Send_Packet_Struct)
{
        CAN_Bus_StructTypeDef *bus = CAN_Bus_Find(CAN_Send_Packet_Struct->hfdcan);
        if (bus == NULL || bus->TxBuffer == NULL) return;

        if (xMessageBufferSend(bus->TxBuffer, CAN_Send_Packet_Struct, sizeof(CAN_Send_Frame_StructTypeDef), 0) == pdFAIL)
        {
                CAN_Tx_Drop_Count++;
        }
}

//TX FIFO 空回调
void HAL_FDCAN_TxFifoEmptyCallback(FDCAN_HandleTypeDef *hfdcan)
{
        CAN_Bus_StructTypeDef *bus = CAN_Bus_Find(hfdcan);
        if (bus == NULL || bus->TxTask == NULL) return;

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(bus->TxTask, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

//单总线发送任务: FIFO 满时只阻塞本总线, 且带超时兜底
static void CAN_TxTask(void *argument)
{
        CAN_Bus_StructTypeDef       *bus   = (CAN_Bus_StructTypeDef *) argument;
        CAN_Send_Frame_StructTypeDef frame = {};

        for (;;)
        {
                if (xMessageBufferReceive(bus->TxBuffer, &frame, sizeof(frame), portMAX_DELAY) != sizeof(frame)) continue;

                //FIFO满: 等本总线空位, 超时说明该总线异常, 不再死等
                if (HAL_FDCAN_GetTxFifoFreeLevel(bus->hfdcan) == 0)
                {
                        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(CAN_TX_TIMEOUT_MS));
                }

                if (HAL_FDCAN_GetTxFifoFreeLevel(bus->hfdcan) == 0)
                {
                        CAN_Tx_Drop_Count++; //该总线卡住, 丢弃本帧
                        continue;
                }

                if (HAL_FDCAN_AddMessageToTxFifoQ(bus->hfdcan, &frame.CAN_TxMsg, frame.txData) != HAL_OK)
                {
                        CAN_Tx_Drop_Count++;
                }
        }
}

//初始化
void CAN_Init()
{
        CAN_Bus_Init(&hfdcan1);         //CAN总线启动 + 过滤器/接收中断
        CAN_Bus_Init(&hfdcan2);
        CAN_Bus_Init(&hfdcan3);
}

void CAN_Bus_Init(FDCAN_HandleTypeDef *hfdcan)
{
        if (CAN_Bus_Count >= CAN_BUS_MAX) return;
        if (CAN_Bus_Find(hfdcan) != NULL) return;

        static const char *const CAN_Tx_TaskName[CAN_BUS_MAX] = {"CAN1_Tx", "CAN2_Tx", "CAN3_Tx"};

        CAN_Bus_StructTypeDef *bus = &CAN_Bus_Table[CAN_Bus_Count];

        bus->hfdcan   = hfdcan;
        bus->TxBuffer = xMessageBufferCreate(CAN_TX_DEPTH * sizeof(CAN_Send_Frame_StructTypeDef));

        xTaskCreate(CAN_TxTask, CAN_Tx_TaskName[CAN_Bus_Count], 256, bus, CAN_TX_TASK_PRIO, &bus->TxTask);

        CAN_Bus_Count++;

        HAL_FDCAN_Start(hfdcan);
        CAN_Filter_Init(hfdcan);
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
                                       FDCAN_IT_TX_FIFO_EMPTY |
                                       FDCAN_IT_ERROR_WARNING |
                                       FDCAN_IT_ERROR_PASSIVE |
                                       FDCAN_IT_BUS_OFF,
                                       0);
}

