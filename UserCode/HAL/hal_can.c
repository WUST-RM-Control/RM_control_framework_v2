//
// Created by ye on 2026/8/22.
//

#include "hal_can.h"

#include <string.h>

QueueHandle_t CAN_TxQueue;

/*===| CAN总线错误处理 |===*/
//总线错误码快照: [0]=CAN1, [1]=CAN2 (仅记录, 供水印/调试观察)
volatile uint32_t CAN_Error_Code[2] = {0, 0};

//CAN错误回调: ABOM=ENABLE 时总线关闭由硬件自动恢复, 这里只留痕
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
        CAN_Error_Code[hcan->Instance == CAN2] = hcan->ErrorCode;
}

//CAN总线重启: 停止→去初始化→重新初始化→启动→重配过滤器/中断 (手动恢复总线错误/总线关闭用)
HAL_StatusTypeDef CAN_Restart(CAN_HandleTypeDef *hcan)
{
        HAL_CAN_Stop(hcan);
        HAL_CAN_DeInit(hcan); //内部会禁用NVIC

        //复用 hcan->Init(MX_CANx_Init 已配置, DeInit 不清空); Init内部会重跑MspInit(重配GPIO与NVIC)
        if (HAL_CAN_Init(hcan) != HAL_OK) return HAL_ERROR;

        CAN_Bus_Init(hcan); //启动 + 过滤器 + 重新激活接收/错误中断
        return HAL_OK;
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

        hcan_node_table[CAN_Node_Count]          = hcan_node;
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

//CAN节点构造: 绑定CAN句柄与收发ID与回调函数
void CAN_Node_Ctor(CAN_Node_HandleTypeDef *hcan_node, CAN_HandleTypeDef *hcan, uint32_t CAN_Send_ID, uint32_t CAN_Feedback_ID, uint32_t CAN_Feedback_ID_Mask, CAN_Node_Handler node_handler, uint16_t err_tick_Timeout, uint16_t err_count_maximum, err_handler err_handler)
{
        Err_Ctor(&hcan_node->herr, err_tick_Timeout, err_count_maximum, err_handler);

        hcan_node->hcan                 = hcan;
        hcan_node->CAN_Send_ID          = CAN_Send_ID;
        hcan_node->CAN_Feedback_ID      = CAN_Feedback_ID;
        hcan_node->CAN_Feedback_ID_Mask = CAN_Feedback_ID_Mask;

        CAN_Node_Register(hcan_node, node_handler);
}

//CAN接收回调
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
        CAN_RxHeaderTypeDef RxHeader;
        uint8_t             CAN_RX_Data[8];

        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, CAN_RX_Data) != HAL_OK) return;

        //标准帧取StdId, 扩展帧取ExtId(bxCAN把标准/扩展ID放在两个不同字段)
        uint32_t CAN_RX_ID = (RxHeader.IDE == CAN_ID_EXT) ? RxHeader.ExtId : RxHeader.StdId;

        //基于can_node的通用分发: 按(总线, 反馈ID)匹配节点, 调用节点数据回调
        for (uint8_t i = 0; i < CAN_Node_Count; i++)
        {
                if (hcan_node_table[i]->hcan == hcan &&
                    hcan_node_table[i]->CAN_Feedback_ID ==( CAN_RX_ID & hcan_node_table[i]->CAN_Feedback_ID_Mask))
                {
                        hcan_node_table[i]->handler(hcan_node_table[i], CAN_RX_Data);
                }
        }
}

//CAN-发送标准帧
void CAN_Send_Data_STD(CAN_Node_HandleTypeDef *hcan_node, const uint8_t *TX_Data)
{
        CAN_Send_Frame_StructTypeDef CAN_Send_Frame_Struct = {};

        CAN_Send_Frame_Struct.hcan = hcan_node->hcan;

        CAN_Send_Frame_Struct.CAN_TxMsg.StdId              = hcan_node->CAN_Send_ID;
        CAN_Send_Frame_Struct.CAN_TxMsg.IDE                = CAN_ID_STD;
        CAN_Send_Frame_Struct.CAN_TxMsg.RTR                = CAN_RTR_DATA;
        CAN_Send_Frame_Struct.CAN_TxMsg.DLC                = 8;
        CAN_Send_Frame_Struct.CAN_TxMsg.TransmitGlobalTime = DISABLE;

        memcpy(CAN_Send_Frame_Struct.txData, TX_Data, sizeof(CAN_Send_Frame_Struct.txData));

        CAN_Send_Data_Queue(&CAN_Send_Frame_Struct);
}

//CAN-发送拓展帧
void CAN_Send_Data_EXD(CAN_Node_HandleTypeDef *hcan_node, uint8_t *TX_Data, uint8_t Length)
{
        CAN_Send_Frame_StructTypeDef CAN_Send_Frame_Struct = {};

        if (Length > 8) Length = 8; //经典帧最大8字节

        CAN_Send_Frame_Struct.hcan = hcan_node->hcan;

        CAN_Send_Frame_Struct.CAN_TxMsg.ExtId              = hcan_node->CAN_Send_ID;
        CAN_Send_Frame_Struct.CAN_TxMsg.IDE                = CAN_ID_EXT;
        CAN_Send_Frame_Struct.CAN_TxMsg.RTR                = CAN_RTR_DATA;
        CAN_Send_Frame_Struct.CAN_TxMsg.DLC                = Length; //经典帧 0~8 字节时 DLC 即字节数
        CAN_Send_Frame_Struct.CAN_TxMsg.TransmitGlobalTime = DISABLE;

        //只取有效长度, 其余保持0(HAL发送时仍会读满8字节缓冲区)
        memcpy(CAN_Send_Frame_Struct.txData, TX_Data, Length);

        CAN_Send_Data_Queue(&CAN_Send_Frame_Struct);
}

static void CAN_TxTask(void *argument)
{
        CAN_Send_Frame_StructTypeDef CAN_Send_Frame_Struct = {};
        uint32_t                     mailbox               = 0;

        for (;;)
        {
                if (xQueueReceive(CAN_TxQueue, &CAN_Send_Frame_Struct, portMAX_DELAY) == pdPASS)
                {
                        //3个邮箱全满则让出CPU等待(不丢帧)
                        while (HAL_CAN_GetTxMailboxesFreeLevel(CAN_Send_Frame_Struct.hcan) == 0) vTaskDelay(1);

                        HAL_CAN_AddTxMessage(CAN_Send_Frame_Struct.hcan, &CAN_Send_Frame_Struct.CAN_TxMsg,
                                             CAN_Send_Frame_Struct.txData, &mailbox);
                }
        }
}

//初始化
void CAN_Init()
{
        //软件缓冲队列
        CAN_TxQueue = xQueueCreate(32, sizeof(CAN_Send_Frame_StructTypeDef));

        xTaskCreate(CAN_TxTask, "CAN_Tx", 256, NULL, 10, NULL);

        CAN_Bus_Init(&hcan1); //CAN总线启动 + 过滤器/接收中断
        CAN_Bus_Init(&hcan2);
}

void CAN_Bus_Init(CAN_HandleTypeDef *hcan)
{
        HAL_CAN_Start(hcan);
        CAN_Filter_Init(hcan);
}

//STM32F407 双bxCAN共享28个过滤bank: CAN1用0~13, CAN2用14~27
#define CAN2_START_FILTER_BANK 14

//CAN过滤器初始化: CAN1→bank0, CAN2→bank14, 全部放行(标准帧+扩展帧)→FIFO0
void CAN_Filter_Init(CAN_HandleTypeDef *hcan)
{
        CAN_FilterTypeDef sFilterConfig = {0};

        sFilterConfig.FilterIdHigh         = 0x0000;
        sFilterConfig.FilterIdLow          = 0x0000;
        sFilterConfig.FilterMaskIdHigh     = 0x0000; //mask=0 → 全部通过(软件层按ID分发)
        sFilterConfig.FilterMaskIdLow      = 0x0000;
        sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
        sFilterConfig.FilterBank           = (hcan->Instance == CAN2) ? CAN2_START_FILTER_BANK : 0;
        sFilterConfig.FilterMode           = CAN_FILTERMODE_IDMASK;
        sFilterConfig.FilterScale          = CAN_FILTERSCALE_32BIT;
        sFilterConfig.FilterActivation     = ENABLE;
        //该字段实际写的是CAN1的FMR.CAN2SB(CAN2的过滤寄存器只能从CAN1侧访问), 两路必须填一致
        sFilterConfig.SlaveStartFilterBank = CAN2_START_FILTER_BANK;

        HAL_CAN_ConfigFilter(hcan, &sFilterConfig);

        //中断
        HAL_CAN_ActivateNotification(hcan,
                                     CAN_IT_RX_FIFO0_MSG_PENDING |
                                     CAN_IT_ERROR_WARNING |
                                     CAN_IT_ERROR_PASSIVE |
                                     CAN_IT_BUSOFF |
                                     CAN_IT_ERROR);
}
