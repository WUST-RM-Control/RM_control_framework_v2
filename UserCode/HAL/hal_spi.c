//
// Created by ye on 2026/10/5.
//

#include "hal_spi.h"

/*===| SPI节点分发框架(基于 spi_node 基类) |===*/
#define SPI_NODE_MAX 8

static SPI_Node_HandleTypeDef *hspi_node_table[SPI_NODE_MAX];
static uint8_t                 SPI_Node_Count = 0;

//注册节点(重复注册则更新回调; 允许 handler 为空 → 节点仍入表, 只是完成回调不分发)
void SPI_Node_Register(SPI_Node_HandleTypeDef *hspi_node, SPI_Node_Handler handler)
{
        if (hspi_node == NULL) return;

        for (uint8_t i = 0; i < SPI_Node_Count; i++)
        {
                if (hspi_node_table[i] == hspi_node)
                {
                        hspi_node_table[i]->handler = handler;
                        return;
                }
        }

        if (SPI_Node_Count >= SPI_NODE_MAX) return;

        hspi_node_table[SPI_Node_Count]          = hspi_node;
        hspi_node_table[SPI_Node_Count]->handler = handler;
        SPI_Node_Count++;
}

//注销节点
void SPI_Node_UnRegister(SPI_Node_HandleTypeDef *hspi_node)
{
        for (uint8_t i = 0; i < SPI_Node_Count; i++)
        {
                if (hspi_node_table[i] == hspi_node)
                {
                        SPI_Node_Count--;
                        hspi_node_table[i] = hspi_node_table[SPI_Node_Count];
                        return;
                }
        }
}

//SPI节点构造: 绑定SPI句柄与片选引脚与完成回调
void SPI_Node_Ctor(SPI_Node_HandleTypeDef *hspi_node, SPI_HandleTypeDef *hspi, GPIO_TypeDef *Port, uint16_t Pin, SPI_Node_Handler node_handler)
{
        if (hspi_node == NULL) return;

        hspi_node->hspi = hspi;
        hspi_node->Port = Port;
        hspi_node->Pin  = Pin;

        hspi_node->txData = NULL;
        hspi_node->rxData = NULL;
        hspi_node->Length = 0;

        //初始取消选中
        SPI_Node_CS_High(hspi_node);

        SPI_Node_Register(hspi_node, node_handler);
}

//查找该总线上正在传输的节点(SPI是共享总线, 同一条总线上同时最多一个节点在传)
static SPI_Node_HandleTypeDef *SPI_Node_FindBusy(SPI_HandleTypeDef *hspi)
{
        for (uint8_t i = 0; i < SPI_Node_Count; i++)
        {
                if (hspi_node_table[i]->hspi == hspi && hspi_node_table[i]->Length != 0)
                {
                        return hspi_node_table[i];
                }
        }

        return NULL;
}

//阻塞收发: 片选拉低→收发→片选拉高
HAL_StatusTypeDef SPI_Node_TransmitReceive(SPI_Node_HandleTypeDef *hspi_node, uint8_t *TX_Data, uint8_t *RX_Data, uint16_t Length)
{
        HAL_StatusTypeDef Status;

        if (hspi_node == NULL || hspi_node->hspi == NULL) return HAL_ERROR;

        SPI_Node_CS_Low(hspi_node);
        Status = HAL_SPI_TransmitReceive(hspi_node->hspi, TX_Data, RX_Data, Length, SPI_NODE_TIMEOUT);
        SPI_Node_CS_High(hspi_node);

        return Status;
}

//阻塞单字节收发
uint8_t SPI_Node_ReadWrite_Byte(SPI_Node_HandleTypeDef *hspi_node, uint8_t TX_Data)
{
        uint8_t RX_Data = 0;

        SPI_Node_TransmitReceive(hspi_node, &TX_Data, &RX_Data, 1);

        return RX_Data;
}

//中断/DMA 收发前的公共准备: 记录传输上下文 + 拉低片选
static HAL_StatusTypeDef SPI_Node_TransmitReceive_Start(SPI_Node_HandleTypeDef *hspi_node, uint8_t *TX_Data, uint8_t *RX_Data, uint16_t Length, bool Is_DMA)
{
        SPI_Node_HandleTypeDef *busy;

        if (hspi_node == NULL || hspi_node->hspi == NULL || Length == 0) return HAL_ERROR;

        busy = SPI_Node_FindBusy(hspi_node->hspi);
        if (busy != NULL)
        {
                //兜底: HAL已回到READY说明上一笔其实已结束(没走完成回调), 清掉残留占用, 避免总线被永久锁死
                if (HAL_SPI_GetState(hspi_node->hspi) != HAL_SPI_STATE_READY) return HAL_BUSY;

                SPI_Node_CS_High(busy);
                busy->Length = 0;
        }

        hspi_node->txData = TX_Data;
        hspi_node->rxData = RX_Data;
        hspi_node->Length = Length;

        SPI_Node_CS_Low(hspi_node);

        if ((Is_DMA ? HAL_SPI_TransmitReceive_DMA(hspi_node->hspi, TX_Data, RX_Data, Length)
                    : HAL_SPI_TransmitReceive_IT(hspi_node->hspi, TX_Data, RX_Data, Length)) != HAL_OK)
        {
                SPI_Node_CS_High(hspi_node);
                hspi_node->Length = 0;
                return HAL_ERROR;
        }

        return HAL_OK;
}

//中断收发
HAL_StatusTypeDef SPI_Node_TransmitReceive_IT(SPI_Node_HandleTypeDef *hspi_node, uint8_t *TX_Data, uint8_t *RX_Data, uint16_t Length)
{
        return SPI_Node_TransmitReceive_Start(hspi_node, TX_Data, RX_Data, Length, false);
}

//DMA收发
HAL_StatusTypeDef SPI_Node_TransmitReceive_DMA(SPI_Node_HandleTypeDef *hspi_node, uint8_t *TX_Data, uint8_t *RX_Data, uint16_t Length)
{
        return SPI_Node_TransmitReceive_Start(hspi_node, TX_Data, RX_Data, Length, true);
}

//传输完成回调(IT与DMA共用): 拉高片选 → 交给节点handler
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
        SPI_Node_HandleTypeDef *hspi_node = SPI_Node_FindBusy(hspi);

        if (hspi_node == NULL) return;

        SPI_Node_CS_High(hspi_node);

        hspi_node->Length = 0;

        if (hspi_node->handler != NULL) hspi_node->handler(hspi_node);
}

/*===| 总线级初始化 |===*/
void SPI_Init()
{
        SPI_Bus_Init(&hspi1);
}

//单条SPI总线初始化: 取消选中挂在该总线上的所有节点
void SPI_Bus_Init(SPI_HandleTypeDef *hspi)
{
        //GPIO/DMA/NVIC 已在 MspInit 配置, 这里只把该总线上的节点全部取消选中
        for (uint8_t i = 0; i < SPI_Node_Count; i++)
        {
                if (hspi_node_table[i]->hspi == hspi) SPI_Node_CS_High(hspi_node_table[i]);
        }
}
