//
// Created by ye on 2026/10/5.
//

#ifndef G4MINI_V3_HAL_SPI_H
#define G4MINI_V3_HAL_SPI_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include "spi.h"

/*===| SPI总线资源分配(各从设备使用的总线与片选, 使用前需包含spi.h) |===*/

//超时(ms): 阻塞收发用
#define SPI_NODE_TIMEOUT 100

typedef struct SPI_Node_HandleTypeDef SPI_Node_HandleTypeDef;

typedef void (*SPI_Node_Handler)(SPI_Node_HandleTypeDef *hspi_node);

/*===| SPI节点基类(所有SPI从设备的公共字段)
 * 一个SPI从机(一条CS) = 一个节点, 节点自带片选 → 驱动里不出现引脚宏
 * 不做错误处理: 节点不带herr, 不注册进错误监控(掉线靠驱动自身返回值体现)
 |===*/
struct SPI_Node_HandleTypeDef
{
        SPI_HandleTypeDef *hspi; //SPI句柄

        GPIO_TypeDef      *Port; //片选端口
        uint16_t           Pin;  //片选引脚

        SPI_Node_Handler handler; //传输完成回调(IT/DMA模式)

        //当前传输上下文(IT/DMA模式用; 缓冲需保持有效到完成回调)
        uint8_t *txData;
        uint8_t *rxData;
        uint16_t Length;
};

/*===| 基类构造与访问接口 |===*/
//SPI节点构造: 绑定SPI句柄与片选引脚(重复构造则更新, 参考 CAN_Node_Ctor)
void SPI_Node_Ctor(SPI_Node_HandleTypeDef *hspi_node, SPI_HandleTypeDef *hspi, GPIO_TypeDef *Port, uint16_t Pin, SPI_Node_Handler node_handler);

__STATIC_INLINE SPI_HandleTypeDef *SPI_Node_GetHSPI(SPI_Node_HandleTypeDef *node)
{
        return node->hspi;
}

/*===| 片选控制(一般不用直接调, 由 SPI_Node_TransmitReceive* 统一管理) |===*/
//Port 为空时(板子未绑定片选)只跳过片选动作, 避免空指针
__STATIC_INLINE void SPI_Node_CS_Low(SPI_Node_HandleTypeDef *node)
{
        if (node->Port != NULL) HAL_GPIO_WritePin(node->Port, node->Pin, GPIO_PIN_RESET);
}

__STATIC_INLINE void SPI_Node_CS_High(SPI_Node_HandleTypeDef *node)
{
        if (node->Port != NULL) HAL_GPIO_WritePin(node->Port, node->Pin, GPIO_PIN_SET);
}

/*===| SPI节点分发框架 |===*/
//注册节点(重复注册则更新回调)
void SPI_Node_Register(SPI_Node_HandleTypeDef *hspi_node, SPI_Node_Handler handler);

//注销节点
void SPI_Node_UnRegister(SPI_Node_HandleTypeDef *hspi_node);

/*===| 统一收发(自动片选) |===*/
//阻塞收发: 片选拉低→收发→片选拉高
HAL_StatusTypeDef SPI_Node_TransmitReceive(SPI_Node_HandleTypeDef *hspi_node, uint8_t *TX_Data, uint8_t *RX_Data, uint16_t Length);

//阻塞单字节收发
uint8_t SPI_Node_ReadWrite_Byte(SPI_Node_HandleTypeDef *hspi_node, uint8_t TX_Data);

//中断收发: 发完在 HAL_SPI_TxRxCpltCallback 里拉高片选并调用节点handler
HAL_StatusTypeDef SPI_Node_TransmitReceive_IT(SPI_Node_HandleTypeDef *hspi_node, uint8_t *TX_Data, uint8_t *RX_Data, uint16_t Length);

//DMA收发: 与IT共用完成回调, 缓冲需保持有效到完成
HAL_StatusTypeDef SPI_Node_TransmitReceive_DMA(SPI_Node_HandleTypeDef *hspi_node, uint8_t *TX_Data, uint8_t *RX_Data, uint16_t Length);

/*===| 总线级初始化 |===*/
void SPI_Init();

//单条SPI总线初始化: 取消选中挂在该总线上的所有节点
void SPI_Bus_Init(SPI_HandleTypeDef *hspi);

#endif //G4MINI_V3_HAL_SPI_H
