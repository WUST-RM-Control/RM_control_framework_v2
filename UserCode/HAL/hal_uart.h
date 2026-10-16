//
// Created by ye on 2026/8/29.
//

#ifndef G4MINI_V3_HAL_UART_H
#define G4MINI_V3_HAL_UART_H

#include "hal_bus.h"
#include "stm32f4xx_hal.h"

#define UART_NODE_MAX    8
#define UART_NODE_RX_MAX 32

typedef struct UART_Node_HandleTypeDef UART_Node_HandleTypeDef;

/*===| UART节点: 继承 Bus_Node(第一成员) |===*/
struct UART_Node_HandleTypeDef
{
        Bus_Node_HandleTypeDef Base;

        UART_HandleTypeDef *huart;

        uint8_t  rxData[UART_NODE_RX_MAX];
        uint16_t rxLength;
};

void UART_Node_Ctor(UART_Node_HandleTypeDef *uart_node, UART_HandleTypeDef *huart, uint8_t Addr, Bus_Node_Handler handler, uint16_t err_tick_timeout, uint16_t err_count_maximum, err_handler err_handler);

void UART_Node_Register(UART_Node_HandleTypeDef *uart_node);

void UART_Node_UnRegister(UART_Node_HandleTypeDef *uart_node);

//阻塞发送(初始化/小数据用)
HAL_StatusTypeDef UART_Node_Transmit(UART_Node_HandleTypeDef *uart_node, const uint8_t *TX_Data, uint16_t Length);

//队列发送(实时路径用, 不阻塞控制循环)
void UART_Node_Transmit_Queue(UART_Node_HandleTypeDef *uart_node, const uint8_t *TX_Data, uint16_t Length);

//单条UART总线初始化: 建队列/任务 + 空闲DMA接收 + 错误重启
void UART_Bus_Init(UART_HandleTypeDef *huart);

#endif //G4MINI_V3_HAL_UART_H
