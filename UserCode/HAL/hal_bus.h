//
// Created by ye on 2026/8/29.
//

#ifndef G4MINI_V3_HAL_BUS_H
#define G4MINI_V3_HAL_BUS_H

#include "err.h"
#include <stdint.h>

typedef enum
{
        BUS_TYPE_CAN = 0,
        BUS_TYPE_UART
} Bus_Type_EnumTypeDef;

typedef struct Bus_Node_HandleTypeDef Bus_Node_HandleTypeDef;

typedef void (*Bus_Node_Handler)(Bus_Node_HandleTypeDef *bus_node);

/*===| 总线节点基类: CAN/UART 节点把它作为第一成员实现"继承" |===*/
struct Bus_Node_HandleTypeDef
{
        Err_HandleTypeDef herr;

        Bus_Type_EnumTypeDef Bus_Type;
        uint32_t             Device_ID;
        Bus_Node_Handler     handler;

        uint8_t *Rx_Data;
        uint16_t Rx_Length;
};

void Bus_Node_Ctor(Bus_Node_HandleTypeDef *bus_node, Bus_Type_EnumTypeDef Bus_Type, uint32_t Device_ID, Bus_Node_Handler handler, uint16_t err_tick_timeout, uint16_t err_count_maximum, err_handler err_handler);

__STATIC_INLINE uint8_t *Bus_Node_GetRxData(Bus_Node_HandleTypeDef *bus_node) { return bus_node->Rx_Data; }

__STATIC_INLINE uint16_t Bus_Node_GetRxLength(Bus_Node_HandleTypeDef *bus_node) { return bus_node->Rx_Length; }

__STATIC_INLINE void Bus_Node_SetHandler(Bus_Node_HandleTypeDef *bus_node, Bus_Node_Handler handler) { bus_node->handler = handler; }

#endif //G4MINI_V3_HAL_BUS_H
