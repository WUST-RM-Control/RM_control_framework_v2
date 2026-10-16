//
// Created by ye on 2026/8/29.
//

#include "hal_bus.h"

void Bus_Node_Ctor(Bus_Node_HandleTypeDef *bus_node, Bus_Type_EnumTypeDef Bus_Type, uint32_t Device_ID, Bus_Node_Handler handler, uint16_t err_tick_timeout, uint16_t err_count_maximum, err_handler err_handler)
{
        Err_Ctor(&bus_node->herr, err_tick_timeout, err_count_maximum, err_handler);

        bus_node->Bus_Type  = Bus_Type;
        bus_node->Device_ID = Device_ID;
        bus_node->handler   = handler;

        bus_node->Rx_Data   = NULL;
        bus_node->Rx_Length = 0;
}
