//
// Created by ye on 2026/8/26.
//

#ifndef G4MINI_V3_HAL_DEBUG_H
#define G4MINI_V3_HAL_DEBUG_H

#include "stm32g4xx_hal.h"

//调试串口对象构造: 绑定调试串口(实例 ↔ 外设绑定, 由 ENT 调用)
void Debug_Ctor(UART_HandleTypeDef *huart);

//调试串口初始化(printf重定向到该串口, 115200)
//内部注册: UART错误回调(per-handle)
void Debug_Init(void);

//获取串口错误计数
uint32_t Debug_Get_Error_Count(void);

#endif //G4MINI_V3_HAL_DEBUG_H
