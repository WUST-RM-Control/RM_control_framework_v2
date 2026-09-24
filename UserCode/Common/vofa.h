//
// Created by ye on 2026/8/29.
//

#ifndef G4MINI_V3_VOFA_H
#define G4MINI_V3_VOFA_H

#include <string.h>

#include "hal_dwt.h"
#include "stm32g4xx_hal.h"

//VOFA对象构造: 绑定调试串口(实例 ↔ 外设绑定, 由 ENT 调用)
void VOFA_Ctor(UART_HandleTypeDef *huart);

void VOFA_Init(void);

void VOFA_Send_Data(uint8_t channel, float data);

//获取串口错误计数
uint32_t VOFA_Get_Error_Count(void);

#endif //G4MINI_V3_VOFA_H
