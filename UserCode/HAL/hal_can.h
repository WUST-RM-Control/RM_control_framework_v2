//
// Created by ye on 2026/8/22.
//

#ifndef G4MINI_V3_HAL_CAN_H
#define G4MINI_V3_HAL_CAN_H

#include "stm32g4xx_hal.h"

void CAN_Send_Data_STD(FDCAN_HandleTypeDef *hfdcan, uint16_t ID, uint8_t *TX_Data);
void CAN_Send_Data_EXD(FDCAN_HandleTypeDef *hfdcan, uint32_t ID, uint8_t *TX_Data, uint8_t Length);
void CAN_Filter_Init(FDCAN_HandleTypeDef *hfdcan);
void CAN_Init(void);

#endif //G4MINI_V3_HAL_CAN_H
