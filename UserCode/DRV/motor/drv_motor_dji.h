//
// Created by ye on 2026/8/22.
//

#ifndef G4MINI_V3_DRV_MOTOR_DJI_H
#define G4MINI_V3_DRV_MOTOR_DJI_H
#include <stdint.h>

#include "drv_motor.h"
void Motor_DJI_SendCurrent(FDCAN_HandleTypeDef *hfdcan, uint16_t CAN_ID, int16_t ID1_Current, int16_t ID2_Current, int16_t ID3_Current, int16_t ID4_Current);

void Motor_DJI_Storage_Data(Motor_HandleTypeDef *DJI_Motor_Data_Struct, const uint8_t *Data);

void Motor_DJI_Ctor(Motor_HandleTypeDef *hmotor,FDCAN_HandleTypeDef *hfdcan, uint16_t CAN_Send_ID, uint16_t CAN_Feedback_ID);

#endif //G4MINI_V3_DRV_MOTOR_DJI_H
