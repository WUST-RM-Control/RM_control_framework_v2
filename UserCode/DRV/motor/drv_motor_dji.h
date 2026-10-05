//
// Created by ye on 2026/8/22.
//

#ifndef G4MINI_V3_DRV_MOTOR_DJI_H
#define G4MINI_V3_DRV_MOTOR_DJI_H
#include <stdint.h>

#include "drv_motor.h"
void Motor_DJI_SendCurrent(CAN_HandleTypeDef *hcan, uint16_t CAN_ID, int16_t ID1_Current, int16_t ID2_Current, int16_t ID3_Current, int16_t ID4_Current);

//单电机力矩发送: DJI一拖四, 实际由组长一次性组帧(组内其余成员调用为空操作)
//需先用 Motor_Group_Init 建组, 否则不发送
void Motor_DJI_Send_Torque(Motor_HandleTypeDef *hmotor, float Torque);

void Motor_DJI_Storage_Data(Motor_HandleTypeDef *DJI_Motor_Data_Struct, const uint8_t *Data);

extern Motor_VTable Motor_DJI_VTable_Default;
extern Motor_Config_StructTypeDef Motor_DJI_Config_Default;

__STATIC_INLINE void Motor_DJI_Set_Zero(Motor_HandleTypeDef *hmotor) { hmotor->Total_Angle_Offset = hmotor->Total_Angle; }

#endif //G4MINI_V3_DRV_MOTOR_DJI_H
