//
// Created by ye on 2026/8/22.
//

#ifndef G4MINI_V3_DRV_MOTOR_ZDT_EMM_V5_H
#define G4MINI_V3_DRV_MOTOR_ZDT_EMM_V5_H

#include "drv_motor.h"

typedef struct
{
        Motor_Base_HandleTypeDef hmotor_base;

        uint16_t Poll_Count;
        float    Last_Target;
        uint8_t  Last_Status;
} Motor_ZDT_HandleTypeDef;

typedef struct
{
        Motor_Config_StructTypeDef Base;

        uint16_t Pulse_Per_Rev; //每圈脉冲数(对应驱动细分, 16细分=3200)
        uint16_t Default_Vel;   //默认速度[RPM]
        uint8_t  Default_Acc;   //默认加速度(0=直接启动)
} Motor_ZDT_Config_StructTypeDef;

extern Motor_VTable                   Motor_ZDT_VTable_Default;
extern Motor_ZDT_Config_StructTypeDef Motor_ZDT_Config_Default;

void Motor_ZDT_Ctor(Motor_ZDT_HandleTypeDef *hmotor_zdt, UART_HandleTypeDef *huart, uint8_t Addr);

#endif //G4MINI_V3_DRV_MOTOR_ZDT_EMM_V5_H
