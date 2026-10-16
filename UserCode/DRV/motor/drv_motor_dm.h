//
// Created by ye on 2026/8/22.
//

#ifndef G4MINI_V3_DRV_MOTOR_DM_H
#define G4MINI_V3_DRV_MOTOR_DM_H

#include "drv_motor.h"
#include "main.h"

/*===| 达妙相关驱动 |===*/
#define P_MIN   (-3.1415926f)
#define P_MAX   3.1415926f
#define V_MIN   (-30)
#define V_MAX   30
#define KP_MIN  0
#define KP_MAX  500
#define KD_MIN  0
#define KD_MAX  5
#define T_MIN   (-16384)
#define T_MAX   16384

typedef struct
{
        Motor_Base_HandleTypeDef hmotor_base;

        Motor_Loop_CtrlTypeDef Ctrl_Angle;
        Motor_Loop_CtrlTypeDef Ctrl_Speed;
} Motor_DM_HandleTypeDef;

typedef struct
{
        Motor_Config_StructTypeDef Base;

        Motor_PID_ParamTypeDef PID_Angle;
        Motor_PID_ParamTypeDef PID_Speed;

        Motor_FFC_ParamTypeDef FFC_Angle;
        Motor_FFC_ParamTypeDef FFC_Speed;
} Motor_DM_Config_StructTypeDef;

extern Motor_VTable                  Motor_DM_VTable_Default;
extern Motor_DM_Config_StructTypeDef Motor_DM_Config_Default;

void Motor_DM_Ctor(Motor_DM_HandleTypeDef *hmotor_dm, CAN_HandleTypeDef *hcan, uint32_t CAN_Send_ID, uint32_t CAN_Feedback_ID);

void Motor_DM_CMD_MIT(Motor_Base_HandleTypeDef *hmotor_base, float _pos, float _vel, float _KP, float _KD, float _torq);

void Motor_DM_CMD_Speed(Motor_Base_HandleTypeDef *hmotor_base, float Speed);

#endif //G4MINI_V3_DRV_MOTOR_DM_H
