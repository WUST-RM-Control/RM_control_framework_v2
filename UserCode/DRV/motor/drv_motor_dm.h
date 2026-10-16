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

        Feedforward_t FFC_Angle_Struct;
        PID_t         PID_Angle_Struct;
        Feedforward_t FFC_Speed_Struct;
        PID_t         PID_Speed_Struct;
} Motor_DM_HandleTypeDef;

typedef struct
{
        Motor_Enum_TypeDef Motor_Enum;

        Err_HandleTypeDef herr;

        uint32_t CAN_Feedback_ID_Mask; //反馈ID掩码

        Motor_VTable *vptr;

        uint8_t Error_Code; //反馈错误码(1=正常, 0=失能, 3~E=故障; DJI不更新恒为1)

        Motor_Status_TypeDef Status_Enum;
} Motor_DM_Config_StructTypeDef;

void Motor_DM_CMD_MIT(Motor_Base_HandleTypeDef *hmotor_base, float _pos, float _vel, float _KP, float _KD, float _torq);

void Motor_DM_CMD_Position(Motor_Base_HandleTypeDef *hmotor_base, float Position, float Speed);

__STATIC_INLINE void Motor_DM_Send_Torque(Motor_Base_HandleTypeDef *hmotor, float torque) { Motor_DM_CMD_MIT(hmotor, 0, 0, 0, 0, torque); }
__STATIC_INLINE void Motor_DM_Send_Pos(Motor_Base_HandleTypeDef *hmotor, float Position) { Motor_DM_CMD_Position(hmotor, Position, 1); } //待测试

extern Motor_VTable Motor_DM_VTable_Default;


#endif //G4MINI_V3_DRV_MOTOR_DM_H
