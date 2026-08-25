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

void Motor_DM_CMD_MIT(Motor_HandleTypeDef *hmotor, float _pos, float _vel, float _KP, float _KD, float _torq);

void Motor_DM_Ctor(Motor_HandleTypeDef *hmotor,FDCAN_HandleTypeDef *hfdcan, uint16_t CAN_Send_ID, uint16_t CAN_Feedback_ID);

__STATIC_INLINE void Motor_DM_Send_Torque(Motor_HandleTypeDef *hmotor, float torque)
{
        Motor_DM_CMD_MIT(hmotor, 0, 0, 0, 0, torque);
}



#endif //G4MINI_V3_DRV_MOTOR_DM_H
