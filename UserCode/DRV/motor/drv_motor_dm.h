//
// Created by ye on 2026/8/22.
//

#ifndef G4MINI_V3_DRV_MOTOR_DM_H
#define G4MINI_V3_DRV_MOTOR_DM_H

#include "drv_motor.h"
#include "main.h"

/*===| 达妙相关驱动 |===*/
#define P_MIN   -3.1415926f
#define P_MAX   3.1415926f
#define V_MIN   -30
#define V_MAX   30
#define KP_MIN  0
#define KP_MAX  500
#define KD_MIN  0
#define KD_MAX  5
#define T_MIN   -16384
#define T_MAX   16384

typedef struct Motor_DM_HandleTypeDef Motor_DM_HandleTypeDef;
typedef struct Motor_DM_VTable Motor_DM_VTable;



void Motor_DM_Ctor(Motor_HandleTypeDef *self,FDCAN_HandleTypeDef *hfdcan, uint16_t CAN_ID);



#endif //G4MINI_V3_DRV_MOTOR_DM_H
