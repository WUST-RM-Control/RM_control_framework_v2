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

struct Motor_DM_VTable
{
        void (*enable)     (Motor_DM_HandleTypeDef *hdmmotor);
        void (*disable)    (Motor_DM_HandleTypeDef *hdmmotor);
        void (*clear_Err)  (Motor_DM_HandleTypeDef *hdmmotor);
        void (*set_Zero)   (Motor_DM_HandleTypeDef *hdmmotor);
        void (*MIT)        (Motor_DM_HandleTypeDef *hdmmotor, float _pos, float _vel, float _KP, float _KD, float _torq);
        void (*Position)   (Motor_DM_HandleTypeDef *hdmmotor, float Position, float Speed);
        void (*Speed)      (Motor_DM_HandleTypeDef *hdmmotor, float Speed);
        void (*Storage)    (uint8_t *Data, Motor_DM_HandleTypeDef *hdmmotor);
};

struct Motor_DM_HandleTypeDef
{
        Motor_HandleTypeDef hmotor;
        Motor_DM_VTable    *vptr;

        FDCAN_HandleTypeDef *hfdcan;
        uint16_t             CAN_ID;
};

extern const Motor_DM_VTable Motor_DM_VTable_Default;

void Motor_DM_Init(Motor_DM_HandleTypeDef *hdmmotor, FDCAN_HandleTypeDef *hfdcan, uint16_t CAN_ID);

void Motor_DM_Enable     (Motor_DM_HandleTypeDef *hdmmotor);
void Motor_DM_Disable    (Motor_DM_HandleTypeDef *hdmmotor);
void Motor_DM_SetZero    (Motor_DM_HandleTypeDef *hdmmotor);
void Motor_DM_ClearErr   (Motor_DM_HandleTypeDef *hdmmotor);
void Motor_DM_MIT        (Motor_DM_HandleTypeDef *hdmmotor, float _pos, float _vel, float _KP, float _KD, float _torq);
void Motor_DM_Position   (Motor_DM_HandleTypeDef *hdmmotor, float Position, float Speed);
void Motor_DM_Speed      (Motor_DM_HandleTypeDef *hdmmotor, float Speed);
void Motor_DM_Storage    (uint8_t *Data, Motor_DM_HandleTypeDef *hdmmotor);

void Motor_DM1to4_Storage_Data(const uint8_t *Data, Motor_HandleTypeDef *Motor_Data_Struct);
void Motor_DM1to4_SendCurrent(FDCAN_HandleTypeDef *hfdcan, uint16_t CAN_ID, int16_t ID1_Current, int16_t ID2_Current, int16_t ID3_Current, int16_t ID4_Current);
void Motor_DM1to4_ClearErr(FDCAN_HandleTypeDef *hfdcan, uint16_t CAN_ID);

#endif //G4MINI_V3_DRV_MOTOR_DM_H
