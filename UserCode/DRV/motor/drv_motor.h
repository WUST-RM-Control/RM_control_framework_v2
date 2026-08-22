//
// Created by ye on 2026/8/22.
//

#ifndef G4MINI_V3_DRV_MOTOR_H
#define G4MINI_V3_DRV_MOTOR_H

#include "controller.h"
#include "main.h"

typedef enum
{
        MOTOR_TORQUE = 0,
        MOTOR_SPEED,
        MOTOR_ANGLE
} Motor_Status_TypeDef;


typedef struct Motor_HandleTypeDef Motor_HandleTypeDef;
typedef struct Motor_VTable Motor_VTable;

struct Motor_VTable {
        void (*set_Torque) (Motor_HandleTypeDef *hmotor);
        float (*get_Total_Angle) (Motor_HandleTypeDef *hmotor);
        float (*get_Speed) (Motor_HandleTypeDef *hmotor);
};

/*===| 电机数据结构体定义 |===*/
struct Motor_HandleTypeDef
{
        Motor_VTable *vptr;

        FDCAN_HandleTypeDef *hfdcan;
        uint16_t CAN_ID;

        Motor_Status_TypeDef Status_Enum;

        Feedforward_t FFC_angle_Struct;
        PID_t         PID_angle_Struct;
        Feedforward_t FFC_speed_Struct;
        PID_t         PID_speed_Struct;

        int16_t  Encoder;               //编码器值
        float    Angle;                 //绝对角度
        float    Total_Angle;           //总角度值
        float    Total_Angle_Speed_RPM; //总角度值速度

        float    Total_Angle_Offset;    //总角度值零点
        int32_t  Round;                 //圈数

        int16_t  Encoder_Last;               //上一个编码器值
        float    Angle_Last;                 //上一个绝对角度
        float    Total_Angle_Last;           //上一个总角度值
        float    Total_Angle_Speed_RPM_Last; //上一个总角度值速度

        uint32_t Total_Angle_DWT_Count;

        float    Speed_RPM;   //转速[RPM]
        float    Torque;      //力矩[电流]
        int8_t   Temperature; //电机温度

        uint16_t Ticker;    //收到数据计时
        uint8_t  If_Online; //是否在线
};




void Motor_Get_TotalAngle_Speed(Motor_HandleTypeDef *Motor_Data_Struct, float K);


#endif //G4MINI_V3_DRV_MOTOR_H
