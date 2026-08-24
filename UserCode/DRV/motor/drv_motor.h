//
// Created by ye on 2026/8/22.
//

#ifndef G4MINI_V3_DRV_MOTOR_H
#define G4MINI_V3_DRV_MOTOR_H

#include "controller.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

typedef enum
{
        MOTOR_TORQUE = 0,
        MOTOR_SPEED,
        MOTOR_ANGLE
} Motor_Status_TypeDef;


typedef struct Motor_HandleTypeDef Motor_HandleTypeDef;
typedef struct Motor_VTable Motor_VTable;

struct Motor_VTable {

        void (*enable)(Motor_HandleTypeDef *hmotor);

        void (*disable)(Motor_HandleTypeDef *hmotor);

        void (*storage_data)(Motor_HandleTypeDef *hmotor, const uint8_t *Data);
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

        volatile float Target_Torque; //目标力矩[电流]
        volatile float Target_Speed;  //目标速度
        volatile float Target_Angle;  //目标角度

        int16_t  Encoder;               //编码器值
        float    Angle;                 //绝对角度
        float    Total_Angle;           //总角度值
        float    Total_Angle_Speed;     //总角度值速度
        float    Speed;                 //转速[RPM]
        int8_t   Temperature;           //电机温度

        float    Total_Angle_Offset;    //总角度值零点
        int32_t  Round;                 //圈数

        int16_t  Encoder_Last;               //上一个编码器值
        float    Angle_Last;                 //上一个绝对角度
        float    Total_Angle_Last;           //上一个总角度值
        float    Total_Angle_Speed_RPM_Last; //上一个总角度值速度

        uint32_t Total_Angle_DWT_Count;

        uint16_t Ticker;    //收到数据计时
        uint8_t  If_Online; //是否在线
};

void Motor_Control_Task(void *pvParameters);

void Motor_Get_TotalAngle_Speed(Motor_HandleTypeDef *hmotor, float K);

__STATIC_INLINE void Motor_Enable(Motor_HandleTypeDef *hmotor)
{
        hmotor->vptr->enable(hmotor);
}

__STATIC_INLINE void Motor_Disable(Motor_HandleTypeDef *hmotor)
{
        hmotor->vptr->disable(hmotor);
}

__STATIC_INLINE void Motor_Storage_Data(Motor_HandleTypeDef *hmotor, const uint8_t *data)
{
        hmotor->vptr->storage_data(hmotor, data);
}

__STATIC_INLINE void Motor_Set_Torque(Motor_HandleTypeDef *hmotor, float torque)
{
        hmotor->Target_Torque = torque;
}

__STATIC_INLINE void Motor_Set_Speed(Motor_HandleTypeDef *hmotor, float speed)
{
        hmotor->Target_Speed = speed;
}

__STATIC_INLINE void Motor_Set_Angle(Motor_HandleTypeDef *hmotor, float angle)
{
        hmotor->Target_Angle = angle;
}

__STATIC_INLINE float Motor_Get_Target_Torque(Motor_HandleTypeDef *hmotor)
{
        return hmotor->Target_Torque;
}

__STATIC_INLINE float Motor_Get_Speed(Motor_HandleTypeDef *hmotor)
{
        return hmotor->Speed;
}

__STATIC_INLINE float Motor_Get_Total_Angle(Motor_HandleTypeDef *hmotor)
{
        return hmotor->Total_Angle;
}




#endif //G4MINI_V3_DRV_MOTOR_H
