//
// Created by ye on 2026/8/22.
//

#ifndef G4MINI_V3_DRV_MOTOR_H
#define G4MINI_V3_DRV_MOTOR_H

#include "controller.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "hal_can.h"

#define MOTOR_ERR_COUNT_MAX 10
#define MOTOR_TIMEOUT 10

extern uint8_t Motor_Count;

typedef enum
{
        MOTOR_TORQUE = 0,
        MOTOR_SPEED,
        MOTOR_ANGLE
} Motor_Status_TypeDef;

typedef enum
{
        MOTOR_DJI = 0,
        MOTOR_DM,
        MOTOR_ZDT
} Motor_Enum_TypeDef;

typedef struct Motor_HandleTypeDef       Motor_HandleTypeDef;
typedef struct Motor_VTable              Motor_VTable;
typedef struct Motor_Group_HandleTypeDef Motor_Group_HandleTypeDef;

typedef struct
{
        Err_HandleTypeDef herr;

        Motor_VTable *       vptr;
        Motor_Enum_TypeDef   Motor_Enum;
        Motor_Status_TypeDef Status_Enum;

        uint32_t CAN_Feedback_ID_Mask;

        bool Is_Feedback_Control: 1; //MCU内闭环为1，电调内闭环为0

        uint8_t Error_Code; //反馈错误码(1=正常, 0=失能, 3~E=故障; DJI不更新恒为1)

        uint16_t Default_Vel;   //默认速度[RPM]
        uint8_t  Default_Acc;   //默认加速度(0=直接启动)

        //步进运动参数: 角度→脉冲换算 + 默认下发参数
        uint16_t Pulse_Per_Rev; //每圈脉冲数(对应驱动细分, 如16细分=3200)
} Motor_Config_StructTypeDef;

struct Motor_VTable
{
        void (*enable)(Motor_HandleTypeDef *hmotor);

        void (*disable)(Motor_HandleTypeDef *hmotor);

        void (*set_zero)(Motor_HandleTypeDef *hmotor);

        void (*send_torque)(Motor_HandleTypeDef *hmotor, float torque);

        void (*send_speed)(Motor_HandleTypeDef *hmotor, float speed);

        void (*send_angle)(Motor_HandleTypeDef *hmotor, float angle);

        void (*storage_data)(Motor_HandleTypeDef *hmotor, const uint8_t *Data);
};

/*===| 电机数据结构体定义 |===*/
struct Motor_HandleTypeDef
{
        CAN_Node_HandleTypeDef Node; //CAN节点基类(第一个成员, 可向上转型为基类指针)

        Motor_Group_HandleTypeDef *Group; //组(NULL=单发设备); 一拖四设备组内共享, 由 Motor_Group_Init 回填

        Motor_Config_StructTypeDef Motor_Config_Struct;

        Feedforward_t FFC_Angle_Struct;
        PID_t         PID_Angle_Struct;
        Feedforward_t FFC_Speed_Struct;
        PID_t         PID_Speed_Struct;

        volatile float Target_Torque; //目标力矩[电流]
        volatile float Target_Speed;  //目标速度
        volatile float Target_Angle;  //目标角度

        float   Torque;      //实际力矩[电流]
        int16_t Encoder;     //编码器值
        float   Angle;       //绝对角度
        float   Total_Angle; //总角度值
        float   Speed;       //转速[RPM]
        int8_t  Temperature; //电机温度

        float   Total_Angle_Offset; //总角度值零点
        int32_t Round;              //圈数

        int16_t  Encoder_Last;     //上一个编码器值
        float    Angle_Last;       //上一个绝对角度
        float    Total_Angle_Last; //上一个总角度值
        float    Speed_Last;       //上一个转速
        uint32_t Total_Angle_DWT_Count;
};

#define MOTOR_GROUP_MAX 4

/*===| 一拖四报文槽位约定(DJI类电调) |===*/
//回报ID = 0x200 + 电调ID; 报文每帧4槽, 槽位 = (电调ID - 1) % 4
#define MOTOR_FEEDBACK_ID_BASE 0x200

//由回报ID推算该电机在组帧报文里的槽位(0~3): 与建组时的数组顺序无关
__STATIC_INLINE uint8_t Motor_Get_FrameSlot(Motor_HandleTypeDef *hmotor)
{
        return (uint8_t) ((hmotor->Node.CAN_Feedback_ID - MOTOR_FEEDBACK_ID_BASE - 1U) % MOTOR_GROUP_MAX);
}

struct Motor_Group_HandleTypeDef
{
        Motor_HandleTypeDef *member[MOTOR_GROUP_MAX];
};


//电机CAN节点数据回调(供SRV层注册到CAN分发框架)
void Motor_CAN_Node_Handler(CAN_Node_HandleTypeDef *node, const uint8_t *Data);

//电机错误回调
void Motor_Err_Handler(Err_HandleTypeDef *herr);

//通过角度改变计算速度
void Motor_Get_TotalAngle_Speed(Motor_HandleTypeDef *hmotor, float K);

void Motor_Ctor(Motor_HandleTypeDef *hmotor, CAN_HandleTypeDef *hcan, uint32_t CAN_Send_ID, uint32_t CAN_Feedback_ID, Motor_Config_StructTypeDef Motor_Config_Struct);

//电机发送组初始化: 回填组内成员的 Group 反向指针(成员数组需先静态初始化好)
//组长 member[0] 为空则忽略(不建组)
void Motor_Group_Init(Motor_Group_HandleTypeDef *group);

//电机控制
__STATIC_INLINE void Motor_Enable(Motor_HandleTypeDef *hmotor) { hmotor->Motor_Config_Struct.vptr->enable(hmotor); }
//电机恢复
__STATIC_INLINE void Motor_Recover(Motor_HandleTypeDef *hmotor)
{
        Motor_Enable(hmotor);
        PID_Reset(&hmotor->PID_Angle_Struct);
        PID_Reset(&hmotor->PID_Speed_Struct);
}

__STATIC_INLINE void Motor_Disable(Motor_HandleTypeDef *hmotor) { hmotor->Motor_Config_Struct.vptr->disable(hmotor); }


__STATIC_INLINE void Motor_Storage_Data(Motor_HandleTypeDef *hmotor, const uint8_t *data) { hmotor->Motor_Config_Struct.vptr->storage_data(hmotor, data); }

//设置参数
__STATIC_INLINE void Motor_Set_Status(Motor_HandleTypeDef *hmotor, Motor_Status_TypeDef Status) { hmotor->Motor_Config_Struct.Status_Enum = Status; }
__STATIC_INLINE void Motor_Set_Zero(Motor_HandleTypeDef *hmotor, const uint8_t *data) { hmotor->Motor_Config_Struct.vptr->set_zero(hmotor); }
__STATIC_INLINE void Motor_Set_Torque(Motor_HandleTypeDef *hmotor, float torque)
{
        hmotor->Motor_Config_Struct.Status_Enum = MOTOR_TORQUE;
        hmotor->Target_Torque                   = torque;
}

__STATIC_INLINE void Motor_Set_Speed(Motor_HandleTypeDef *hmotor, float speed)
{
        hmotor->Motor_Config_Struct.Status_Enum = MOTOR_SPEED;
        hmotor->Target_Speed                    = speed;
}

__STATIC_INLINE void Motor_Set_Angle(Motor_HandleTypeDef *hmotor, float angle)
{
        hmotor->Motor_Config_Struct.Status_Enum = MOTOR_ANGLE;
        hmotor->Target_Angle                    = angle;
}

//获取参数
__STATIC_INLINE float Motor_Get_Target_Torque(Motor_HandleTypeDef *hmotor) { return hmotor->Target_Torque; }
__STATIC_INLINE float Motor_Get_Speed(Motor_HandleTypeDef *hmotor) { return hmotor->Speed; }
__STATIC_INLINE float Motor_Get_Total_Angle(Motor_HandleTypeDef *hmotor) { return hmotor->Total_Angle; }

/*===| 统一发送接口(通过vtable分发到各型号电机驱动) |===*/
__STATIC_INLINE void Motor_Send_Torque(Motor_HandleTypeDef *hmotor, float torque) { hmotor->Motor_Config_Struct.vptr->send_torque(hmotor, torque); }
__STATIC_INLINE void Motor_Send_Speed(Motor_HandleTypeDef *hmotor, float speed) { hmotor->Motor_Config_Struct.vptr->send_speed(hmotor, speed); }
__STATIC_INLINE void Motor_Send_Angle(Motor_HandleTypeDef *hmotor, float angle) { hmotor->Motor_Config_Struct.vptr->send_angle(hmotor, angle); }

#endif //G4MINI_V3_DRV_MOTOR_H
