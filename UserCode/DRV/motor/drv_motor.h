//
// Created by ye on 2026/8/22.
//

#ifndef G4MINI_V3_DRV_MOTOR_H
#define G4MINI_V3_DRV_MOTOR_H

#include "controller.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "hal_bus.h"
#include "hal_can.h"
#include "hal_uart.h"

#define MOTOR_ERR_COUNT_MAX 10
#define MOTOR_TIMEOUT       10

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
        MOTOR_ZDT,
        MOTOR_SERVO
} Motor_Enum_TypeDef;

typedef struct Motor_Base_HandleTypeDef Motor_Base_HandleTypeDef;
typedef struct Motor_VTable             Motor_VTable;

/*===| 单个控制环: 前馈 + 反馈 |===*/
typedef struct
{
        Feedforward_t FFC;
        PID_t         PID;
} Motor_Loop_CtrlTypeDef;

/*===| 控制环参数(由上层整定表提供) |===*/
typedef struct
{
        float Kp;
        float Ki;
        float Kd;

        float MaxOut;
        float IntegralLimit;
        float DeadBand;

        float CoefA;
        float CoefB;

        float Output_LPF_RC;
        float Derivative_LPF_RC;

        uint16_t OLS_Order;
        uint8_t  Improve;
} Motor_PID_ParamTypeDef;

typedef struct
{
        float MaxOut;
        float C[3];
        float LPF_RC;

        uint16_t Ref_Dot_OLS_Order;
        uint16_t Ref_Ddot_OLS_Order;
} Motor_FFC_ParamTypeDef;

__STATIC_INLINE void Motor_Loop_Init(Motor_Loop_CtrlTypeDef *loop, const Motor_PID_ParamTypeDef *pid_param, const Motor_FFC_ParamTypeDef *ffc_param)
{
        PID_Init(&loop->PID, pid_param->MaxOut, pid_param->IntegralLimit, pid_param->DeadBand,
                 pid_param->Kp, pid_param->Ki, pid_param->Kd,
                 pid_param->CoefA, pid_param->CoefB,
                 pid_param->Output_LPF_RC, pid_param->Derivative_LPF_RC,
                 pid_param->OLS_Order, pid_param->Improve);

        Feedforward_Init(&loop->FFC, ffc_param->MaxOut, ffc_param->C, ffc_param->LPF_RC,
                         ffc_param->Ref_Dot_OLS_Order, ffc_param->Ref_Ddot_OLS_Order);
}

/*===| 电机虚函数表 |===*/
struct Motor_VTable
{
        void (*enable)(Motor_Base_HandleTypeDef *hmotor_base);

        void (*disable)(Motor_Base_HandleTypeDef *hmotor_base);

        void (*set_zero)(Motor_Base_HandleTypeDef *hmotor_base);

        //闭环计算: 目标 → Target_Torque (仅MCU内闭环的型号有实际动作)
        void (*update)(Motor_Base_HandleTypeDef *hmotor_base);

        //下发: MIT 5 参数(型号不支持的量传0忽略)
        void (*send)(Motor_Base_HandleTypeDef *hmotor_base, float _pos, float _vel, float _KP, float _KD, float _torq);

        //总线帧到达: 从 Link.Node.Rx_Data/Rx_Length 自行解包
        void (*on_frame)(Motor_Base_HandleTypeDef *hmotor_base);
};

/*===| 电机配置(型号默认值) |===*/
typedef struct
{
        Motor_VTable *       vptr;
        Motor_Enum_TypeDef   Motor_Enum;
        Motor_Status_TypeDef Status_Enum;
        bool                 Is_Feedback_Control;

        float Gain_Position;
        float Gain_Velocity;
} Motor_Config_StructTypeDef;

/*===| 电机数据结构体 |===*/
struct Motor_Base_HandleTypeDef
{
        /* 链路节点: 三个视图共用同一块内存(公共首段相同), 按实际总线取用
         *   Link.Node : 基类视图  → herr / Bus_Type / Device_ID / handler
         *   Link.CAN  : CAN 电机
         *   Link.UART : UART 电机 */
        union
        {
                Bus_Node_HandleTypeDef  Node;
                CAN_Node_HandleTypeDef  CAN;
                UART_Node_HandleTypeDef UART;
        } Link;

        Motor_VTable *vptr;

        Motor_Enum_TypeDef   Motor_Enum;
        Motor_Status_TypeDef Status_Enum;

        bool Is_Feedback_Control : 1;

        uint8_t Error_Code; //反馈错误码(1=正常, 0=失能, 3~E=故障)

        float Gain_Position;
        float Gain_Velocity;

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

//电机总线节点回调(供总线层注册, CAN/UART 共用)
void Motor_Bus_Node_Handler(Bus_Node_HandleTypeDef *bus_node);

//电机错误回调
void Motor_Err_Handler(Err_HandleTypeDef *herr);

//通过角度改变计算速度
void Motor_Get_TotalAngle_Speed(Motor_Base_HandleTypeDef *hmotor_base, float K);

void Motor_Base_Ctor(Motor_Base_HandleTypeDef *hmotor_base, const Motor_Config_StructTypeDef *config);

/*===| vtable 空实现(签名与 vtable 一致, 避免函数指针强转) |===*/
__STATIC_INLINE void Motor_Null_Void(Motor_Base_HandleTypeDef *hmotor_base) { (void) hmotor_base; }

/*===| 控制接口 |===*/
__STATIC_INLINE void Motor_Enable(Motor_Base_HandleTypeDef *hmotor_base) { hmotor_base->vptr->enable(hmotor_base); }

__STATIC_INLINE void Motor_Disable(Motor_Base_HandleTypeDef *hmotor_base) { hmotor_base->vptr->disable(hmotor_base); }

__STATIC_INLINE void Motor_Set_Zero(Motor_Base_HandleTypeDef *hmotor_base) { hmotor_base->vptr->set_zero(hmotor_base); }

__STATIC_INLINE void Motor_Recover(Motor_Base_HandleTypeDef *hmotor_base) { Motor_Enable(hmotor_base); }

__STATIC_INLINE void Motor_Update(Motor_Base_HandleTypeDef *hmotor_base) { hmotor_base->vptr->update(hmotor_base); }

//从对象当前状态组包下发
__STATIC_INLINE void Motor_Send(Motor_Base_HandleTypeDef *hmotor_base)
{
        hmotor_base->vptr->send(hmotor_base, hmotor_base->Target_Angle, hmotor_base->Target_Speed,
                                hmotor_base->Gain_Position, hmotor_base->Gain_Velocity, hmotor_base->Target_Torque);
}

//显式 5 参数下发
__STATIC_INLINE void Motor_Send_Cmd(Motor_Base_HandleTypeDef *hmotor_base, float _pos, float _vel, float _KP, float _KD, float _torq)
{
        hmotor_base->vptr->send(hmotor_base, _pos, _vel, _KP, _KD, _torq);
}

/*===| 目标值 |===*/
__STATIC_INLINE void Motor_Set_Torque(Motor_Base_HandleTypeDef *hmotor_base, float torque)
{
        hmotor_base->Status_Enum   = MOTOR_TORQUE;
        hmotor_base->Target_Torque = torque;
}

__STATIC_INLINE void Motor_Set_Speed(Motor_Base_HandleTypeDef *hmotor_base, float speed)
{
        hmotor_base->Status_Enum  = MOTOR_SPEED;
        hmotor_base->Target_Speed = speed;
}

__STATIC_INLINE void Motor_Set_Angle(Motor_Base_HandleTypeDef *hmotor_base, float angle)
{
        hmotor_base->Status_Enum  = MOTOR_ANGLE;
        hmotor_base->Target_Angle = angle;
}

/*===| 反馈值 |===*/
__STATIC_INLINE float Motor_Get_Target_Torque(Motor_Base_HandleTypeDef *hmotor_base) { return hmotor_base->Target_Torque; }

__STATIC_INLINE float Motor_Get_Speed(Motor_Base_HandleTypeDef *hmotor_base) { return hmotor_base->Speed; }

__STATIC_INLINE float Motor_Get_Total_Angle(Motor_Base_HandleTypeDef *hmotor_base) { return hmotor_base->Total_Angle; }

#endif //G4MINI_V3_DRV_MOTOR_H
