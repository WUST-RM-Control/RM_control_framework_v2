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

#define MOTOR_COUNT 9

typedef enum
{
        MOTOR_TORQUE = 0,
        MOTOR_SPEED,
        MOTOR_ANGLE
} Motor_Status_TypeDef;


typedef struct Motor_HandleTypeDef Motor_HandleTypeDef;
typedef struct Motor_VTable        Motor_VTable;

struct Motor_VTable
{
        void (*enable)(Motor_HandleTypeDef *hmotor);

        void (*disable)(Motor_HandleTypeDef *hmotor);

        void (*set_zero)(Motor_HandleTypeDef *hmotor);

        void (*storage_data)(Motor_HandleTypeDef *hmotor, const uint8_t *Data);
};

/*===| 电机数据结构体定义 |===*/
struct Motor_HandleTypeDef
{
        CAN_Node_HandleTypeDef Node;   //CAN节点基类(第一个成员, 可向上转型为基类指针)

        Motor_VTable *vptr;
        Motor_Status_TypeDef Status_Enum;

        uint8_t Error_Code;     //DM反馈错误码(1=正常, 0=失能, 3~E=故障; DJI不更新恒为1)

        Feedforward_t FFC_Angle_Struct;
        PID_t         PID_Angle_Struct;
        Feedforward_t FFC_Speed_Struct;
        PID_t         PID_Speed_Struct;

        volatile float Target_Torque; //目标力矩[电流]
        volatile float Target_Speed;  //目标速度
        volatile float Target_Angle;  //目标角度

        float   Torque;            //实际力矩[电流]
        int16_t Encoder;           //编码器值
        float   Angle;             //绝对角度
        float   Total_Angle;       //总角度值
        float   Speed;             //转速[RPM]
        int8_t  Temperature;       //电机温度

        float   Total_Angle_Offset; //总角度值零点
        int32_t Round;              //圈数

        int16_t Encoder_Last;               //上一个编码器值
        float   Angle_Last;                 //上一个绝对角度
        float   Total_Angle_Last;           //上一个总角度值
        float   Speed_Last;                 //上一个转速[RPM]

        uint32_t Total_Angle_DWT_Count;

        uint16_t Ticker;    //收到数据计时
        uint8_t  If_Online; //是否在线
};


/*===| 电机对象实例 |===*/
extern Motor_HandleTypeDef hmotor_chassis1;
extern Motor_HandleTypeDef hmotor_chassis2;
extern Motor_HandleTypeDef hmotor_chassis3;
extern Motor_HandleTypeDef hmotor_chassis4;
extern Motor_HandleTypeDef hmotor_yaw;
extern Motor_HandleTypeDef hmotor_pitch;
extern Motor_HandleTypeDef hmotor_fric_right;
extern Motor_HandleTypeDef hmotor_fric_left;
extern Motor_HandleTypeDef hmotor_trigger;

extern Motor_HandleTypeDef *hmotor[MOTOR_COUNT];

//电机在线检查(由监控任务周期调用): 超过 timeout_tick 周期未收到反馈则离线
__STATIC_INLINE void Motor_Online_Check(Motor_HandleTypeDef *hmotor, uint16_t timeout_tick)
{
        if (hmotor->If_Online)
        {
                hmotor->Ticker++;//调用Motor_Storage_Data时会将之置0
                if (hmotor->Ticker > timeout_tick)
                        hmotor->If_Online = 0;
        }
}

//电机CAN节点数据回调(供SRV层注册到CAN分发框架)
void Motor_CAN_Node_Handler(CAN_Node_HandleTypeDef *node, const uint8_t *Data);

//通过角度改变计算速度
void Motor_Get_TotalAngle_Speed(Motor_HandleTypeDef *hmotor, float K);


__STATIC_INLINE void Motor_Enable(Motor_HandleTypeDef *hmotor) { hmotor->vptr->enable(hmotor); }

//电机恢复: 使能(vtable: DM=ClearErr+Enable, DJI=空操作) + 清除PID积分/输出历史(防止恢复后积分饱和)
__STATIC_INLINE void Motor_Recover(Motor_HandleTypeDef *hmotor)
{
        Motor_Enable(hmotor);
        PID_Reset(&hmotor->PID_Angle_Struct);
        PID_Reset(&hmotor->PID_Speed_Struct);
}

__STATIC_INLINE void Motor_Disable(Motor_HandleTypeDef *hmotor) { hmotor->vptr->disable(hmotor); }

__STATIC_INLINE void Motor_Set_Status(Motor_HandleTypeDef *hmotor, Motor_Status_TypeDef Status) { hmotor->Status_Enum = Status; }

__STATIC_INLINE void Motor_Set_Zero(Motor_HandleTypeDef *hmotor, const uint8_t *data) { hmotor->vptr->set_zero(hmotor); }

__STATIC_INLINE void Motor_Storage_Data(Motor_HandleTypeDef *hmotor, const uint8_t *data) { hmotor->vptr->storage_data(hmotor, data); }

__STATIC_INLINE void Motor_Set_Torque(Motor_HandleTypeDef *hmotor, float torque)
{
        hmotor->Status_Enum   = MOTOR_TORQUE;
        hmotor->Target_Torque = torque;
}

__STATIC_INLINE void Motor_Set_Speed(Motor_HandleTypeDef *hmotor, float speed)
{
        hmotor->Status_Enum  = MOTOR_SPEED;
        hmotor->Target_Speed = speed;
}

__STATIC_INLINE void Motor_Set_Angle(Motor_HandleTypeDef *hmotor, float angle)
{
        hmotor->Status_Enum  = MOTOR_ANGLE;
        hmotor->Target_Angle = angle;
}

__STATIC_INLINE float Motor_Get_Target_Torque(Motor_HandleTypeDef *hmotor) { return hmotor->Target_Torque; }

__STATIC_INLINE float Motor_Get_Speed(Motor_HandleTypeDef *hmotor) { return hmotor->Speed; }

__STATIC_INLINE float Motor_Get_Total_Angle(Motor_HandleTypeDef *hmotor) { return hmotor->Total_Angle; }

#endif //G4MINI_V3_DRV_MOTOR_H
