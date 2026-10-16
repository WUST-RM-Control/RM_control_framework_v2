//============================================================================================================================================================================
//                                              Motor_Driver[电机驱动]
//      包含电机对象基类、总线回调与通用构造, 型号驱动见 drv_motor_xxx
//============================================================================================================================================================================

#include "drv_motor.h"

#include "hal_dwt.h"

uint8_t Motor_Count = 0;

//通过角度改变计算速度
void Motor_Get_TotalAngle_Speed(Motor_Base_HandleTypeDef *hmotor_base, float K)
{
        float Dt = DWT_GetDeltaT(&hmotor_base->Total_Angle_DWT_Count);

        hmotor_base->Speed            = (hmotor_base->Total_Angle - hmotor_base->Total_Angle_Last) / Dt * 60.0f / 360.0f;
        hmotor_base->Speed            = K * hmotor_base->Speed + (1 - K) * hmotor_base->Speed_Last;
        hmotor_base->Speed_Last       = hmotor_base->Speed;
        hmotor_base->Total_Angle_Last = hmotor_base->Total_Angle;
}

//总线帧到达 → 分发到型号驱动的 on_frame
void Motor_Bus_Node_Handler(Bus_Node_HandleTypeDef *bus_node)
{
        Motor_Base_HandleTypeDef *hmotor_base = container_of(bus_node, Motor_Base_HandleTypeDef, Link.Node);

        hmotor_base->vptr->on_frame(hmotor_base);
}

void Motor_Err_Handler(Err_HandleTypeDef *herr)
{
        herr->tick++;
        if (herr->tick > herr->tick_timeout) herr->If_Err = 1;

        if (herr->count < herr->count_maximum)
        {
                if (herr->tick > 2 * herr->tick_timeout)
                {
                        herr->count++;
                        herr->tick = herr->tick_timeout;
                        Motor_Recover(container_of(herr, Motor_Base_HandleTypeDef, Link.Node.herr));
                }
        }
}

//基类构造: 只初始化基类段, 链路节点由型号驱动的 Ctor 负责
void Motor_Base_Ctor(Motor_Base_HandleTypeDef *hmotor_base, const Motor_Config_StructTypeDef *config)
{
        memset(hmotor_base, 0, sizeof(Motor_Base_HandleTypeDef));

        hmotor_base->vptr                = config->vptr;
        hmotor_base->Motor_Enum          = config->Motor_Enum;
        hmotor_base->Status_Enum         = config->Status_Enum;
        hmotor_base->Is_Feedback_Control = config->Is_Feedback_Control;
        hmotor_base->Gain_Position       = config->Gain_Position;
        hmotor_base->Gain_Velocity       = config->Gain_Velocity;
        hmotor_base->Error_Code          = 1;

        Motor_Count++;
}
