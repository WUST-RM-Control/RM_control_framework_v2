//============================================================================================================================================================================
//                                              Motor_Driver[电机驱动]
//      包含电机对象的实例、创建与底层数据回调, 控制逻辑见 SRV 层 srv_motor_control
//============================================================================================================================================================================

#include "drv_motor.h"

#include "drv_buzzer.h"
#include "hal_can.h"
#include "hal_dwt.h"

uint8_t Motor_Count = 0;

//通过角度改变计算速度
void Motor_Get_TotalAngle_Speed(Motor_HandleTypeDef *hmotor, float K)
{
        float Dt                 = DWT_GetDeltaT(&hmotor->Total_Angle_DWT_Count);
        hmotor->Speed            = (hmotor->Total_Angle - hmotor->Total_Angle_Last) / Dt * 60.0f / 360.0f;
        hmotor->Speed            = K * hmotor->Speed + (1 - K) * hmotor->Speed_Last;
        hmotor->Speed_Last       = hmotor->Speed;
        hmotor->Total_Angle_Last = hmotor->Total_Angle;
}

//电机CAN节点数据回调: 转发到电机vtable的storage_data
void Motor_CAN_Node_Handler(CAN_Node_HandleTypeDef *node, const uint8_t *Data)
{
        Motor_HandleTypeDef *hmotor = (Motor_HandleTypeDef *)node;
        hmotor->Motor_Config_Struct.vptr->storage_data(hmotor, Data);
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
                        Motor_Recover((Motor_HandleTypeDef *)herr);
                }
        }
}

//创建电机对象
void Motor_Ctor(Motor_HandleTypeDef *hmotor,CAN_HandleTypeDef *hcan, uint32_t CAN_Send_ID, uint32_t CAN_Feedback_ID, Motor_Config_StructTypeDef Motor_Config_Struct)
{
        memset(hmotor, 0, sizeof(Motor_HandleTypeDef));

        hmotor->Motor_Config_Struct = Motor_Config_Struct;
        CAN_Node_Ctor(&hmotor->Node, hcan, CAN_Send_ID, CAN_Feedback_ID, Motor_Config_Struct.CAN_Feedback_ID_Mask, Motor_CAN_Node_Handler, Motor_Config_Struct.herr.tick_timeout, Motor_Config_Struct.herr.count_maximum, Motor_Config_Struct.herr.handler);

        Motor_Count++;
}

//电机组初始化
//成员数组需先用静态初始化写好; 组长 member[0] 为空则不建组
void Motor_Group_Init(Motor_Group_HandleTypeDef *group)
{
        if (group == NULL || group->member[0] == NULL) return;

        for (uint8_t i = 0; i < MOTOR_GROUP_MAX; i++)
        {
                if (group->member[i] != NULL)
                {
                        group->member[i]->Group = group;
                }
        }
}
