//============================================================================================================================================================================
//                                              Motor_Driver[电机驱动]
//      包含电机对象的实例、创建与底层数据回调, 控制逻辑见 SRV 层 srv_motor_control
//============================================================================================================================================================================

#include "drv_motor.h"

#include "drv_buzzer.h"
#include "hal_can.h"
#include "hal_dwt.h"



//通过角度改变计算速度
void Motor_Get_TotalAngle_Speed(Motor_HandleTypeDef *hmotor, float K)
{
        float Dt                           = DWT_GetDeltaT(&hmotor->Total_Angle_DWT_Count);
        hmotor->Total_Angle_Speed          = (hmotor->Total_Angle - hmotor->Total_Angle_Last) / Dt * 60.0f / 360.0f;
        hmotor->Total_Angle_Speed          = K * hmotor->Total_Angle_Speed + (1 - K) * hmotor->Total_Angle_Speed_RPM_Last;
        hmotor->Total_Angle_Speed_RPM_Last = hmotor->Total_Angle_Speed;

        hmotor->Total_Angle_Last = hmotor->Total_Angle;
}

//电机CAN节点数据回调: 转发到电机vtable的storage_data
void Motor_CAN_Node_Handler(CAN_Node_HandleTypeDef *node, const uint8_t *Data)
{
        Motor_HandleTypeDef *hmotor = (Motor_HandleTypeDef *)node;
        hmotor->vptr->storage_data(hmotor, Data);
}

void Motor_Err_Handler(Err_HandleTypeDef *herr)
{
        herr->tick++;
        if (herr->tick > herr->tick_timeout) herr->If_Online = 0;

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
void Motor_Ctor(Motor_HandleTypeDef *hmotor,FDCAN_HandleTypeDef *hfdcan, uint16_t CAN_Send_ID, uint16_t CAN_Feedback_ID, Motor_VTable *Motor_VTable, uint16_t err_tick_timeout, uint16_t err_count_maximum, Err_Handler err_handler)
{
        memset(hmotor, 0, sizeof(Motor_HandleTypeDef));

        CAN_Node_Ctor(&hmotor->Node, hfdcan, CAN_Send_ID, CAN_Feedback_ID, Motor_CAN_Node_Handler, err_tick_timeout, err_count_maximum, err_handler);
        hmotor->vptr = Motor_VTable;
        hmotor->Error_Code = 1;//默认使能
}
