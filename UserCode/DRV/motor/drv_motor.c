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
void Motor_Get_TotalAngle_Speed(Motor_Base_HandleTypeDef *hmotor, float K)
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
        Motor_Base_HandleTypeDef *hmotor = container_of(node, Motor_Base_HandleTypeDef, Node);
        hmotor->vptr->storage_data(hmotor, Data);
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
                        Motor_Recover(container_of(herr, Motor_Base_HandleTypeDef, Node.herr));
                }
        }
}

//创建电机对象
void Motor_Base_Ctor(
        Motor_Base_HandleTypeDef *hmotor_base,

        CAN_HandleTypeDef *hcan,
        uint16_t CAN_Send_ID,
        uint16_t CAN_Feedback_ID,
        uint32_t CAN_Feedback_ID_Mask,

        Motor_VTable *Motor_VTable,

        uint16_t err_tick_timeout,
        uint16_t err_count_maximum,
        err_handler err_handler
        )
{
        memset(hmotor_base, 0, sizeof(Motor_Base_HandleTypeDef));

        hmotor_base->vptr = Motor_VTable;

        CAN_Node_Ctor(&hmotor_base->Node, hcan, CAN_Send_ID, CAN_Feedback_ID, CAN_Feedback_ID_Mask, Motor_CAN_Node_Handler, err_tick_timeout, err_count_maximum, err_handler);

        Motor_Count++;
}
