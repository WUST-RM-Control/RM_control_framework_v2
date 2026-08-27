//============================================================================================================================================================================
//                                              Motor_Driver[电机驱动]
//      包含电机对象的实例、创建与底层数据回调, 控制逻辑见 SRV 层 srv_motor_control
//============================================================================================================================================================================

#include "drv_motor.h"

#include "hal_can.h"
#include "hal_dwt.h"

/*===| 电机对象实例 |===*/
Motor_HandleTypeDef hmotor_chassis1    = {};
Motor_HandleTypeDef hmotor_chassis2    = {};
Motor_HandleTypeDef hmotor_chassis3    = {};
Motor_HandleTypeDef hmotor_chassis4    = {};
Motor_HandleTypeDef hmotor_yaw         = {};
Motor_HandleTypeDef hmotor_pitch       = {};
Motor_HandleTypeDef hmotor_fric_right  = {};
Motor_HandleTypeDef hmotor_fric_left   = {};
Motor_HandleTypeDef hmotor_trigger     = {};

Motor_HandleTypeDef *hmotor[MOTOR_COUNT] = {
        &hmotor_chassis1,
        &hmotor_chassis2,
        &hmotor_chassis3,
        &hmotor_chassis4,
        &hmotor_yaw,
        &hmotor_pitch,
        &hmotor_fric_right,
        &hmotor_fric_left,
        &hmotor_trigger
};

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
