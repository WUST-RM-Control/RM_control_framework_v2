//============================================================================================================================================================================
//                                              Motor_Driver[电机驱动]
//      包含了一些电机驱动相关的封装好的函数
//============================================================================================================================================================================

#include "drv_motor.h"

#include "drv_motor_dji.h"
#include "fdcan.h"
#include "hal_can.h"
#include "hal_dwt.h"

Motor_HandleTypeDef hmotor_chassis[4] = {};
Motor_HandleTypeDef hmotor_yaw        = {};
Motor_HandleTypeDef hmotor_pitch      = {};
Motor_HandleTypeDef hmotor_shoot[2]   = {};
Motor_HandleTypeDef hmotor_trigger    = {};


void Motor_Get_TotalAngle_Speed(Motor_HandleTypeDef *hmotor, float K)
{
        float Dt = DWT_GetDeltaT(&hmotor->Total_Angle_DWT_Count);
        hmotor->Total_Angle_Speed = (hmotor->Total_Angle - hmotor->Total_Angle_Last) / Dt * 60.0f / 360.0f;
        hmotor->Total_Angle_Speed = K * hmotor->Total_Angle_Speed + (1 - K) * hmotor->Total_Angle_Speed_RPM_Last;
        hmotor->Total_Angle_Speed_RPM_Last = hmotor->Total_Angle_Speed;

        hmotor->Total_Angle_Last = hmotor->Total_Angle;
}

void Motor_Control_Task(void *pvParameters)
{
        for (;;)
        {
                Motor_DJI_SendCurrent(CHASSIS_MOTOR_CAN, 0x200,
                        (int16_t)Motor_Get_Target_Torque(&hmotor_chassis[0]),
                        (int16_t)Motor_Get_Target_Torque(&hmotor_chassis[1]),
                        (int16_t)Motor_Get_Target_Torque(&hmotor_chassis[2]),
                        (int16_t)Motor_Get_Target_Torque(&hmotor_chassis[3])
                        );

                vTaskDelay(1);
        }
}

