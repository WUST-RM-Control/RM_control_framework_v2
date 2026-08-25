//============================================================================================================================================================================
//                                              Motor_Driver[电机驱动]
//      包含了一些电机驱动相关的封装好的函数
//============================================================================================================================================================================

#include "drv_motor.h"

#include "drv_motor_dji.h"
#include "drv_motor_dm.h"
#include "fdcan.h"
#include "hal_can.h"
#include "hal_dwt.h"


Motor_HandleTypeDef hmotor_chassis1 = {};
Motor_HandleTypeDef hmotor_chassis2 = {};
Motor_HandleTypeDef hmotor_chassis3 = {};
Motor_HandleTypeDef hmotor_chassis4 = {};
Motor_HandleTypeDef hmotor_yaw      = {};
Motor_HandleTypeDef hmotor_pitch    = {};
Motor_HandleTypeDef hmotor_shoot0   = {};
Motor_HandleTypeDef hmotor_shoot1   = {};
Motor_HandleTypeDef hmotor_trigger  = {};

Motor_HandleTypeDef *hmotor[MOTOR_COUNT] = {
        &hmotor_chassis1,
        &hmotor_chassis2,
        &hmotor_chassis3,
        &hmotor_chassis4,
        &hmotor_yaw,
        &hmotor_pitch,
        &hmotor_shoot0,
        &hmotor_shoot1,
        &hmotor_trigger
};


void Motor_Get_TotalAngle_Speed(Motor_HandleTypeDef *hmotor, float K)
{
        float Dt                           = DWT_GetDeltaT(&hmotor->Total_Angle_DWT_Count);
        hmotor->Total_Angle_Speed          = (hmotor->Total_Angle - hmotor->Total_Angle_Last) / Dt * 60.0f / 360.0f;
        hmotor->Total_Angle_Speed          = K * hmotor->Total_Angle_Speed + (1 - K) * hmotor->Total_Angle_Speed_RPM_Last;
        hmotor->Total_Angle_Speed_RPM_Last = hmotor->Total_Angle_Speed;

        hmotor->Total_Angle_Last = hmotor->Total_Angle;
}

//共九个电机初始化
static void Motor_Init()
{
        static const uint16_t Motor_CAN_Feedback_ID[9] = {
                CHASSIS_MOTOR1_FEEDBACK_CAN_ID,
                CHASSIS_MOTOR2_FEEDBACK_CAN_ID,
                CHASSIS_MOTOR3_FEEDBACK_CAN_ID,
                CHASSIS_MOTOR4_FEEDBACK_CAN_ID,
                GIMBAL_YAW_FEEDBACK_CAN_ID,
                GIMBAL_PITCH_FEEDBACK_CAN_ID,
                SHOOT_FRIC_RIGHT_FEEDBACK_CAN_ID,
                SHOOT_FRIC_LEFT_FEEDBACK_CAN_ID,
                SHOOT_TRIGGER_FEEDBACK_CAN_ID
        };
        //底盘电机 * 4
        for (int i = 0; i < 4; i++)
        {
                Motor_DJI_Ctor(hmotor[i], &CHASSIS_MOTOR_CAN, CHASSIS_MOTOR_SEND_CAN_ID, Motor_CAN_Feedback_ID[i]);
                Motor_Set_Status(hmotor[i], MOTOR_SPEED);
                PID_Init(
                         &hmotor[i]->PID_Speed_Struct,
                         32767,
                         16384,
                         0,
                         1,
                         0,
                         0,
                         0,
                         0,
                         0,
                         0,
                         0,
                         Integral_Limit
                        );
        }

        //yaw
        {
                Motor_DJI_Ctor(&hmotor_pitch, &GIMBAL_YAW_CAN, GIMBAL_YAW_SEND_CAN_ID, GIMBAL_YAW_FEEDBACK_CAN_ID);
                Motor_Set_Status(&hmotor_yaw, MOTOR_ANGLE);

                PID_Init(
                                 &hmotor_pitch.PID_Angle_Struct,
                                 32767,
                                 16384,
                                 0,
                                 1,
                                 0,
                                 0,
                                 0,
                                 0,
                                 0,
                                 0,
                                 0,
                                 Integral_Limit
                                );


                PID_Init(
                         &hmotor_pitch.PID_Speed_Struct,
                         32767,
                         16384,
                         0,
                         10,
                         0,
                         0,
                         0,
                         0,
                         0,
                         0,
                         0,
                         Integral_Limit
                        );
        }

        //pitch
        {
                Motor_DJI_Ctor(&hmotor_pitch, &GIMBAL_PITCH_CAN, GIMBAL_PITCH_SEND_CAN_ID, GIMBAL_PITCH_FEEDBACK_CAN_ID);
                Motor_Set_Status(&hmotor_pitch, MOTOR_ANGLE);

                PID_Init(
                         &hmotor_pitch.PID_Angle_Struct,
                         32767,
                         16384,
                         0,
                         1,
                         0,
                         0,
                         0,
                         0,
                         0,
                         0,
                         0,
                         Integral_Limit
                        );

                PID_Init(
                         &hmotor_pitch.PID_Speed_Struct,
                         32767,
                         16384,
                         0,
                         1,
                         0,
                         0,
                         0,
                         0,
                         0,
                         0,
                         0,
                         Integral_Limit
                        );
        }

        //摩擦轮 * 2
        for (int i = 6; i < 8; i++)
        {
                Motor_DJI_Ctor(hmotor[i], &SHOOT_FRIC_CAN, SHOOT_FRIC_SEND_CAN_ID, Motor_CAN_Feedback_ID[i]);
                Motor_Set_Status(hmotor[i], MOTOR_SPEED);

                PID_Init(
                         &hmotor[i]->PID_Speed_Struct,
                         32767,
                         16384,
                         0,
                         1,
                         0,
                         0,
                         0,
                         0,
                         0,
                         0,
                         0,
                         Integral_Limit
                        );
        }


        //拨弹盘
        {
                Motor_DJI_Ctor(&hmotor_trigger, &SHOOT_TRIGGER_CAN, SHOOT_TRIGGER_SEND_CAN_ID,SHOOT_TRIGGER_FEEDBACK_CAN_ID);
                Motor_Set_Status(&hmotor_trigger, MOTOR_ANGLE);

                PID_Init(
                         &hmotor_trigger.PID_Angle_Struct,
                         32767,
                         16384,
                         0,
                         1,
                         0,
                         0,
                         0,
                         0,
                         0,
                         0,
                         0,
                         Integral_Limit
                        );

                PID_Init(
                         &hmotor_trigger.PID_Speed_Struct,
                         32767,
                         16384,
                         0,
                         1,
                         0,
                         0,
                         0,
                         0,
                         0,
                         0,
                         0,
                         Integral_Limit
                        );
        }
}






void Motor_Control_Task(void *pvParameters)
{
        Motor_Init();

        for (;;)
        {
                for (int i = 0; i < MOTOR_COUNT; i++)
                {
                        //位置环
                        if (hmotor[i]->Status_Enum == MOTOR_ANGLE)
                        {
                                hmotor[i]->Target_Speed = PID_Calculate(&hmotor[i]->PID_Speed_Struct, hmotor[i]->Angle, hmotor[i]->Target_Angle);
                        }
                        //速度环
                        if (hmotor[i]->Status_Enum == MOTOR_ANGLE && hmotor[i]->Status_Enum == MOTOR_SPEED)
                        {
                                hmotor[i]->Target_Torque = PID_Calculate(&hmotor[i]->PID_Speed_Struct, hmotor[i]->Speed, hmotor[i]->Target_Speed);
                        }
                }


                Motor_DJI_SendCurrent(&CHASSIS_MOTOR_CAN, CHASSIS_MOTOR_SEND_CAN_ID,
                                      (int16_t) Motor_Get_Target_Torque(&hmotor_chassis1),
                                      (int16_t) Motor_Get_Target_Torque(&hmotor_chassis2),
                                      (int16_t) Motor_Get_Target_Torque(&hmotor_chassis3),
                                      (int16_t) Motor_Get_Target_Torque(&hmotor_chassis4)
                                     );

                Motor_DM_Set_Torque(&hmotor_yaw, Motor_Get_Target_Torque(&hmotor_yaw));

                Motor_DJI_SendCurrent(&SHOOT, CHASSIS_MOTOR_SEND_CAN_ID,
                                      (int16_t) Motor_Get_Target_Torque(&hmotor_chassis1),
                                      (int16_t) Motor_Get_Target_Torque(&hmotor_chassis2),
                                      (int16_t) Motor_Get_Target_Torque(&hmotor_chassis3),
                                      (int16_t) Motor_Get_Target_Torque(&hmotor_chassis4)
                                     );

                Motor_DJI_SendCurrent(&CHASSIS_MOTOR_CAN, CHASSIS_MOTOR_SEND_CAN_ID,
                                      (int16_t) Motor_Get_Target_Torque(&hmotor_chassis1),
                                      (int16_t) Motor_Get_Target_Torque(&hmotor_chassis2),
                                      (int16_t) Motor_Get_Target_Torque(&hmotor_chassis3),
                                      (int16_t) Motor_Get_Target_Torque(&hmotor_chassis4)
                                     );

                vTaskDelay(1);
        }
}
