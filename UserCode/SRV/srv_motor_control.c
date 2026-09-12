//
// Created by ye on 2026/8/26.
//

#include "srv_motor_control.h"

#include "drv_motor_dji.h"
#include "drv_motor_dm.h"
#include "fdcan.h"

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

Motor_HandleTypeDef *hmotor_table[MOTOR_COUNT] = {
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

/*===| 电机系统初始化(创建对象 + PID整定 + 注册CAN节点) |===*/
void Motor_Init()
{
        static const uint16_t Motor_CAN_Feedback_ID_Table[9] = {
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
                Motor_Ctor(
                        hmotor_table[i],
                        &CHASSIS_MOTOR_CAN,
                        CHASSIS_MOTOR_SEND_CAN_ID,
                        Motor_CAN_Feedback_ID_Table[i],
                        &Motor_DJI_VTable_Default,
                        MOTOR_TIMEOUT,
                        ERR_COUNT_MAX,
                        Motor_Err_Handler
                        );
                // Motor_Set_Status(hmotor_table[i], MOTOR_TORQUE);//这里设置并无屌用，留着只是告诉大家这个电机是什么控制方式
        }

        //yaw
        {
                Motor_Ctor(
                        &hmotor_yaw,
                        &GIMBAL_YAW_CAN,
                        GIMBAL_YAW_SEND_CAN_ID,
                        GIMBAL_YAW_FEEDBACK_CAN_ID,
                        &Motor_DM_VTable_Default,
                        MOTOR_TIMEOUT,
                        ERR_COUNT_MAX,
                        Motor_Err_Handler
                        );
                // Motor_Set_Status(&hmotor_yaw, MOTOR_ANGLE);

                PID_Init(
                         &hmotor_yaw.PID_Angle_Struct,
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
                         (Integral_Limit | ErrorHandle)
                        );

                PID_Init(
                         &hmotor_yaw.PID_Speed_Struct,
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
                         (Integral_Limit | ErrorHandle)
                        );
        }

        //pitch
        {
                Motor_Ctor(
                        &hmotor_pitch,
                        &GIMBAL_PITCH_CAN,
                        GIMBAL_PITCH_SEND_CAN_ID,
                        GIMBAL_PITCH_FEEDBACK_CAN_ID,
                        &Motor_DJI_VTable_Default,
                        MOTOR_TIMEOUT,
                        ERR_COUNT_MAX,
                        Motor_Err_Handler
                        );
                // Motor_Set_Status(&hmotor_pitch, MOTOR_ANGLE);

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
                         (Integral_Limit | ErrorHandle)
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
                         (Integral_Limit | ErrorHandle)
                        );
        }

        //摩擦轮 * 2
        for (int i = 6; i < 8; i++)
        {
                Motor_Ctor(
                        hmotor_table[i],
                        &SHOOT_FRIC_CAN,
                        SHOOT_FRIC_SEND_CAN_ID,
                        Motor_CAN_Feedback_ID_Table[i],
                        &Motor_DJI_VTable_Default,
                        MOTOR_TIMEOUT,
                        ERR_COUNT_MAX,
                        Motor_Err_Handler
                        );
                // Motor_Set_Status(hmotor_table[i], MOTOR_SPEED);

                PID_Init(
                         &hmotor_table[i]->PID_Speed_Struct,
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
                         (Integral_Limit | ErrorHandle)
                        );
        }

        //拨弹盘
        {
                Motor_Ctor(
                        &hmotor_trigger,
                        &SHOOT_TRIGGER_CAN,
                        SHOOT_TRIGGER_SEND_CAN_ID,
                        SHOOT_TRIGGER_FEEDBACK_CAN_ID,
                        &Motor_DJI_VTable_Default,
                        MOTOR_TIMEOUT,
                        ERR_COUNT_MAX,
                        Motor_Err_Handler
                        );
                // Motor_Set_Status(&hmotor_trigger, MOTOR_ANGLE);

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
                         (Integral_Limit | ErrorHandle)
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
                         (Integral_Limit | ErrorHandle)
                        );
        }

}

/*===| 电机控制任务: PID计算 + 发送力矩 |===*/
void Motor_Control_Task(void *pvParameters)
{
        Motor_Init();
        for (;;)
        {
                //计算所有电机的pid
                for (int i = 0; i < MOTOR_COUNT; i++)
                {
                        //离线输出0力矩
                        if (!Motor_Is_Online(hmotor_table[i]))
                        {
                                Motor_Set_Torque(hmotor_table[i], 0);
                                continue;
                        }

                        //位置环: 角度 → 目标速度
                        if (hmotor_table[i]->Status_Enum == MOTOR_ANGLE)
                        {
                                hmotor_table[i]->Target_Speed = PID_Calculate(&hmotor_table[i]->PID_Angle_Struct, hmotor_table[i]->Angle, hmotor_table[i]->Target_Angle);
                        }
                        //速度环: 速度 → 目标力矩
                        if (hmotor_table[i]->Status_Enum == MOTOR_SPEED ||
                            hmotor_table[i]->Status_Enum == MOTOR_ANGLE)
                        {
                                hmotor_table[i]->Target_Torque = PID_Calculate(&hmotor_table[i]->PID_Speed_Struct, hmotor_table[i]->Speed, hmotor_table[i]->Target_Speed);
                        }
                }

                //底盘电机发电流 can1
                Motor_DJI_SendCurrent(&CHASSIS_MOTOR_CAN, CHASSIS_MOTOR_SEND_CAN_ID,
                                      (int16_t) Motor_Get_Target_Torque(&hmotor_chassis1),
                                      (int16_t) Motor_Get_Target_Torque(&hmotor_chassis2),
                                      (int16_t) Motor_Get_Target_Torque(&hmotor_chassis3),
                                      (int16_t) Motor_Get_Target_Torque(&hmotor_chassis4)
                                     );

                //yaw电机发电流 can2
                Motor_DM_Send_Torque(&hmotor_yaw, Motor_Get_Target_Torque(&hmotor_yaw));

                //拨弹盘 can2
                Motor_DJI_SendCurrent(&SHOOT_TRIGGER_CAN, SHOOT_TRIGGER_SEND_CAN_ID,
                                      0,
                                      0,
                                      0,
                                      (int16_t) Motor_Get_Target_Torque(&hmotor_trigger)
                                     );

                //pitch电机发电流 can3
                Motor_DJI_SendCurrent(&GIMBAL_PITCH_CAN, GIMBAL_PITCH_SEND_CAN_ID,
                                      0,
                                      (int16_t) Motor_Get_Target_Torque(&hmotor_pitch),
                                      0,
                                      0
                                     );

                //俩摩擦轮 can3
                Motor_DJI_SendCurrent(&SHOOT_FRIC_CAN, SHOOT_FRIC_SEND_CAN_ID,
                                      (int16_t) Motor_Get_Target_Torque(&hmotor_fric_right),
                                      (int16_t) Motor_Get_Target_Torque(&hmotor_fric_left),
                                      0,
                                      0
                                     );

                //ps：g4的每个can控制器都只有3个邮箱，所以不可以一次塞入3个以上个包
                vTaskDelay(1);
        }
}
