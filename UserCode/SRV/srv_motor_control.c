//
// Created by ye on 2026/8/26.
//

#include "srv_motor_control.h"

#include "drv_motor_dji.h"
#include "drv_motor_dm.h"
#include "fdcan.h"

/*===| 电机系统初始化(创建对象 + PID整定 + 注册CAN节点) |===*/
void Motor_Init()
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
                Motor_Set_Status(hmotor[i], MOTOR_TORQUE);
        }

        //yaw
        {
                Motor_DM_Ctor(&hmotor_yaw, &GIMBAL_YAW_CAN, GIMBAL_YAW_SEND_CAN_ID, GIMBAL_YAW_FEEDBACK_CAN_ID);
                Motor_Set_Status(&hmotor_yaw, MOTOR_ANGLE);

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
                         (Integral_Limit | ErrorHandle)
                        );
        }

        //拨弹盘
        {
                Motor_DJI_Ctor(&hmotor_trigger, &SHOOT_TRIGGER_CAN, SHOOT_TRIGGER_SEND_CAN_ID, SHOOT_TRIGGER_FEEDBACK_CAN_ID);
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

        //注册所有电机CAN节点到通用分发框架
        for (int i = 0; i < MOTOR_COUNT; i++)
        {
                CAN_Node_Register(&hmotor[i]->Node, Motor_CAN_Node_Handler);
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
                        //离线安全: 未收到反馈的电机不计算PID, 输出0力矩
                        if (!hmotor[i]->If_Online)
                        {
                                hmotor[i]->Target_Torque = 0;
                                continue;
                        }

                        //位置环: 角度 → 目标速度
                        if (hmotor[i]->Status_Enum == MOTOR_ANGLE)
                        {
                                hmotor[i]->Target_Speed = PID_Calculate(&hmotor[i]->PID_Angle_Struct, hmotor[i]->Angle, hmotor[i]->Target_Angle);
                        }
                        //速度环: 速度 → 目标力矩
                        if (hmotor[i]->Status_Enum == MOTOR_SPEED ||
                            hmotor[i]->Status_Enum == MOTOR_ANGLE)
                        {
                                hmotor[i]->Target_Torque = PID_Calculate(&hmotor[i]->PID_Speed_Struct, hmotor[i]->Speed, hmotor[i]->Target_Speed);
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
