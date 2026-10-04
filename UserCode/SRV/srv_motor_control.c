//
// Created by ye on 2026/8/26.
//

#include "srv_motor_control.h"

#include "drv_motor_dji.h"
#include "drv_motor_dm.h"
#include "fdcan.h"

/*===| 电机对象实例 |===*/
static Motor_HandleTypeDef hmotor_chassis1   = {};
static Motor_HandleTypeDef hmotor_chassis2   = {};
static Motor_HandleTypeDef hmotor_chassis3   = {};
static Motor_HandleTypeDef hmotor_chassis4   = {};
static Motor_HandleTypeDef hmotor_yaw        = {};
static Motor_HandleTypeDef hmotor_pitch      = {};
static Motor_HandleTypeDef hmotor_fric_right = {};
static Motor_HandleTypeDef hmotor_fric_left  = {};
static Motor_HandleTypeDef hmotor_trigger    = {};

static Motor_HandleTypeDef *hmotor_table[MOTOR_COUNT] = {
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

/*===| 发送组(一拖四): 报文槽位由成员回报ID决定, 数组顺序只决定组长(member[0]) |===*/
//底盘 4 个 (hfdcan1, 0x200): 回报ID 0x201~0x204 → 槽位0~3
static Motor_Group_HandleTypeDef hmotor_group_chassis = {
        .member = {&hmotor_chassis1, &hmotor_chassis2, &hmotor_chassis3, &hmotor_chassis4}
};
//摩擦轮 (hfdcan3, 0x200): 回报ID 0x201/0x202 → 槽位0/1, 其余槽报文填0
static Motor_Group_HandleTypeDef hmotor_group_fric = {
        .member = {&hmotor_fric_right, &hmotor_fric_left}
};
//拨弹盘 (hfdcan2, 0x200): 回报ID 0x204 → 槽位3
static Motor_Group_HandleTypeDef hmotor_group_trigger = {
        .member = {&hmotor_trigger}
};
//pitch (hfdcan3, 0x1FE): 回报ID 0x206 → 槽位1
static Motor_Group_HandleTypeDef hmotor_group_pitch = {
        .member = {&hmotor_pitch}
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
                           Motor_DJI_Config_Default
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
                           Motor_DM_Config_Default
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
                           Motor_DJI_Config_Default
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
                           Motor_DJI_Config_Default
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
                           Motor_DJI_Config_Default
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

        /*===| 发送组绑定(一拖四设备): 回填组内成员的 Group 反向指针 |===*/
        Motor_Group_Init(&hmotor_group_chassis);
        Motor_Group_Init(&hmotor_group_fric);
        Motor_Group_Init(&hmotor_group_trigger);
        Motor_Group_Init(&hmotor_group_pitch);
}

/*===| 电机控制任务: "计算 → 发送" 两阶段 |===*/
void Motor_Control_Task(void *pvParameters)
{
        for (;;)
        {
                /*===| 阶段1: 计算(更新所有电机的目标值) |===*/
                for (int i = 0; i < Motor_Count; i++)
                {
                        //离线输出0力矩
                        if (Is_Err((Err_HandleTypeDef *) hmotor_table[i]))
                        {
                                Motor_Set_Torque(hmotor_table[i], 0);
                                continue;
                        }

                        if (hmotor_table[i]->Motor_Config_Struct.Is_Feedback_Control)//下位机闭环
                        {
                                //位置环: 角度 → 目标速度
                                if (hmotor_table[i]->Motor_Config_Struct.Status_Enum == MOTOR_ANGLE)
                                {
                                        hmotor_table[i]->Target_Speed = PID_Calculate(&hmotor_table[i]->PID_Angle_Struct, hmotor_table[i]->Angle, hmotor_table[i]->Target_Angle);
                                }
                                //速度环: 速度 → 目标力矩
                                if (hmotor_table[i]->Motor_Config_Struct.Status_Enum == MOTOR_SPEED ||
                                    hmotor_table[i]->Motor_Config_Struct.Status_Enum == MOTOR_ANGLE)
                                {
                                        hmotor_table[i]->Target_Torque = PID_Calculate(&hmotor_table[i]->PID_Speed_Struct, hmotor_table[i]->Speed, hmotor_table[i]->Target_Speed);
                                }
                        }
                }

                /*===| 阶段2: 发送 ================================================
                 * 与阶段1分开的原因: 一拖四设备(DJI)由组长一次性组帧,
                 * 组长必须读到组内所有成员本轮的 Target_*, 否则其余成员会滞后一个周期
                 *===============================================================*/
                for (int i = 0; i < Motor_Count; i++)
                {
                        if (hmotor_table[i]->Motor_Config_Struct.Is_Feedback_Control)//下位机闭环: 下发MCU算好的力矩
                        {
                                Motor_Send_Torque(hmotor_table[i], hmotor_table[i]->Target_Torque);
                        }
                        else//电调闭环: 目标直接下发
                        {
                                if (hmotor_table[i]->Motor_Config_Struct.Status_Enum == MOTOR_ANGLE)
                                {
                                        Motor_Send_Angle(hmotor_table[i], hmotor_table[i]->Target_Angle);
                                }
                                else if (hmotor_table[i]->Motor_Config_Struct.Status_Enum == MOTOR_SPEED)
                                {
                                        Motor_Send_Speed(hmotor_table[i], hmotor_table[i]->Target_Speed);
                                }
                                else if (hmotor_table[i]->Motor_Config_Struct.Status_Enum == MOTOR_TORQUE)
                                {
                                        Motor_Send_Torque(hmotor_table[i], hmotor_table[i]->Target_Torque);
                                }
                        }
                }

                //ps：g4的每个can控制器都只有3个邮箱，所以不可以一次塞入3个以上个包
                vTaskDelay(1);
        }
}
