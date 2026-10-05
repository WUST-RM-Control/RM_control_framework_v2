//
// Created by ye on 2026/8/26.
//

#include "srv_motor_control.h"

#include "drv_motor_dji.h"
#include "drv_motor_dm.h"
#include "can.h"

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

//电机组
static Motor_Group_HandleTypeDef hmotor_group_chassis = {
        .member = {&hmotor_chassis1, &hmotor_chassis2, &hmotor_chassis3, &hmotor_chassis4}
};
static Motor_Group_HandleTypeDef hmotor_group_fric = {
        .member = {&hmotor_fric_right, &hmotor_fric_left}
};
static Motor_Group_HandleTypeDef hmotor_group_trigger = {
        .member = {&hmotor_trigger}
};
static Motor_Group_HandleTypeDef hmotor_group_pitch = {
        .member = {&hmotor_pitch}
};

/*===| 电机系统初始化(创建对象 + PID整定 + 注册CAN节点) |===*/
void Motor_Init()
{
        static const uint16_t Motor_CAN_Feedback_ID_Table[9] = {

        };
}

/*===| 电机控制任务: "计算 → 发送" 两阶段 |===*/
void Motor_Control_Task(void *pvParameters)
{
        for (;;)
        {
                //计算
                for (int i = 0; i < Motor_Count; i++)
                {
                        //离线输出0力矩
                        if (Is_Err((Err_HandleTypeDef *) hmotor_table[i]))
                        {
                                Motor_Set_Torque(hmotor_table[i], 0);
                                continue;
                        }

                        if (hmotor_table[i]->Motor_Config_Struct.Is_Feedback_Control) //下位机闭环
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

                //发送
                for (int i = 0; i < Motor_Count; i++)
                {
                        if (hmotor_table[i]->Motor_Config_Struct.Is_Feedback_Control) //下位机闭环: 下发MCU算好的力矩
                        {
                                Motor_Send_Torque(hmotor_table[i], hmotor_table[i]->Target_Torque);
                        }
                        else //电调闭环: 目标直接下发
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

                //ps：f4的每个can控制器都只有3个邮箱，所以不可以一次塞入3个以上个包
                vTaskDelay(1);
        }
}
