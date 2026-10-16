//
// Created by ye on 2026/8/26.
//

#include "srv_motor_control.h"

#include "drv_motor_dji.h"
#include "drv_motor_dm.h"
#include "can.h"
#include "drv_motor_zdt_emm_V5.h"

/*===| 电机对象实例 |===*/
#define MOTOR_COUNT 1

Motor_Base_HandleTypeDef hmotor1   = {};


static Motor_Base_HandleTypeDef *hmotor_table[MOTOR_COUNT] = {
        &hmotor1,
};

//电机组
// static Motor_Group_HandleTypeDef hmotor_group_chassis = {
//         .member = {&hmotor1}
// };


/*===| 电机系统初始化(创建对象 + PID整定 + 注册CAN节点) |===*/
void Motor_Init()
{
        // static const uint16_t Motor_CAN_Feedback_ID_Table[MOTOR_COUNT] = {
        //
        // };
        // Motor_Ctor(&hmotor1, &hcan1, 0x01, 0x01 << 8, Motor_ZDT_Config_Default);
}

/*===| 电机控制任务: "计算 → 发送" 两阶段 |===*/
void Motor_Control_Task(void *pvParameters)
{
        for (;;)
        {
                for (int i = 0; i < Motor_Count; i++)
                {
                        Motor_Base_HandleTypeDef *hmotor = hmotor_table[i];

                        if (Is_Err(&hmotor->Node.herr))
                        {
                                Motor_Set_Torque(hmotor, 0);
                        }
                }

                for (int i = 0; i < Motor_Count; i++)
                {
                        Motor_Base_HandleTypeDef *hmotor = hmotor_table[i];

                        if (hmotor->Status_Enum == MOTOR_ANGLE)      Motor_Send_Angle(hmotor, hmotor->Target_Angle);
                        else if (hmotor->Status_Enum == MOTOR_SPEED) Motor_Send_Speed(hmotor, hmotor->Target_Speed);
                        else                                         Motor_Send_Torque(hmotor, hmotor->Target_Torque);
                }

                vTaskDelay(1);
        }
}
