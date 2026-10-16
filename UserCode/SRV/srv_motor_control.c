//
// Created by ye on 2026/8/26.
//

#include "srv_motor_control.h"

#include "can.h"
#include "usart.h"
#include "drv_motor_dji.h"
#include "drv_motor_dm.h"
#include "drv_motor_zdt_emm_V5.h"

/*===| 电机对象实例 |===*/
#define MOTOR_COUNT 2

Motor_DJI_HandleTypeDef hmotor1     = {};
Motor_ZDT_HandleTypeDef hmotor_zdt1 = {};

static Motor_Base_HandleTypeDef *hmotor_table[MOTOR_COUNT] = {
        &hmotor1.hmotor_base,
        &hmotor_zdt1.hmotor_base,
};

static Motor_DJIGroup_HandleTypeDef hmotor_group_chassis;

/*===| 电机系统初始化: 构造对象 + 建组 |===*/
void Motor_Init()
{
        //发送ID/反馈ID/地址按实际接线改
        Motor_DJI_Ctor(&hmotor1, &hcan1, 0x1FF, 0x201);
        Motor_DJI_Group_Init(&hmotor_group_chassis, &hmotor1, NULL, NULL, NULL);

        Motor_ZDT_Ctor(&hmotor_zdt1, &huart1, 1);
}

/*===| 电机控制任务: "计算 → 发送" |===*/
void Motor_Control_Task(void *pvParameters)
{
        for (;;)
        {
                for (int i = 0; i < Motor_Count; i++)
                {
                        Motor_Base_HandleTypeDef *hmotor = hmotor_table[i];

                        if (Is_Err(&hmotor->Link.Node.herr))
                        {
                                Motor_Set_Torque(hmotor, 0);
                        }
                        else
                        {
                                Motor_Update(hmotor);
                        }

                        Motor_Send(hmotor);
                }

                vTaskDelay(1);
        }
}
