//
// Created by ye on 2026/8/26.
//

#ifndef G4MINI_V3_SRV_MOTOR_CONTROL_H
#define G4MINI_V3_SRV_MOTOR_CONTROL_H

#include "drv_motor.h"
#include "hal_can.h"

#define MOTOR_COUNT 9
#define ERR_COUNT_MAX 10
#define MOTOR_TIMEOUT 10

/*===| 电机对象实例 |===*/
extern Motor_HandleTypeDef hmotor_chassis1;
extern Motor_HandleTypeDef hmotor_chassis2;
extern Motor_HandleTypeDef hmotor_chassis3;
extern Motor_HandleTypeDef hmotor_chassis4;
extern Motor_HandleTypeDef hmotor_yaw;
extern Motor_HandleTypeDef hmotor_pitch;
extern Motor_HandleTypeDef hmotor_fric_right;
extern Motor_HandleTypeDef hmotor_fric_left;
extern Motor_HandleTypeDef hmotor_trigger;

extern Motor_HandleTypeDef *hmotor_table[MOTOR_COUNT];

/*===| 电机控制服务(初始化 + 控制循环, 位于SRV层) |===*/

//电机系统初始化: 创建电机对象 + PID整定 + 注册CAN节点
void Motor_Init(void);

//电机控制任务: PID计算 + 发送力矩
void Motor_Control_Task(void *pvParameters);

#endif //G4MINI_V3_SRV_MOTOR_CONTROL_H
