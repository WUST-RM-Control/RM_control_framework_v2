//
// Created by ye on 2026/8/22.
//

#ifndef G4MINI_V3_HAL_CAN_H
#define G4MINI_V3_HAL_CAN_H

#include "stm32g4xx_hal.h"

////////////////////////////////////////////////////////////////////
//                                                               //
// 修改了CANID后接收数据不用动，需要在发送函数处根据ID重新排序 //
//                                                               //
////////////////////////////////////////////////////////////////////
/*===| 底盘电机通信相关定义 |===*/
#define CHASSIS_MOTOR_CAN                   hfdcan1  //底盘CAN
#define CHASSIS_MOTOR_SEND_CAN_ID           0x200    //底盘电机CAN发送ID
#define CHASSIS_MOTOR1_FEEDBACK_CAN_ID      0x201    //底盘电机1CAN反馈ID
#define CHASSIS_MOTOR2_FEEDBACK_CAN_ID      0x202    //底盘电机2CAN反馈ID
#define CHASSIS_MOTOR3_FEEDBACK_CAN_ID      0x203    //底盘电机3CAN反馈ID
#define CHASSIS_MOTOR4_FEEDBACK_CAN_ID      0x204    //底盘电机4CAN反馈ID

/*===| 云台电机通信相关定义 |===*/
#define GIMBAL_YAW_CAN                      hfdcan2  //Yaw_使用的CAN
#define GIMBAL_YAW_SEND_CAN_ID              0x06     //Yaw_CAN发送ID
#define GIMBAL_YAW_FEEDBACK_CAN_ID          0x106    //Yaw_CAN反馈ID


#define GIMBAL_PITCH_CAN                    hfdcan3  //Pitch_使用的CAN
#define GIMBAL_PITCH_SEND_CAN_ID            0x1FE    //Pitch_CAN发送ID
#define GIMBAL_PITCH_FEEDBACK_CAN_ID        0x206    //Pitch_CAN反馈ID

/*===| 发射电机通信相关定义 |===*/
#define SHOOT_FRIC_CAN                      hfdcan3  //Fric_使用的CAN
#define SHOOT_FRIC_SEND_CAN_ID              0x200    //Fric_CAN发送ID
#define SHOOT_FRIC_RIGHT_FEEDBACK_CAN_ID    0x201    //右Fric_CAN反馈ID
#define SHOOT_FRIC_LEFT_FEEDBACK_CAN_ID     0x202    //左Fric_CAN反馈ID

#define SHOOT_TRIGGER_CAN                   hfdcan2  //Trigger_使用的CAN
#define SHOOT_TRIGGER_SEND_CAN_ID           0x200    //Trigger_CAN发送ID
#define SHOOT_TRIGGER_FEEDBACK_CAN_ID       0x204    //Trigger_CAN反馈ID

/*===| 超电通信相关定义 |==*/
#define SUPERCAP_CAN                        hfdcan1  //hcan1
#define SUPERCAP_CONTROL_CAN_ID             0x030    //超电控制CANID
#define SUPERCAP_FEEDBACK_CAN_ID            0x031    //超电反馈CANID

/*===| 陀螺仪通信相关定义 |==*/
#define IMU_CAN                             hfdcan3  //hcan3
#define IMU_CONTROL_CAN_ID                  0x001    //IMU控制CANID
#define IMU_FEEDBACK_CAN_ID                 0x002    //IMU反馈CANID

/*===| 遥控CAN转发通信相关定义 |==*/
#define REMOTE_CAN                          hfdcan3
#define REMOTE_CAN_JOYSTIC_CAN_ID           0x21
#define REMOTE_CAN_KEYBOARDMOUSE_CAN_ID     0x22

void CAN_Send_Data_STD(FDCAN_HandleTypeDef *hfdcan, uint16_t ID, const uint8_t *TX_Data);

void CAN_Send_Data_EXD(FDCAN_HandleTypeDef *hfdcan, uint32_t ID, uint8_t *TX_Data, uint8_t Length);

void CAN_Filter_Init(FDCAN_HandleTypeDef *hfdcan);

void CAN_Init(void);

#endif //G4MINI_V3_HAL_CAN_H
