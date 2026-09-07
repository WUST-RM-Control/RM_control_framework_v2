//
// Created by ye on 2026/8/22.
//

#ifndef G4MINI_V3_HAL_CAN_H
#define G4MINI_V3_HAL_CAN_H

#include "stm32g4xx_hal.h"

/*===| CAN总线资源分配(各设备使用的总线与ID, 使用前需包含fdcan.h) |===*/
//底盘电机
#define CHASSIS_MOTOR_CAN                   hfdcan1  //底盘CAN
#define CHASSIS_MOTOR_SEND_CAN_ID           0x200    //底盘电机CAN发送ID
#define CHASSIS_MOTOR1_FEEDBACK_CAN_ID      0x201    //底盘电机1CAN反馈ID
#define CHASSIS_MOTOR2_FEEDBACK_CAN_ID      0x202    //底盘电机2CAN反馈ID
#define CHASSIS_MOTOR3_FEEDBACK_CAN_ID      0x203    //底盘电机3CAN反馈ID
#define CHASSIS_MOTOR4_FEEDBACK_CAN_ID      0x204    //底盘电机4CAN反馈ID

//云台电机
#define GIMBAL_YAW_CAN                      hfdcan2  //Yaw_使用的CAN
#define GIMBAL_YAW_SEND_CAN_ID              0x06     //Yaw_CAN发送ID
#define GIMBAL_YAW_FEEDBACK_CAN_ID          0x106    //Yaw_CAN反馈ID

#define GIMBAL_PITCH_CAN                    hfdcan3  //Pitch_使用的CAN
#define GIMBAL_PITCH_SEND_CAN_ID            0x1FE    //Pitch_CAN发送ID
#define GIMBAL_PITCH_FEEDBACK_CAN_ID        0x206    //Pitch_CAN反馈ID

//发射机构电机
#define SHOOT_FRIC_CAN                      hfdcan3  //Fric_使用的CAN
#define SHOOT_FRIC_SEND_CAN_ID              0x200    //Fric_CAN发送ID
#define SHOOT_FRIC_RIGHT_FEEDBACK_CAN_ID    0x201    //右Fric_CAN反馈ID
#define SHOOT_FRIC_LEFT_FEEDBACK_CAN_ID     0x202    //左Fric_CAN反馈ID

#define SHOOT_TRIGGER_CAN                   hfdcan2  //Trigger_使用的CAN
#define SHOOT_TRIGGER_SEND_CAN_ID           0x200    //Trigger_CAN发送ID
#define SHOOT_TRIGGER_FEEDBACK_CAN_ID       0x204    //Trigger_CAN反馈ID

//超电
#define SUPERCAP_CAN                        hfdcan1  //hcan1
#define SUPERCAP_CONTROL_CAN_ID             0x030    //超电控制CANID
#define SUPERCAP_FEEDBACK_CAN_ID            0x031    //超电反馈CANID

//陀螺仪(IMU)
#define IMU_CAN                             hfdcan3  //hcan3
#define IMU_CONTROL_CAN_ID                  0x001    //IMU控制CANID
#define IMU_FEEDBACK_CAN_ID                 0x002    //IMU反馈CANID

//遥控CAN转发
#define REMOTE_CAN                          hfdcan3
#define REMOTE_CAN_JOYSTIC_CAN_ID           0x21
#define REMOTE_CAN_KEYBOARDMOUSE_CAN_ID     0x22

/*===| CAN节点基类(所有CAN设备的公共字段) |===*/
typedef struct
{
        FDCAN_HandleTypeDef *hfdcan;          //FDCAN句柄
        uint16_t             CAN_Send_ID;     //发送ID
        uint16_t             CAN_Feedback_ID; //反馈ID
} CAN_Node_HandleTypeDef;

/*===| 基类访问接口 |===*/
__STATIC_INLINE FDCAN_HandleTypeDef *CAN_Node_Get_HFDCAN(CAN_Node_HandleTypeDef *node)
{
        return node->hfdcan;
}

__STATIC_INLINE uint16_t CAN_Node_Get_Send_ID(CAN_Node_HandleTypeDef *node)
{
        return node->CAN_Send_ID;
}

__STATIC_INLINE uint16_t CAN_Node_Get_Feedback_ID(CAN_Node_HandleTypeDef *node)
{
        return node->CAN_Feedback_ID;
}

//CAN节点构造(参考 LED_Ctor)
void CAN_Node_Ctor(CAN_Node_HandleTypeDef *node, FDCAN_HandleTypeDef *hfdcan, uint16_t CAN_Send_ID, uint16_t CAN_Feedback_ID);

/*===| CAN节点分发框架 |===*/

//节点数据回调: 收到匹配(总线+反馈ID)的帧时被调用
//注: 设备结构体需将 CAN_Node_HandleTypeDef Node 作为第一个成员, 回调内可将其转回设备指针
//    例如: Motor_HandleTypeDef *hmotor = (Motor_HandleTypeDef *)node;
typedef void (*CAN_Node_Handler)(CAN_Node_HandleTypeDef *node, const uint8_t *Data);

//注册节点(重复注册则更新回调)
void CAN_Node_Register(CAN_Node_HandleTypeDef *node, CAN_Node_Handler handler);

//注销节点
void CAN_Node_UnRegister(CAN_Node_HandleTypeDef *node);

/*===| CAN总线错误处理 |===*/

//总线离线计时(由监控任务周期调用: 每周期递增, 收到数据清零)
void CAN_Bus_Tick(void);

//查询总线在线状态(超过 CAN_OFFLINE_TICK 周期无数据视为离线)
uint8_t CAN_Get_Bus_Online(FDCAN_HandleTypeDef *hfdcan);

//查询总线硬件错误计数(由 HAL_FDCAN_ErrorCallback 累计)
uint16_t CAN_Get_Bus_ErrorCount(FDCAN_HandleTypeDef *hfdcan);

//总线离线判定阈值(监控周期10ms时为100ms)
#define CAN_OFFLINE_TICK 10

//CAN总线重启: 停止→去初始化→重新初始化→启动→重配过滤器/中断 (处理总线错误/总线关闭)
HAL_StatusTypeDef CAN_Bus_Restart(FDCAN_HandleTypeDef *hfdcan);

void CAN_Send_Data_STD(FDCAN_HandleTypeDef *hfdcan, uint16_t ID, const uint8_t *TX_Data);

void CAN_Send_Data_EXD(FDCAN_HandleTypeDef *hfdcan, uint32_t ID, uint8_t *TX_Data, uint8_t Length);

void CAN_Filter_Init(FDCAN_HandleTypeDef *hfdcan);

void CAN_Init(void);

#endif //G4MINI_V3_HAL_CAN_H
