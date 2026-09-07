//
// Created by ye on 2026/8/26.
//

#ifndef G4MINI_V3_SRV_ERROR_MONITOR_H
#define G4MINI_V3_SRV_ERROR_MONITOR_H

#include "main.h"

/*===| 故障类型(位图) |===*/
typedef enum
{
        FAULT_NONE              = 0x00000000u,
        FAULT_CAN1_OFFLINE      = 0x00000001u, //CAN1总线离线
        FAULT_CAN2_OFFLINE      = 0x00000002u, //CAN2总线离线
        FAULT_CAN3_OFFLINE      = 0x00000004u, //CAN3总线离线
        FAULT_MOTOR_OFFLINE     = 0x00000008u, //有电机掉线(详情看各电机If_Online)
        FAULT_MOTOR_BLOCKED     = 0x00000010u, //有电机堵转
        FAULT_MOTOR_OVERTEMP    = 0x00000020u, //有电机过温(仅指示, 不处理)
        FAULT_REMOTE_DISCONNECT = 0x00000040u, //遥控器断开
        FAULT_VT03_DISCONNECT   = 0x00000080u, //图传断开
} Fault_TypeDef;

/*===| 故障状态(全局) |===*/
typedef struct
{
        uint32_t Fault_Bitmap;      //当前故障位图
        uint32_t Fault_Bitmap_Last; //上一周期故障位图(用于边沿检测)
} Fault_Status_TypeDef;

extern Fault_Status_TypeDef hfault;

//错误监控任务(周期10ms): 在线计时 + 超时判离线 + 故障聚合 + 蜂鸣器/LED指示 + printf上报
void Error_Monitor_Task(void *pvParameters);

#endif //G4MINI_V3_SRV_ERROR_MONITOR_H
