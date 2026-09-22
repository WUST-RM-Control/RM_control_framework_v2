//
// Created by ye on 2026/8/26.
// 错误监控服务(策略层): 周期推进各错误对象的计时, 聚合故障并告警
// 错误对象/注册表等机制见 Common/err.h
//

#ifndef G4MINI_V3_SRV_ERROR_MONITOR_H
#define G4MINI_V3_SRV_ERROR_MONITOR_H

#include "err.h"

extern uint8_t Is_Fault;

//错误监控任务
void Error_Monitor_Task(void *pvParameters);

#endif //G4MINI_V3_SRV_ERROR_MONITOR_H
