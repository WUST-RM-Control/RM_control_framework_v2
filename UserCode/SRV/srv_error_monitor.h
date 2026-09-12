//
// Created by ye on 2026/8/26.
//

#ifndef G4MINI_V3_SRV_ERROR_MONITOR_H
#define G4MINI_V3_SRV_ERROR_MONITOR_H

#include "main.h"

#define ERR_MAX 255

extern uint8_t Fault_Bitmap;

typedef struct Err_HandleTypeDef Err_HandleTypeDef;

typedef void (*Err_Handler)(Err_HandleTypeDef *herror);

struct Err_HandleTypeDef{
        volatile uint16_t tick;      //当前时间
        uint16_t tick_timeout;       //超时时间

        volatile uint16_t count;     //当前错误次数
        uint16_t count_maximum;      //错误次数上限

        volatile uint8_t If_Online;  //是否在线

        Err_Handler handler;         //错误处理回调
};

extern void Err_Register(Err_HandleTypeDef *herr, Err_Handler handler);
extern void Err_UnRegister(Err_HandleTypeDef *herr);
extern void Err_Ctor(Err_HandleTypeDef *herr, uint16_t err_tick_Timeout, uint16_t err_count_maximum, Err_Handler handler);

//错误监控任务(周期10ms): 在线计时 + 超时判离线 + 故障聚合 + 蜂鸣器/LED指示 + printf上报
void Error_Monitor_Task(void *pvParameters);

#endif //G4MINI_V3_SRV_ERROR_MONITOR_H
