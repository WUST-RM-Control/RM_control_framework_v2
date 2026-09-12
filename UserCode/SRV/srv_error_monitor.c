//
// Created by ye on 2026/8/26.
//

#include "srv_error_monitor.h"
#include "drv_buzzer.h"
#include "drv_led.h"
#include "drv_motor.h"
#include "drv_remote.h"
#include <stdio.h>
#include "usart.h"

/*===| 监控参数 |===*/
#define MONITOR_PERIOD_MS  10  //监控周期

//错误聚合位图
uint8_t Fault_Bitmap = 0;

//若不在错误后重启接收, 遥控器将失联
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
        if (huart == &huart1)
        {
                Remote_Restart_Receive(huart);
        }
}


/*===| 错误分发框架(基于 herr 基类) |===*/
static Err_HandleTypeDef *herr_table[ERR_MAX];
static uint8_t           herr_count = 0;

//错误处理对象注册
void Err_Register(Err_HandleTypeDef *herr, Err_Handler handler)
{
        if (herr == NULL || handler == NULL) return;

        for (uint8_t i = 0; i < herr_count; i++)
        {
                if (herr_table[i] == herr)
                {
                        herr_table[i]->handler = handler;
                        return;
                }
        }

        if (herr_count >= ERR_MAX) return;

        herr_table[herr_count]          = herr;
        herr_table[herr_count]->handler = handler;
        herr_count++;
}

//错误处理对象注销
void Err_UnRegister(Err_HandleTypeDef *herr)
{
        for (uint8_t i = 0; i < herr_count; i++)
        {
                if (herr_table[i] == herr)
                {
                        herr_count--;
                        herr_table[i] = herr_table[herr_count];
                        return;
                }
        }
}

//错误处理对象构造
void Err_Ctor(Err_HandleTypeDef *herr, uint16_t err_tick_Timeout, uint16_t err_count_maximum, Err_Handler handler)
{
        herr->tick          = 0;
        herr->tick_timeout  = err_tick_Timeout;

        herr->count         = 0;
        herr->count_maximum = err_count_maximum;

        herr->If_Online     = 0;
        Err_Register(herr, handler);
}


void Error_Monitor_Task(void *pvParameters)
{
        for (;;)
        {
                for (uint8_t i = 0; i < herr_count; i++)
                {
                        herr_table[i]->handler(herr_table[i]);
                }

                if (Fault_Bitmap != 0) Buzzer_Set_SoundEffect(&hbuzzer1, Buzzer_SoundEffect_Error);//只哔哔不处理
                vTaskDelay(MONITOR_PERIOD_MS);
        }
}

// /*===| CAN总线错误重启,电机重新使能 |===*/
// static void Error_Recover(void)
// {
//         //1. CAN总线错误重启: 检测到错误(计数≥阈值)直接重启
//         //   CAN_Bus_Restart 内部会清零错误计数, 天然限频(需重新累计到阈值才会再次重启)
//         for (uint8_t bus = 0; bus < 3; bus++)
//         {
//                 FDCAN_HandleTypeDef *hfdcan = (bus == 0) ? &hfdcan1 : (bus == 1) ? &hfdcan2 : &hfdcan3;
//
//                 if (CAN_Get_Bus_ErrorCount(hfdcan) >= CAN_RESTART_THRESHOLD)
//                 {
//                         CAN_Bus_Restart(hfdcan);
//                 }
//         }
//
//         //2. 电机恢复: 离线超时重启(所有电机) + DM失能重新使能(Error_Code==0), 每100ms一次, 无限重试
//         static uint8_t Recover_Counter = 0;
//         if (++Recover_Counter >= TICK_OF(MOTOR_RECOVER_MS))
//         {
//                 Recover_Counter = 0;
//
//                 for (uint8_t i = 0; i < MOTOR_COUNT; i++)
//                 {
//                         if (!hmotor_table[i]->If_Online || hmotor_table[i]->Error_Code == 0)
//                                 Motor_Recover(hmotor_table[i]);
//                 }
//         }
// }


// LED_Set(&hled1, 0, 255, 0); //正常: 绿
//
// LED_Set(&hled1, 255, 0, 0); //故障: 红
//
// Buzzer_Set_SoundEffect(&hbuzzer1, Buzzer_SoundEffect_Error);