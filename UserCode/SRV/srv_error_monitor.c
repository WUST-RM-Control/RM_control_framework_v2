//
// Created by ye on 2026/8/26.
//

#include "srv_error_monitor.h"
#include "drv_buzzer.h"
#include "drv_led.h"
#include "drv_motor.h"
#include "drv_remote.h"
#include <stdio.h>

/*===| 监控参数 |===*/
#define MONITOR_PERIOD_MS  10  //监控周期

//错误聚合
uint8_t Is_Fault = 0;


void Error_Monitor_Task(void *pvParameters)
{
        for (;;)
        {
                //遍历注册表: 推进计时 + 聚合故障(机制由 Common/err 提供)
                for (uint8_t i = 0; i < Err_Get_Count(); i++)
                {
                        Err_HandleTypeDef *herr = Err_Get_Handle(i);
                        if (herr == NULL || herr->handler == NULL) continue;

                        herr->handler(herr);
                        Is_Fault |= Is_Err(herr);
                }

                if (Is_Fault != 0) Buzzer_Set_SoundEffect(&hbuzzer1, Buzzer_SoundEffect_Error);//只哔哔不处理
                vTaskDelay(MONITOR_PERIOD_MS);
        }
}


// LED_Set(&hled1, 0, 255, 0); //正常: 绿
//
// LED_Set(&hled1, 255, 0, 0); //故障: 红
//
// Buzzer_Set_SoundEffect(&hbuzzer1, Buzzer_SoundEffect_Error);