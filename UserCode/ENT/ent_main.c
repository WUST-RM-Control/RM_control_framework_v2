//
// Created by ye on 2026/8/24.
//

#include "ent_main.h"

#include "drv_buzzer.h"
#include "drv_led.h"
#include "drv_remote.h"
#include "hal_can.h"
#include "hal_debug.h"
#include "hal_dwt.h"
#include "srv_error_monitor.h"
#include "srv_motor_control.h"
#include "fdcan.h"
#include "tim.h"
#include "usart.h"
#include "drv_vofa.h"
#include "drv_imu.h"
#include "srv_ins.h"

void Debug_Task(void *pvParameters)
{
        LED_Set(&hled1, 255, 255, 255);
        for (;;)
        {

                VOFA_Send_Data(0, Remote_Get_Left_X(&hremote_dt7));
                VOFA_Send_Data(1, Remote_Get_Left_Y(&hremote_dt7));
                VOFA_Send_Data(2, Remote_Get_Right_X(&hremote_dt7));
                VOFA_Send_Data(3, Remote_Get_Right_Y(&hremote_dt7));





                vTaskDelay(20);
        }
}

void main_init()
{

        //DWT周期计时(供冷却计时/电机速度计算使用)
        DWT_Init(SystemCoreClock / 1000000);

        //CAN外设启动与过滤器
        CAN_Init();

        //调试串口(printf → USART3)
        Debug_Init();

        VOFA_Init();

        //RGB LED(htim1 PWM通道1/2/3)
        LED_Ctor(&hled1, &htim1);
        LED_Init(&hled1);

        // IMU_Init(&himu1);

        //惯导(任务内初始化)
        // xTaskCreate(INS_Task, "INS", 1024, NULL, 5, NULL);

        //蜂鸣器(任务内初始化)
        xTaskCreate(Buzzer_Task, "Buzzer", 256, &hbuzzer1, 5, NULL);

        //遥控器(内部完成: 构造 + 注册UART空闲回调 + 启动DMA接收)
        Remote_Init(&huart1);

        //电机(任务内初始化 + PID控制循环)
        xTaskCreate(Motor_Control_Task, "Motor", 512, NULL, 6, NULL);

        //错误监控(10ms周期: 在线检测 + 故障聚合 + 指示上报)
        // xTaskCreate(Error_Monitor_Task, "ErrorMon", 512, NULL, 4, NULL);

        xTaskCreate(Debug_Task, "Debug", 128, NULL, 4, NULL);
}


