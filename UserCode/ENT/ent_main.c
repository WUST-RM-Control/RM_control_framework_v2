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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "vofa.h"
#include "drv_imu.h"
#include "drv_key.h"
#include "drv_remote_dt7.h"
#include "srv_ins.h"
#include "srv_led_key.h"

//开发时偷懒做点小事在这()
void Debug_Task(void *pvParameters)
{
        // uint8_t TX[] = "abc\r\n";
        LED_Set(&hled1, 255, 255, 255);
        for (;;)
        {

                VOFA_Send_Data(0, Key_Get_State(&hkey1));



                vTaskDelay(40);
        }
}

void main_init()
{
        /*===| 平台与外设启动 |===*/
        DWT_Init(170);                  //DWT周期计时(主频170MHz)

        Debug_Ctor(&huart3);            //printf重定向目标
        Debug_Init();

        VOFA_Ctor(&huart2);             //VOFA上位机串口
        VOFA_Init();

        CAN_Bus_Init(&hfdcan1);         //CAN总线启动 + 过滤器/接收中断
        CAN_Bus_Init(&hfdcan2);
        CAN_Bus_Init(&hfdcan3);

        /*===| 实例 ↔ 外设绑定(全部集中在此) |===*/
        Key_Ctor(&hkey1, GPIOB, GPIO_PIN_7);

        LED_Ctor(&hled1, &htim1);                             //RGB LED(htim1 PWM通道1/2/3)
        LED_Init(&hled1);

        Buzzer_Ctor(&hbuzzer1, &htim16, TIM_CHANNEL_1, 10);   //蜂鸣器(htim16 CH1, 10ms任务周期)
        Buzzer_Init(&hbuzzer1);

        IMU_Ctor(&himu1, &hspi1, SPI1_CS_GPIO_Port, SPI1_CS_Pin);   //QMI8658A(SPI1 + 片选引脚)
        IMU_Init(&himu1);

        //遥控器: 构造 + 启动接收
        Remote_Ctor(&hremote_dt7, &huart1, &Remote_DT7_VTable_Default,
                    REMOTE_OFFLINE_TIMEOUT, REMOTE_ERR_COUNT_MAX, Remote_Err_Handler);
        Remote_Init(&hremote_dt7);

        /*===| 服务初始化 |===*/
        INS_Init();                     //惯导(IMU参数 + 四元数EKF)
        Motor_Init();                   //9个电机对象 + PID整定 + CAN节点注册

        /*===| 任务创建 |===*/
        xTaskCreate(LED_Key_Task, "LED_Key", 256, NULL, 4, NULL);
        xTaskCreate(Buzzer_Task, "Buzzer", 256, &hbuzzer1, 4, NULL);
        xTaskCreate(Motor_Control_Task, "Motor", 512, NULL, 6, NULL);
        xTaskCreate(INS_Task, "INS", 1024, NULL, 10, NULL);
        // xTaskCreate(Error_Monitor_Task, "ErrorMon", 512, NULL, 5, NULL);

        xTaskCreate(Debug_Task, "Debug", 1024, NULL, 6, NULL);
        Buzzer_Set_SoundEffect(&hbuzzer1, Buzzer_SoundEffect_SystemStart);
}


