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
#include "can.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "vofa.h"
#include "drv_imu.h"
#include "drv_bmi088.h"
#include "drv_key.h"
#include "drv_motor_zdt_emm_V5.h"
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

                VOFA_Send_Data(0, INS_Get_Pitch());
                VOFA_Send_Data(1, INS_Get_Yaw());
                VOFA_Send_Data(2, INS_Get_Roll());
                // Motor_Send_Angle(&hmotor1, 1000);
                // Emm_V5_Pos_Control(&hmotor1, 0, 1000, 0, 32000, 0, 0);

                vTaskDelay(40);

                // Motor_Send_Angle(&hmotor1, 0);

                // vTaskDelay(1000);

        }
}

void main_init()
{
        /*===| 平台与外设启动 |===*/
        DWT_Init(170);                  //DWT周期计时(主频170MHz)

        Debug_Ctor(&huart3);            //printf重定向目标
        Debug_Init();

        VOFA_Ctor(&huart6);             //VOFA上位机串口
        VOFA_Init();

        CAN_Init();

        /*===| 实例 ↔ 外设绑定(全部集中在此) |===*/
        Key_Ctor(&hkey1, GPIOA, GPIO_PIN_0);

        LED_Ctor(&hled1, &htim5);                             //RGB LED(htim1 PWM通道1/2/3)
        LED_Init(&hled1);

        Buzzer_Ctor(&hbuzzer1, &htim4, TIM_CHANNEL_3, 10);   //蜂鸣器(htim16 CH1, 10ms任务周期)
        Buzzer_Init(&hbuzzer1);

        IMU_Ctor(&himu1, IMU_BMI088_Config_Default);   //BMI088(SPI1 + CS1_ACCEL + CS1_GYRO)
        IMU_Init(&himu1);

        //遥控器: 构造 + 启动接收
        // Remote_Ctor(&hremote_dt7, &huart1, &Remote_DT7_VTable_Default,
        //             REMOTE_OFFLINE_TIMEOUT, REMOTE_ERR_COUNT_MAX, Remote_Err_Handler);
        // Remote_Init(&hremote_dt7);

        /*===| 服务初始化 |===*/
        INS_Init();
        Motor_Init();

        /*===| 任务创建 |===*/
        xTaskCreate(LED_Key_Task, "LED_Key", 256, NULL, 4, NULL);
        // xTaskCreate(Buzzer_Task, "Buzzer", 256, &hbuzzer1, 4, NULL);
        xTaskCreate(Motor_Control_Task, "Motor", 512, NULL, 6, NULL);
        xTaskCreate(INS_Task, "INS", 1024, NULL, 10, NULL);
        // xTaskCreate(Error_Monitor_Task, "ErrorMon", 512, NULL, 5, NULL);

        xTaskCreate(Debug_Task, "Debug", 1024, NULL, 6, NULL);
        Buzzer_Set_SoundEffect(&hbuzzer1, Buzzer_SoundEffect_SystemStart);
}


