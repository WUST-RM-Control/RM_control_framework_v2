//
// Created by ye on 2026/8/26.
//

#include "srv_error_monitor.h"

#include "drv_buzzer.h"
#include "drv_led.h"
#include "drv_motor.h"
#include "drv_remote.h"
#include "hal_can.h"
#include "fdcan.h"

#include <stdio.h>

#include "usart.h"

/*===| 监控参数 |===*/
#define MONITOR_PERIOD_MS  10                      //监控周期
#define CAN_OFFLINE_MS     100                     //CAN总线超时
#define MOTOR_OFFLINE_MS   100                     //电机超时
#define REMOTE_OFFLINE_MS  100                     //遥控超时
#define VT03_OFFLINE_MS    200                     //图传超时
#define MOTOR_OVERTEMP_C   80                      //电机过温阈值(仅指示)

#define TICK_OF(ms)        (((ms) + MONITOR_PERIOD_MS - 1) / MONITOR_PERIOD_MS)


/*===| 恢复参数 |===*/
#define CAN_RESTART_THRESHOLD   5    //CAN错误计数重启阈值
#define MOTOR_RECOVER_MS        100  //电机恢复重试间隔

//若不在错误后重启接收, 遥控器将永久失联(回调再也进不来)
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
        if (huart == &huart1)
        {
                Remote_Restart_Receive(huart);
        }
}


Fault_Status_TypeDef hfault;

static void Error_Recover(void);

static void Error_Indicate(void);

static void Error_Print(void);

void Error_Monitor_Task(void *pvParameters)
{
        for (;;)
        {
                //1. CAN在线计时
                CAN_Bus_Tick();

                for (uint8_t i = 0; i < MOTOR_COUNT; i++)
                        Motor_Online_Check(hmotor[i], TICK_OF(MOTOR_OFFLINE_MS));

                Remote_Online_Check(&hremote_dt7, TICK_OF(REMOTE_OFFLINE_MS));
                Remote_Online_Check(&hremote_vt03, TICK_OF(VT03_OFFLINE_MS));

                //2. 故障聚合
                hfault.Fault_Bitmap_Last = hfault.Fault_Bitmap;
                hfault.Fault_Bitmap      = FAULT_NONE;

                if (!CAN_Get_Bus_Online(&hfdcan1)) hfault.Fault_Bitmap |= FAULT_CAN1_OFFLINE;
                if (!CAN_Get_Bus_Online(&hfdcan2)) hfault.Fault_Bitmap |= FAULT_CAN2_OFFLINE;
                if (!CAN_Get_Bus_Online(&hfdcan3)) hfault.Fault_Bitmap |= FAULT_CAN3_OFFLINE;

                for (uint8_t i = 0; i < MOTOR_COUNT; i++)
                {
                        if (!hmotor[i]->If_Online) hfault.Fault_Bitmap |= FAULT_MOTOR_OFFLINE;
                        if (hmotor[i]->Temperature >= MOTOR_OVERTEMP_C) hfault.Fault_Bitmap |= FAULT_MOTOR_OVERTEMP;
                        if (hmotor[i]->PID_Angle_Struct.ERRORHandler.ERRORType == Motor_Blocked ||
                            hmotor[i]->PID_Speed_Struct.ERRORHandler.ERRORType == Motor_Blocked)
                                hfault.Fault_Bitmap |= FAULT_MOTOR_BLOCKED;
                }

                if (!hremote_dt7.If_Connect) hfault.Fault_Bitmap |= FAULT_REMOTE_DISCONNECT;
                if (!hremote_vt03.If_Connect) hfault.Fault_Bitmap |= FAULT_VT03_DISCONNECT;

                //2.5 恢复动作(CAN重启 + 电机重新使能)
                Error_Recover();

                //3. 指示与上报
                Error_Indicate();
                Error_Print();

                vTaskDelay(MONITOR_PERIOD_MS);
        }
}

/*===| CAN总线错误重启,电机重新使能 |===*/
static void Error_Recover(void)
{
        //1. CAN总线错误重启: 检测到错误(计数≥阈值)直接重启
        //   CAN_Bus_Restart 内部会清零错误计数, 天然限频(需重新累计到阈值才会再次重启)
        for (uint8_t bus = 0; bus < 3; bus++)
        {
                FDCAN_HandleTypeDef *hfdcan = (bus == 0) ? &hfdcan1 : (bus == 1) ? &hfdcan2 : &hfdcan3;

                if (CAN_Get_Bus_ErrorCount(hfdcan) >= CAN_RESTART_THRESHOLD)
                {
                        printf("[Recover] CAN%d restart (Err=%d)\r\n", bus + 1, CAN_Get_Bus_ErrorCount(hfdcan));
                        CAN_Bus_Restart(hfdcan);
                }
        }

        //2. 电机恢复: 离线超时重启(所有电机) + DM失能重新使能(Error_Code==0), 每100ms一次, 无限重试
        static uint8_t Recover_Counter = 0;
        if (++Recover_Counter >= TICK_OF(MOTOR_RECOVER_MS))
        {
                Recover_Counter = 0;

                for (uint8_t i = 0; i < MOTOR_COUNT; i++)
                {
                        //统一处理: 离线 或 失能(Error_Code==0) 时恢复
                        //DM: ClearErr+Enable; DJI: 空操作(无害); 并清除PID积分防止恢复后饱和
                        if (!hmotor[i]->If_Online || hmotor[i]->Error_Code == 0)
                                Motor_Recover(hmotor[i]);
                }
        }
}

/*===| 故障指示: LED常亮(绿=正常/红=故障) + 蜂鸣器(故障发生瞬间响一次) |===*/
static void Error_Indicate(void)
{
        if (hfault.Fault_Bitmap == FAULT_NONE || hfault.Fault_Bitmap == FAULT_VT03_DISCONNECT)
        {
                LED_Set(&hled1, 0, 255, 0); //正常: 绿
        }
        else
        {
                LED_Set(&hled1, 255, 0, 0); //故障: 红

                if (hfault.Fault_Bitmap_Last == FAULT_NONE)
                        Buzzer_Set_SoundEffect(&hbuzzer1, Buzzer_SoundEffect_Error);
        }
}

