//
// Created by ye on 2026/8/26.
//

#include "drv_remote.h"

#include <stdio.h>
#include <string.h>

#include "drv_remote_dt7.h"
#include "drv_remote_vt03.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

Remote_HandleTypeDef hremote_dt7; //DT7  遥控器
// Remote_HandleTypeDef hremote_vt03; //VT03 图传


//uart1接收缓冲与回调(模块私有)
static uint8_t Remote_RxBuff[64] = {};

#define REMOTE_RX_FRAME_LEN 18    //DT7 一帧长度
#define REMOTE_RX_QUEUE_LEN  4    //队列深度(帧率~100Hz, 4深足以缓冲)

static QueueHandle_t Remote_RxQueue;

static void Remote_RxTask(void *argument)
{
        Remote_HandleTypeDef *hremote = (Remote_HandleTypeDef *)argument;
        uint8_t frame[REMOTE_RX_FRAME_LEN];

        for (;;)
        {
                if (xQueueReceive(Remote_RxQueue, frame, portMAX_DELAY) == pdPASS)
                {
                        Remote_Rx_Handle(hremote, frame); //协议解包
                }
        }
}

static void Remote_RxEvent_CallBack(UART_HandleTypeDef *huart, uint16_t Pos)
{
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        // if (Pos == REMOTE_RX_FRAME_LEN) //收到完整一帧
        {
                xQueueSendFromISR(Remote_RxQueue, Remote_RxBuff, &xHigherPriorityTaskWoken);
        }

        //重新开启下一次接收(必须在ISR内完成, 否则丢帧)
        HAL_UARTEx_ReceiveToIdle_DMA(huart, Remote_RxBuff, sizeof(Remote_RxBuff));
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* UART错误后重启接收: DMA接收模式下任何错误(ORE/FE/NE)都会被HAL视为阻塞错误并中止接收
 * 若不在错误后重启接收, 遥控器将永久失联(回调再也进不来)
 * 由 Remote_UART_Error_Callback 调用(下一个函数) */
static void Remote_Restart_Receive(UART_HandleTypeDef *huart)
{
        HAL_UARTEx_ReceiveToIdle_DMA(huart, Remote_RxBuff, sizeof(Remote_RxBuff));
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
}

/* UART错误回调: 由 Remote_Init 注册到 hremote->huart(per-handle, 支持多串口设备互不干扰)
 * 只依赖 huart 参数, 不引用具体实例, 因此随 ENT 装配变化 */
static void Remote_UART_Error_Callback(UART_HandleTypeDef *huart)
{
        Remote_Restart_Receive(huart);
}

void Remote_Err_Handler(Err_HandleTypeDef *herr)
{
        herr->tick++;

        //超时未收到数据 → 失联
        if (herr->tick > herr->tick_timeout) herr->If_Err = 1;

        //失联持续则累计错误次数(仅计数, 不做处理)
        if (herr->count < herr->count_maximum)
        {
                if (herr->tick > 2 * herr->tick_timeout)
                {
                        herr->count++;
                        herr->tick = herr->tick_timeout;
                }
        }
}

/*===| 遥控器通用构造: 绑定串口句柄 + vtable + 错误检测参数(参考 Motor_Ctor) |===*/
void Remote_Ctor(Remote_HandleTypeDef *hremote, UART_HandleTypeDef *huart, Remote_VTable *Remote_VTable, uint16_t err_tick_timeout, uint16_t err_count_maximum, err_handler err_handler)
{
        memset(hremote, 0, sizeof(Remote_HandleTypeDef));

        hremote->huart = huart;
        hremote->vptr  = Remote_VTable;

        Err_Ctor(&hremote->herr, err_tick_timeout, err_count_maximum, err_handler);
}

/*===| 启动遥控器接收: 队列 + 接收任务 + 空闲回调 + DMA
 * 对象(实例 ↔ 串口句柄/协议vtable 的绑定)由 ENT 调用 Remote_Ctor 完成 |===*/
void Remote_Init(Remote_HandleTypeDef *hremote)
{
        if (Remote_RxQueue != NULL) return; //已启动过, 避免重复创建队列/任务

        Remote_RxQueue = xQueueCreate(REMOTE_RX_QUEUE_LEN, REMOTE_RX_FRAME_LEN);
        if (Remote_RxQueue != NULL)
        {
                xTaskCreate(Remote_RxTask, "RemoteRx", 512, hremote, 7, NULL);
        }

        //注册UART空闲中断回调
        HAL_UART_RegisterRxEventCallback(hremote->huart, Remote_RxEvent_CallBack);

        //注册UART错误回调(per-handle, 每个串口设备各自一份)
        HAL_UART_RegisterCallback(hremote->huart, HAL_UART_ERROR_CB_ID, Remote_UART_Error_Callback);

        HAL_UARTEx_ReceiveToIdle_DMA(hremote->huart, Remote_RxBuff, sizeof(Remote_RxBuff));
        __HAL_DMA_DISABLE_IT(hremote->huart->hdmarx, DMA_IT_HT);
}
