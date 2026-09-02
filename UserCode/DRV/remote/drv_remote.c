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
#include "usart.h"


Remote_HandleTypeDef hremote_dt7;  //DT7  遥控器
Remote_HandleTypeDef hremote_vt03; //VT03 图传


//uart1接收缓冲与回调(模块私有)
uint8_t Remote_RxBuff[64] = {};

#define REMOTE_RX_FRAME_LEN 18    //DT7 一帧长度
#define REMOTE_RX_QUEUE_LEN  4    //队列深度(帧率~100Hz, 4深足以缓冲)

static QueueHandle_t Remote_RxQueue;

static void Remote_RxTask(void *argument)
{
        uint8_t frame[REMOTE_RX_FRAME_LEN];

        for (;;)
        {
                if (xQueueReceive(Remote_RxQueue, frame, portMAX_DELAY) == pdPASS)
                {
                        //临界区保护
                        taskENTER_CRITICAL();
                        Remote_Rx_Handle(&hremote_dt7, frame); //DT7协议解包
                        taskEXIT_CRITICAL();
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
 * 由 HAL_UART_ErrorCallback (srv_error_monitor.c) 调用 */
void Remote_Restart_Receive(UART_HandleTypeDef *huart)
{
        HAL_UARTEx_ReceiveToIdle_DMA(huart, Remote_RxBuff, sizeof(Remote_RxBuff));
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
}

void Remote_Init(UART_HandleTypeDef *huart)
{
        Remote_DT7_Ctor(&hremote_dt7, huart);
        Remote_VT03_Ctor(&hremote_vt03, NULL);

        Remote_RxQueue = xQueueCreate(REMOTE_RX_QUEUE_LEN, REMOTE_RX_FRAME_LEN);
        if (Remote_RxQueue != NULL)
        {
                xTaskCreate(Remote_RxTask, "RemoteRx", 512, NULL, 7, NULL);
        }

        //注册UART空闲中断回调
        HAL_UART_RegisterRxEventCallback(huart, Remote_RxEvent_CallBack);
        HAL_UARTEx_ReceiveToIdle_DMA(huart, Remote_RxBuff, sizeof(Remote_RxBuff));
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
}
