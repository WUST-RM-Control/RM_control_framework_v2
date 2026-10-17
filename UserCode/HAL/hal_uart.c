//
// Created by ye on 2026/8/29.
//

#include "hal_uart.h"

#include <string.h>

#include "FreeRTOS.h"
#include "stream_buffer.h"
#include "task.h"

#define UART_BUS_MAX         2
#define UART_RX_STREAM_SIZE  128
#define UART_TX_STREAM_SIZE  128
#define UART_RX_ACCUM_MAX    (UART_NODE_RX_MAX * 2)
#define UART_FRAME_MIN_LEN   4
#define UART_TX_TIMEOUT_MS   10
#define UART_RX_POLL_MS      10

#define UART_TX_TASK_PRIO 5
#define UART_RX_TASK_PRIO 5

typedef struct
{
        UART_HandleTypeDef *huart;

        StreamBufferHandle_t TxStream; //待发字节流
        StreamBufferHandle_t RxStream; //已收字节流

        UART_FrameParser FrameParser;

        TaskHandle_t TxTask; //DMA发完由中断通知

        volatile uint8_t RxPending; //重启接收失败(huart锁被发送占用), 待补

        uint8_t rxBuff[UART_NODE_RX_MAX];
} UART_Bus_StructTypeDef;

static UART_Bus_StructTypeDef UART_Bus_Table[UART_BUS_MAX];
static uint8_t                UART_Bus_Count = 0;

static UART_Node_HandleTypeDef *UART_Node_Table[UART_NODE_MAX];
static uint8_t                  UART_Node_Count = 0;

static UART_Bus_StructTypeDef *UART_Bus_Find(UART_HandleTypeDef *huart)
{
        for (uint8_t i = 0; i < UART_Bus_Count; i++)
        {
                if (UART_Bus_Table[i].huart == huart) return &UART_Bus_Table[i];
        }

        return NULL;
}

//DMA接收下任何错误(ORE/FE/NE)都会中止接收, 不重启就永久失联
static HAL_StatusTypeDef UART_Restart_Receive(UART_Bus_StructTypeDef *bus)
{
        HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(bus->huart, bus->rxBuff, sizeof(bus->rxBuff));

        if (status == HAL_OK) __HAL_DMA_DISABLE_IT(bus->huart->hdmarx, DMA_IT_HT);

        return status;
}

//只做字节入环, 分帧交给任务(处理粘包/半包)
static void UART_RxEvent_Callback(UART_HandleTypeDef *huart, uint16_t Pos)
{
        UART_Bus_StructTypeDef *bus = UART_Bus_Find(huart);
        if (bus == NULL) return;

        uint16_t pos = (Pos > sizeof(bus->rxBuff)) ? sizeof(bus->rxBuff) : Pos;

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        if (pos > 0) xStreamBufferSendFromISR(bus->RxStream, bus->rxBuff, pos, &xHigherPriorityTaskWoken);

        //huart锁可能正被DMA发送占用, 失败则标记待补偿(RxTask每10ms重试)
        if (UART_Restart_Receive(bus) != HAL_OK) bus->RxPending = 1;

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void UART_Error_Callback(UART_HandleTypeDef *huart)
{
        UART_Bus_StructTypeDef *bus = UART_Bus_Find(huart);
        if (bus == NULL) return;

        if (UART_Restart_Receive(bus) != HAL_OK) bus->RxPending = 1;
}

//按帧首地址分发到节点
static void UART_Dispatch(UART_Bus_StructTypeDef *bus, const uint8_t *data, uint16_t length)
{
        for (uint8_t i = 0; i < UART_Node_Count; i++)
        {
                UART_Node_HandleTypeDef *node = UART_Node_Table[i];
                if (node == NULL || node->Base.handler == NULL) continue;
                if (node->huart != bus->huart) continue;
                if (data[0] != (uint8_t) node->Base.Device_ID) continue;

                uint16_t len = (length > UART_NODE_RX_MAX) ? UART_NODE_RX_MAX : length;

                node->rxLength       = len;
                node->Base.Rx_Length = len;
                memcpy(node->rxData, data, len);

                node->Base.handler(&node->Base);
                return;
        }
}

static uint16_t UART_Frame_Parse(UART_Bus_StructTypeDef *bus, const uint8_t *data, uint16_t length)
{
        if (length < UART_FRAME_MIN_LEN) return 0;
        if (bus->FrameParser == NULL) return length; //未挂接解析器: 整段视为一帧

        return bus->FrameParser(data, length);
}

static void UART_RxTask(void *argument)
{
        UART_Bus_StructTypeDef *bus = (UART_Bus_StructTypeDef *) argument;

        uint8_t  accum[UART_RX_ACCUM_MAX];
        uint16_t length = 0;

        for (;;)
        {
                if (length == sizeof(accum)) length = 0;

                size_t received = xStreamBufferReceive(bus->RxStream, accum + length, sizeof(accum) - length, pdMS_TO_TICKS(UART_RX_POLL_MS));
                if (received == 0)
                {
                        //无数据时兜底: 补一次被发送占锁打断的接收重启
                        if (bus->RxPending && UART_Restart_Receive(bus) == HAL_OK) bus->RxPending = 0;
                        continue;
                }

                length += (uint16_t) received;

                uint16_t offset = 0;
                while (offset < length)
                {
                        uint16_t frame_len = UART_Frame_Parse(bus, accum + offset, length - offset);
                        if (frame_len == 0 || frame_len > length - offset) break;

                        UART_Dispatch(bus, accum + offset, frame_len);
                        offset += frame_len;
                }

                if (offset > 0)
                {
                        length -= offset;
                        if (length > 0) memmove(accum, accum + offset, length);
                }
        }
}

//DMA发送: 发起后阻塞等完成通知, 不忙等CPU
static void UART_TxTask(void *argument)
{
        UART_Bus_StructTypeDef *bus = (UART_Bus_StructTypeDef *) argument;
        uint8_t                 frame[UART_NODE_RX_MAX];

        for (;;)
        {
                size_t length = xStreamBufferReceive(bus->TxStream, frame, sizeof(frame), portMAX_DELAY);
                if (length == 0) continue;

                if (HAL_UART_Transmit_DMA(bus->huart, frame, (uint16_t) length) == HAL_OK)
                {
                        //超时则中止, 防完成回调丢失导致永久阻塞
                        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(UART_TX_TIMEOUT_MS)) == 0)
                        {
                                HAL_UART_AbortTransmit(bus->huart);
                        }
                }
                else
                {
                        HAL_UART_AbortTransmit(bus->huart); //状态异常, 复位后继续下一帧
                }

                if (bus->RxPending && UART_Restart_Receive(bus) == HAL_OK) bus->RxPending = 0;
        }
}

//DMA发送完成: 通知TxTask继续下一帧
static void UART_TxCplt_Callback(UART_HandleTypeDef *huart)
{
        UART_Bus_StructTypeDef *bus = UART_Bus_Find(huart);
        if (bus == NULL || bus->TxTask == NULL) return;

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(bus->TxTask, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void UART_Node_Register(UART_Node_HandleTypeDef *uart_node)
{
        if (uart_node == NULL || uart_node->Base.handler == NULL) return;

        for (uint8_t i = 0; i < UART_Node_Count; i++)
        {
                if (UART_Node_Table[i] == uart_node) return;
        }

        if (UART_Node_Count >= UART_NODE_MAX) return;

        UART_Node_Table[UART_Node_Count++] = uart_node;
}

void UART_Node_UnRegister(UART_Node_HandleTypeDef *uart_node)
{
        for (uint8_t i = 0; i < UART_Node_Count; i++)
        {
                if (UART_Node_Table[i] == uart_node)
                {
                        UART_Node_Count--;
                        UART_Node_Table[i] = UART_Node_Table[UART_Node_Count];
                        return;
                }
        }
}

void UART_Node_Ctor(UART_Node_HandleTypeDef *uart_node, UART_HandleTypeDef *huart, uint8_t Addr, Bus_Node_Handler handler, uint16_t err_tick_timeout, uint16_t err_count_maximum, err_handler err_handler)
{
        Bus_Node_Ctor(&uart_node->Base, BUS_TYPE_UART, Addr, handler, err_tick_timeout, err_count_maximum, err_handler);

        uart_node->huart    = huart;
        uart_node->rxLength = 0;

        uart_node->Base.Rx_Data   = uart_node->rxData;
        uart_node->Base.Rx_Length = 0;

        UART_Node_Register(uart_node);
}

HAL_StatusTypeDef UART_Node_Transmit(UART_Node_HandleTypeDef *uart_node, const uint8_t *TX_Data, uint16_t Length)
{
        return HAL_UART_Transmit(uart_node->huart, (uint8_t *) TX_Data, Length, UART_TX_TIMEOUT_MS);
}

void UART_Node_Transmit_Queue(UART_Node_HandleTypeDef *uart_node, const uint8_t *TX_Data, uint16_t Length)
{
        UART_Bus_StructTypeDef *bus = UART_Bus_Find(uart_node->huart);
        if (bus == NULL) return;

        xStreamBufferSend(bus->TxStream, TX_Data, Length, 0);
}

void UART_Bus_SetFrameParser(UART_HandleTypeDef *huart, UART_FrameParser parser)
{
        UART_Bus_StructTypeDef *bus = UART_Bus_Find(huart);
        if (bus == NULL) return;

        bus->FrameParser = parser;
}

void UART_Bus_Init(UART_HandleTypeDef *huart)
{
        if (UART_Bus_Count >= UART_BUS_MAX) return;
        if (UART_Bus_Find(huart) != NULL) return;

        UART_Bus_StructTypeDef *bus = &UART_Bus_Table[UART_Bus_Count++];

        bus->huart       = huart;
        bus->FrameParser = NULL;
        bus->TxStream    = xStreamBufferCreate(UART_TX_STREAM_SIZE, 1);
        bus->RxStream    = xStreamBufferCreate(UART_RX_STREAM_SIZE, 1);

        xTaskCreate(UART_TxTask, "UART_Tx", 256, bus, UART_TX_TASK_PRIO, &bus->TxTask);
        xTaskCreate(UART_RxTask, "UART_Rx", 256, bus, UART_RX_TASK_PRIO, NULL);

        HAL_UART_RegisterRxEventCallback(huart, UART_RxEvent_Callback);
        HAL_UART_RegisterCallback(huart, HAL_UART_ERROR_CB_ID, UART_Error_Callback);
        HAL_UART_RegisterCallback(huart, HAL_UART_TX_COMPLETE_CB_ID, UART_TxCplt_Callback);

        UART_Restart_Receive(bus);
}
