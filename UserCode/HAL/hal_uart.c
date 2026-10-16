//
// Created by ye on 2026/8/29.
//

#include "hal_uart.h"

#include <string.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#define UART_BUS_MAX       2
#define UART_RX_QUEUE_LEN  8
#define UART_TX_QUEUE_LEN  16
#define UART_FRAME_MIN_LEN 4
#define UART_TX_TIMEOUT    100

typedef struct
{
        UART_HandleTypeDef *huart;
        uint8_t             data[UART_NODE_RX_MAX];
        uint16_t            length;
} UART_Frame_StructTypeDef;

typedef struct
{
        UART_HandleTypeDef *huart;

        QueueHandle_t TxQueue;
        QueueHandle_t RxQueue;

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
static void UART_Restart_Receive(UART_Bus_StructTypeDef *bus)
{
        HAL_UARTEx_ReceiveToIdle_DMA(bus->huart, bus->rxBuff, sizeof(bus->rxBuff));
        __HAL_DMA_DISABLE_IT(bus->huart->hdmarx, DMA_IT_HT);
}

static void UART_RxEvent_Callback(UART_HandleTypeDef *huart, uint16_t Pos)
{
        UART_Bus_StructTypeDef *bus = UART_Bus_Find(huart);
        if (bus == NULL) return;

        UART_Frame_StructTypeDef frame = {};

        frame.huart  = huart;
        frame.length = (Pos > UART_NODE_RX_MAX) ? UART_NODE_RX_MAX : Pos;
        memcpy(frame.data, bus->rxBuff, frame.length);

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(bus->RxQueue, &frame, &xHigherPriorityTaskWoken);

        UART_Restart_Receive(bus);

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void UART_Error_Callback(UART_HandleTypeDef *huart)
{
        UART_Bus_StructTypeDef *bus = UART_Bus_Find(huart);
        if (bus == NULL) return;

        UART_Restart_Receive(bus);
}

//按帧首地址分发到节点
static void UART_RxTask(void *argument)
{
        UART_Bus_StructTypeDef *bus = (UART_Bus_StructTypeDef *) argument;
        UART_Frame_StructTypeDef frame;

        for (;;)
        {
                if (xQueueReceive(bus->RxQueue, &frame, portMAX_DELAY) != pdPASS) continue;
                if (frame.length < UART_FRAME_MIN_LEN) continue;

                for (uint8_t i = 0; i < UART_Node_Count; i++)
                {
                        UART_Node_HandleTypeDef *node = UART_Node_Table[i];
                        if (node == NULL || node->Base.handler == NULL) continue;
                        if (node->huart != frame.huart) continue;
                        if (frame.data[0] != (uint8_t) node->Base.Device_ID) continue;

                        node->rxLength       = frame.length;
                        node->Base.Rx_Length = frame.length;
                        memcpy(node->rxData, frame.data, frame.length);

                        node->Base.handler(&node->Base);
                        break;
                }
        }
}

static void UART_TxTask(void *argument)
{
        UART_Bus_StructTypeDef *bus = (UART_Bus_StructTypeDef *) argument;
        UART_Frame_StructTypeDef frame;

        for (;;)
        {
                if (xQueueReceive(bus->TxQueue, &frame, portMAX_DELAY) == pdPASS)
                {
                        HAL_UART_Transmit(frame.huart, frame.data, frame.length, UART_TX_TIMEOUT);
                }
        }
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

        uart_node->huart = huart;
        uart_node->rxLength = 0;

        uart_node->Base.Rx_Data   = uart_node->rxData;
        uart_node->Base.Rx_Length = 0;

        UART_Node_Register(uart_node);
}

HAL_StatusTypeDef UART_Node_Transmit(UART_Node_HandleTypeDef *uart_node, const uint8_t *TX_Data, uint16_t Length)
{
        return HAL_UART_Transmit(uart_node->huart, (uint8_t *) TX_Data, Length, UART_TX_TIMEOUT);
}

void UART_Node_Transmit_Queue(UART_Node_HandleTypeDef *uart_node, const uint8_t *TX_Data, uint16_t Length)
{
        UART_Bus_StructTypeDef *bus = UART_Bus_Find(uart_node->huart);
        if (bus == NULL || Length > UART_NODE_RX_MAX) return;

        UART_Frame_StructTypeDef frame = {};

        frame.huart  = uart_node->huart;
        frame.length = Length;
        memcpy(frame.data, TX_Data, Length);

        xQueueSend(bus->TxQueue, &frame, 0);
}

void UART_Bus_Init(UART_HandleTypeDef *huart)
{
        if (UART_Bus_Count >= UART_BUS_MAX) return;
        if (UART_Bus_Find(huart) != NULL) return;

        UART_Bus_StructTypeDef *bus = &UART_Bus_Table[UART_Bus_Count++];

        bus->huart   = huart;
        bus->TxQueue = xQueueCreate(UART_TX_QUEUE_LEN, sizeof(UART_Frame_StructTypeDef));
        bus->RxQueue = xQueueCreate(UART_RX_QUEUE_LEN, sizeof(UART_Frame_StructTypeDef));

        xTaskCreate(UART_TxTask, "UART_Tx", 256, bus, 10, NULL);
        xTaskCreate(UART_RxTask, "UART_Rx", 256, bus, 7, NULL);

        HAL_UART_RegisterRxEventCallback(huart, UART_RxEvent_Callback);
        HAL_UART_RegisterCallback(huart, HAL_UART_ERROR_CB_ID, UART_Error_Callback);

        UART_Restart_Receive(bus);
}
