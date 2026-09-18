//
// Created by ye on 2026/8/29.
//

#include "drv_vofa.h"



/*===| VOFA相关定义 |===*/
#define VOFA_TXDATA_SIZE 12      //vofa调试发送的信息量（float）
#define VOFA_RXDATA_SIZE 12
#define VOFA_TX_DELAY    30      //ms

static UART_HandleTypeDef *VOFA_UART = NULL;

static float VOFA_fdata[VOFA_TXDATA_SIZE] = {0};
static uint8_t VOFA_tail[4] = {0x00, 0x00, 0x80, 0x7F};
static float VOFA_RxData[VOFA_RXDATA_SIZE] = {0};

static uint8_t VOFA_message[VOFA_TXDATA_SIZE * 4 + 4] = {0};




//串口错误计数(发送侧错误, 供调试观察)
static volatile uint32_t VOFA_Error_Count = 0;


void VOFA_Ctor(UART_HandleTypeDef *huart)
{
        VOFA_UART = huart;
}

static void VOFA_RxEvent_CallBack(UART_HandleTypeDef *huart, uint16_t Pos)
{
        //重新开启下一次接收(必须在ISR内完成, 否则丢帧)
        HAL_UARTEx_ReceiveToIdle_DMA(huart, (uint8_t *)VOFA_RxData, sizeof(VOFA_RxData));
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);

        //处理。。。
}

/* UART错误回调: 由 VOFA_Init 注册到 VOFA_UART(per-handle, 各串口设备互不干扰)
 * 发送侧错误时 HAL/DMA 层已复位状态, 下次 VOFA_Send_Data 会重新发起, 此处仅计数 */
static void VOFA_UART_Error_Callback(UART_HandleTypeDef *huart)
{
        (void) huart;
        VOFA_Error_Count++;
}

void VOFA_Init()
{
        memcpy(VOFA_message + VOFA_TXDATA_SIZE * 4, VOFA_tail, 4);

        //注册UART错误回调
        if (VOFA_UART != NULL)
        {
                //注册UART空闲中断回调
                HAL_UART_RegisterRxEventCallback(VOFA_UART, VOFA_RxEvent_CallBack);
                HAL_UART_RegisterCallback(VOFA_UART, HAL_UART_ERROR_CB_ID, VOFA_UART_Error_Callback);
        }
}

void VOFA_Send_Data(uint8_t channel, float data)
{
        if (VOFA_UART == NULL) return;

        VOFA_fdata[channel] = data;

        static float tx_time_last = 0.0f;

        //两次发送间隔必须大于VOFA_TX_DELAY，防止欧阳的VOFA被干爆（）
        if (DWT_GetTimeline_ms() -  tx_time_last > VOFA_TX_DELAY)
        {
                tx_time_last = DWT_GetTimeline_ms();

                memcpy((char *)VOFA_message, (char *)VOFA_fdata, VOFA_TXDATA_SIZE * 4);
                HAL_UART_Transmit_DMA(VOFA_UART, (uint8_t *)VOFA_message, sizeof(VOFA_message));
        }


}

//获取串口错误计数
uint32_t VOFA_Get_Error_Count(void)
{
        return VOFA_Error_Count;
}
