//
// Created by ye on 2026/8/29.
//

#include "drv_vofa.h"



/*===| 调试相关定义 |===*/
#define VOFA_TXDATA_SIZE 12      //vofa调试发送的信息量（float）
#define VOFA_RXDATA_SIZE 12
#define VOFA_TX_DELAY    30      //ms
#define VOFA_UART        &huart2

static float VOFA_fdata[VOFA_TXDATA_SIZE] = {0};
static uint8_t VOFA_tail[4] = {0x00, 0x00, 0x80, 0x7F};
// static float VOFA_RxData[VOFA_RXDATA_SIZE] = {0};

static uint8_t VOFA_message[VOFA_TXDATA_SIZE * 4 + 4] = {0};


void VOFA_Init()
{
        memcpy(VOFA_message + VOFA_TXDATA_SIZE * 4, VOFA_tail, 4);
}

void VOFA_Send_Data(uint8_t channel, float data)
{
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
