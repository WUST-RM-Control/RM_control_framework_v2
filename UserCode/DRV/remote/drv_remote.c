//
// Created by ye on 2026/8/26.
//

#include "drv_remote.h"

#include "drv_remote_dt7.h"
#include "drv_remote_vt03.h"
#include "usart.h"


Remote_HandleTypeDef hremote_dt7;  //DT7  遥控器
Remote_HandleTypeDef hremote_vt03; //VT03 图传

//uart1接收缓冲与回调(模块私有)
static uint8_t Remote_RxBuff[18] = {};
static void Remote_RxEvent_CallBack(UART_HandleTypeDef *huart, uint16_t Pos)
{
        if (huart == hremote_dt7.huart)
        {
                if (Pos > 0) //收到数据
                {
                        Remote_Rx_Handle(&hremote_dt7, Remote_RxBuff); //丢给DT7协议解包
                }
                //重新开启下一次接收
                HAL_UARTEx_ReceiveToIdle_DMA(huart, Remote_RxBuff, sizeof(Remote_RxBuff));
        }
}

void Remote_Init(UART_HandleTypeDef *huart)
{
        Remote_DT7_Ctor(&hremote_dt7, huart);
        Remote_VT03_Ctor(&hremote_vt03, NULL);

        //SBUS为反相电平，开启RXINV翻转
        SET_BIT(huart1.Instance->CR2, USART_CR2_RXINV);

        //注册UART空闲中断回调 + 启动DMA空闲接收(数据经 HAL_FDCAN 类似的回调分发)
        HAL_UART_RegisterRxEventCallback(huart, Remote_RxEvent_CallBack);
        HAL_UARTEx_ReceiveToIdle_DMA(huart, Remote_RxBuff, sizeof(Remote_RxBuff));
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
}
