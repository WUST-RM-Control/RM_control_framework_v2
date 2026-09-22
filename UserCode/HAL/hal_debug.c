//
// Created by ye on 2026/8/26.
//

#include "hal_debug.h"

static UART_HandleTypeDef *Debug_UART = NULL;

//串口错误计数(发送侧错误, 供调试观察)
static volatile uint32_t Debug_Error_Count = 0;

void Debug_Ctor(UART_HandleTypeDef *huart)
{
        Debug_UART = huart;
}

/* UART错误回调: 由 Debug_Init 注册到 Debug_UART(per-handle, 各串口设备互不干扰)
 * 发送侧错误时 HAL 层已复位状态, 此处仅计数 */
static void Debug_UART_Error_Callback(UART_HandleTypeDef *huart)
{
        (void) huart;
        Debug_Error_Count++;
}

void Debug_Init(void)
{
        //注册UART错误回调
        if (Debug_UART != NULL)
        {
                HAL_UART_RegisterCallback(Debug_UART, HAL_UART_ERROR_CB_ID, Debug_UART_Error_Callback);
        }
}

//printf 重定向(覆盖 syscalls.c 中的弱符号)
int __io_putchar(int ch)
{
        uint8_t c = (uint8_t)ch;

        if (Debug_UART != NULL) HAL_UART_Transmit_IT(Debug_UART, &c, 1);
        return ch;
}

//获取串口错误计数
uint32_t Debug_Get_Error_Count(void)
{
        return Debug_Error_Count;
}
