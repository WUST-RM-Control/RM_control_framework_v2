//
// Created by ye on 2026/8/26.
//

#include "hal_debug.h"

#include "usart.h"

void Debug_Init(void)
{
        //USART3 已由 MX_USART3_UART_Init 配置(115200), 此处留作调试串口初始化扩展
}

//printf 重定向到 USART3(覆盖 syscalls.c 中的弱符号)
int __io_putchar(int ch)
{
        uint8_t c = (uint8_t)ch;
        HAL_UART_Transmit_IT(&huart3, &c, 1);
        return ch;
}
