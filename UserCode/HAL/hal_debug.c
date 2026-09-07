//
// Created by ye on 2026/8/26.
//

#include "hal_debug.h"

#include "usart.h"

void Debug_Init(void)
{

}

//printf 重定向到 USART3(覆盖 syscalls.c 中的弱符号)
int __io_putchar(int ch)
{
        uint8_t c = (uint8_t)ch;
        HAL_UART_Transmit_IT(&huart3, &c, 1);
        return ch;
}
