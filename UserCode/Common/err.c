//
// Created by ye on 2026/8/26.
//

#include "err.h"

/*===| 错误对象注册表(供上层周期遍历) |===*/
static Err_HandleTypeDef *herr_table[ERR_MAX];
static uint8_t            herr_count = 0;

//错误处理对象注册
void Err_Register(Err_HandleTypeDef *herr, err_handler handler)
{
        if (herr == NULL || handler == NULL) return;

        for (uint8_t i = 0; i < herr_count; i++)
        {
                if (herr_table[i] == herr)
                {
                        herr_table[i]->handler = handler;
                        return;
                }
        }

        if (herr_count >= ERR_MAX) return;

        herr_table[herr_count]          = herr;
        herr_table[herr_count]->handler = handler;
        herr_count++;
}

//错误处理对象注销
void Err_UnRegister(Err_HandleTypeDef *herr)
{
        for (uint8_t i = 0; i < herr_count; i++)
        {
                if (herr_table[i] == herr)
                {
                        herr_count--;
                        herr_table[i] = herr_table[herr_count];
                        return;
                }
        }
}

//错误处理对象构造
void Err_Ctor(Err_HandleTypeDef *herr, uint16_t err_tick_Timeout, uint16_t err_count_maximum, err_handler handler)
{
        herr->tick          = 0;
        herr->tick_timeout  = err_tick_Timeout;

        herr->count         = 0;
        herr->count_maximum = err_count_maximum;

        herr->If_Err        = 1; //默认是错误的
        Err_Register(herr, handler);
}

uint8_t Err_Get_Count(void)
{
        return herr_count;
}

Err_HandleTypeDef *Err_Get_Handle(uint8_t index)
{
        if (index >= herr_count) return NULL;
        return herr_table[index];
}
