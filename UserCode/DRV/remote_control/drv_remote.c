//
// Created by ye on 2026/8/26.
//

#include "drv_remote.h"

#include "drv_remote_dt7.h"
#include "drv_remote_vt03.h"


Remote_HandleTypeDef hremote_dt7;  //DT7 遥控器(串口接收)
Remote_HandleTypeDef hremote_vt03; //VT03 图传键鼠


void Remote_Init(UART_HandleTypeDef *huart_dt7)
{
        Remote_DT7_Ctor(&hremote_dt7, huart_dt7);
        Remote_VT03_Ctor(&hremote_vt03);
}
