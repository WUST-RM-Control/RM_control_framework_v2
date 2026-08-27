//
// Created by ye on 2026/8/26.
//

#ifndef G4MINI_V3_DRV_REMOTE_DT7_H
#define G4MINI_V3_DRV_REMOTE_DT7_H

#include "drv_remote.h"

//DT7 构造(参考 Motor_DJI_Ctor)
void Remote_DT7_Ctor(Remote_HandleTypeDef *hremote, UART_HandleTypeDef *huart);

//DT7 协议解包(参考 Motor_DJI_Storage_Data)
void Remote_DT7_GetData(Remote_HandleTypeDef *hremote, const uint8_t *DataBuff);

#endif //G4MINI_V3_DRV_REMOTE_DT7_H
