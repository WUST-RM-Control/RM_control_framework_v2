//
// Created by ye on 2026/8/26.
//

#ifndef G4MINI_V3_DRV_REMOTE_DT7_H
#define G4MINI_V3_DRV_REMOTE_DT7_H

#include "drv_remote.h"

//DT7 协议vtable(参考 Motor_DJI_VTable_Default)
extern Remote_VTable Remote_DT7_VTable_Default;

//DT7 协议解包(参考 Motor_DJI_Storage_Data)
void Remote_DT7_GetData(Remote_HandleTypeDef *hremote, const uint8_t *DataBuff);

#endif //G4MINI_V3_DRV_REMOTE_DT7_H
