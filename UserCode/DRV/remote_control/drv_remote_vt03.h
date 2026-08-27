//
// Created by ye on 2026/8/26.
//

#ifndef G4MINI_V3_DRV_REMOTE_VT03_H
#define G4MINI_V3_DRV_REMOTE_VT03_H

#include "drv_remote.h"

//VT03 构造(参考 Motor_DM_Ctor)
void Remote_VT03_Ctor(Remote_HandleTypeDef *hremote);

//VT03 协议解包(参考 Motor_DJI_Storage_Data)
void Remote_VT03_GetData(Remote_HandleTypeDef *hremote, const uint8_t *DataBuff);

//VT03 接收入口(直接传键鼠结构体)
__STATIC_INLINE void Remote_VT03_Rx_Handle(Remote_HandleTypeDef *hremote, const Keyboard_Mouse_Data_StructTypedef *Keyboard_Mouse_Struct)
{
        Remote_Rx_Handle(hremote, (const uint8_t *)Keyboard_Mouse_Struct);
}

#endif //G4MINI_V3_DRV_REMOTE_VT03_H
