//
// Created by ye on 2026/8/26.
//

#include "drv_remote_vt03.h"

#include <string.h>

/*===| VT03 图传键鼠协议解包 |===*/
void Remote_VT03_GetData(Remote_HandleTypeDef *hremote, const uint8_t *DataBuff)
{
        const Keyboard_Mouse_Data_StructTypedef *kb = (const Keyboard_Mouse_Data_StructTypedef *)DataBuff;

        memcpy(&hremote->Data_Last, &hremote->Data, sizeof(Remote_Data_StructTypeDef));

        hremote->Data.Mouse_Speed_X  = (float) kb->Mouse_Speed_X / 32.0f;
        hremote->Data.Mouse_Speed_Y  = (float) kb->Mouse_Speed_Y / 32.0f;
        hremote->Data.Mouse_Speed_Z  = (float) kb->Mouse_Speed_Z / 32.0f;

        hremote->Data.Mouse_Press_L  = kb->Mouse_Left;
        hremote->Data.Mouse_Press_R  = kb->Mouse_Right;

        hremote->Data.Keyboard_W     = !!(kb->KeyBoard & 0x0001);
        hremote->Data.Keyboard_S     = !!(kb->KeyBoard & 0x0002);
        hremote->Data.Keyboard_A     = !!(kb->KeyBoard & 0x0004);
        hremote->Data.Keyboard_D     = !!(kb->KeyBoard & 0x0008);
        hremote->Data.Keyboard_Shift = !!(kb->KeyBoard & 0x0010);
        hremote->Data.Keyboard_Ctrl  = !!(kb->KeyBoard & 0x0020);
        hremote->Data.Keyboard_Q     = !!(kb->KeyBoard & 0x0040);
        hremote->Data.Keyboard_E     = !!(kb->KeyBoard & 0x0080);
        hremote->Data.Keyboard_R     = !!(kb->KeyBoard & 0x0100);
        hremote->Data.Keyboard_F     = !!(kb->KeyBoard & 0x0200);
        hremote->Data.Keyboard_G     = !!(kb->KeyBoard & 0x0400);
        hremote->Data.Keyboard_Z     = !!(kb->KeyBoard & 0x0800);
        hremote->Data.Keyboard_X     = !!(kb->KeyBoard & 0x1000);
        hremote->Data.Keyboard_C     = !!(kb->KeyBoard & 0x2000);
        hremote->Data.Keyboard_V     = !!(kb->KeyBoard & 0x4000);
        hremote->Data.Keyboard_B     = !!(kb->KeyBoard & 0x8000);
}

/*===| VT03 构造 |===*/
static Remote_VTable Remote_VT03_VTable_Default = {
        .get_data = Remote_VT03_GetData
};

void Remote_VT03_Ctor(Remote_HandleTypeDef *hremote, UART_HandleTypeDef *huart)
{
        memset(hremote, 0, sizeof(Remote_HandleTypeDef));

        hremote->huart = huart;
        hremote->vptr = &Remote_VT03_VTable_Default;
}
