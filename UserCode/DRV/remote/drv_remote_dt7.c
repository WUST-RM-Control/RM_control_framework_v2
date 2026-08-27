//
// Created by ye on 2026/8/26.
//

#include "drv_remote_dt7.h"

#include <string.h>

/*===| DT7 协议解包 |===*/
void Remote_DT7_GetData(Remote_HandleTypeDef *hremote, const uint8_t *DataBuff)
{
        memcpy(&hremote->Data_Last, &hremote->Data, sizeof(Remote_Data_StructTypeDef));

        hremote->Data.RC_Right_X =  (float) (((DataBuff[0] | (DataBuff[1] << 8)) & 0x07ff) - RC_CH_VALUE_OFFSET) / 660.0f;
        hremote->Data.RC_Right_Y =  (float) ((((DataBuff[1] >> 3) | (DataBuff[2] << 5)) & 0x07ff) - RC_CH_VALUE_OFFSET) / 660.0f;
        hremote->Data.RC_Left_X  =  (float) ((((DataBuff[2] >> 6) | (DataBuff[3] << 2) | (DataBuff[4] << 10)) & 0x07ff) - RC_CH_VALUE_OFFSET) / 660.0f;
        hremote->Data.RC_Left_Y  =  (float) ((((DataBuff[4] >> 1) | (DataBuff[5] << 7)) & 0x07ff) - RC_CH_VALUE_OFFSET) / 660.0f;
        hremote->Data.RC_Side    = -(float) ((DataBuff[16] | (DataBuff[17] << 8)) - RC_CH_VALUE_OFFSET) / 660.0f;

        hremote->Data.S1 = ((DataBuff[5] >> 4) & 0x000C) >> 2;
        hremote->Data.S2 = ((DataBuff[5] >> 4) & 0x0003);

        int16_t Mouse_X = (int16_t) ((uint16_t) DataBuff[6] | ((uint16_t) DataBuff[7] << 8));
        int16_t Mouse_Y = (int16_t) ((uint16_t) DataBuff[8] | ((uint16_t) DataBuff[9] << 8));
        int16_t Mouse_Z = (int16_t) ((uint16_t) DataBuff[10] | ((uint16_t) DataBuff[11] << 8));

        hremote->Data.Mouse_Speed_X = (float) Mouse_X / 32.0f;
        hremote->Data.Mouse_Speed_Y = (float) Mouse_Y / 32.0f;
        hremote->Data.Mouse_Speed_Z = (float) Mouse_Z / 32.0f;

        hremote->Data.Mouse_Press_L = DataBuff[12];
        hremote->Data.Mouse_Press_R = DataBuff[13];

        hremote->Data.Keyboard_W     = !!(DataBuff[14] & 0x01);
        hremote->Data.Keyboard_S     = !!(DataBuff[14] & 0x02);
        hremote->Data.Keyboard_A     = !!(DataBuff[14] & 0x04);
        hremote->Data.Keyboard_D     = !!(DataBuff[14] & 0x08);
        hremote->Data.Keyboard_Shift = !!(DataBuff[14] & 0x10);
        hremote->Data.Keyboard_Ctrl  = !!(DataBuff[14] & 0x20);
        hremote->Data.Keyboard_Q     = !!(DataBuff[14] & 0x40);
        hremote->Data.Keyboard_E     = !!(DataBuff[14] & 0x80);
        hremote->Data.Keyboard_R     = !!(DataBuff[15] & 0x01);
        hremote->Data.Keyboard_F     = !!(DataBuff[15] & 0x02);
        hremote->Data.Keyboard_G     = !!(DataBuff[15] & 0x04);
        hremote->Data.Keyboard_Z     = !!(DataBuff[15] & 0x08);
        hremote->Data.Keyboard_X     = !!(DataBuff[15] & 0x10);
        hremote->Data.Keyboard_C     = !!(DataBuff[15] & 0x20);
        hremote->Data.Keyboard_V     = !!(DataBuff[15] & 0x40);
        hremote->Data.Keyboard_B     = !!(DataBuff[15] & 0x80);

        //死区限制与范围映射
        if      (hremote->Data.RC_Right_X >  0.1f) hremote->Data.RC_Right_X = (hremote->Data.RC_Right_X - 0.1f) / 0.9f;
        else if (hremote->Data.RC_Right_X < -0.1f) hremote->Data.RC_Right_X = (hremote->Data.RC_Right_X + 0.1f) / 0.9f;
        else     hremote->Data.RC_Right_X = 0;

        if      (hremote->Data.RC_Right_Y >  0.1f) hremote->Data.RC_Right_Y = (hremote->Data.RC_Right_Y - 0.1f) / 0.9f;
        else if (hremote->Data.RC_Right_Y < -0.1f) hremote->Data.RC_Right_Y = (hremote->Data.RC_Right_Y + 0.1f) / 0.9f;
        else     hremote->Data.RC_Right_Y = 0;

        if      (hremote->Data.RC_Left_X >  0.1f) hremote->Data.RC_Left_X = (hremote->Data.RC_Left_X - 0.1f) / 0.9f;
        else if (hremote->Data.RC_Left_X < -0.1f) hremote->Data.RC_Left_X = (hremote->Data.RC_Left_X + 0.1f) / 0.9f;
        else     hremote->Data.RC_Left_X = 0;

        if      (hremote->Data.RC_Left_Y >  0.1f) hremote->Data.RC_Left_Y = (hremote->Data.RC_Left_Y - 0.1f) / 0.9f;
        else if (hremote->Data.RC_Left_Y < -0.1f) hremote->Data.RC_Left_Y = (hremote->Data.RC_Left_Y + 0.1f) / 0.9f;
        else     hremote->Data.RC_Left_Y = 0;
}

/*===| DT7 构造 |===*/
static Remote_VTable Remote_DT7_VTable_Default = {
        .get_data = Remote_DT7_GetData
};

void Remote_DT7_Ctor(Remote_HandleTypeDef *hremote, UART_HandleTypeDef *huart)
{
        memset(hremote, 0, sizeof(Remote_HandleTypeDef));

        hremote->huart = huart;
        hremote->vptr  = &Remote_DT7_VTable_Default;
}
