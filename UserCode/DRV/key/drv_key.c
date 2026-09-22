//
// Created by ye on 2026/9/22.
//

#include "drv_key.h"

#include <string.h>

#include "utils.h"

#define KEY_MAX 16

static Key_HandleTypeDef *hkey_table[KEY_MAX];
static uint8_t            Key_Count = 0;

Key_HandleTypeDef hkey1 = {};

void Key_Ctor(Key_HandleTypeDef *hkey, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
        memset(hkey, 0, sizeof(Key_HandleTypeDef));
        hkey->GPIOx    = GPIOx;
        hkey->GPIO_Pin = GPIO_Pin;

        hkey->State = Release;

        if (Key_Count >= KEY_MAX) return;
        hkey_table[Key_Count] = hkey;
        Key_Count++;
}


void Key_Scan_Task(void) //10ms
{
        for (uint8_t i = 0; i < Key_Count; i++)
        {
                hkey_table[i]->Pin_Now = HAL_GPIO_ReadPin(hkey_table[i]->GPIOx, hkey_table[i]->GPIO_Pin);

                if (hkey_table[i]->Pin_Now == 0 && hkey_table[i]->Pin_Last == 1)
                {
                        //按下瞬间
                        hkey_table[i]->Push_Time    = 0;
                        hkey_table[i]->Release_Time = 0;
                }
                if (hkey_table[i]->Pin_Now == 0 && hkey_table[i]->Pin_Last == 0)
                {
                        //持续按下
                        hkey_table[i]->Push_Time++;
                        hkey_table[i]->Push_Time = int16_limit(hkey_table[i]->Push_Time, 0, 110);
                }
                if (hkey_table[i]->Pin_Now == 1 && hkey_table[i]->Pin_Last == 0)
                {
                        //松开瞬间
                        if (hkey_table[i]->Push_Time >= 20) hkey_table[i]->State = LongPush;
                        else hkey_table[i]->State                                 = ShortPush;
                }
                if (hkey_table[i]->Pin_Now == 1 && hkey_table[i]->Pin_Last == 1)
                {
                        //持续松开
                        hkey_table[i]->Release_Time++;
                        hkey_table[i]->Release_Time = int16_limit(hkey_table[i]->Release_Time, 0, 100);
                        if (hkey_table[i]->Release_Time >= 50)
                                hkey_table[i]->State = Release;
                }

                hkey_table[i]->Pin_Last = hkey_table[i]->Pin_Now;
        }
}


Key_State Key_Get_State(Key_HandleTypeDef *hkey)
{
        Key_State temp = hkey->State;
        hkey->State    = Release;
        return temp;
}
