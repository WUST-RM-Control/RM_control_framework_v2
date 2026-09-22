//
// Created by ye on 2026/9/22.
//

#ifndef G4MINI_V3_DRV_KEY_H
#define G4MINI_V3_DRV_KEY_H

#include "stm32g4xx_hal.h"

typedef enum
{
        Release   = 0,
        ShortPush = 1,
        LongPush  = 2,
    } Key_State;

typedef struct
{
        GPIO_TypeDef *GPIOx;
        uint16_t GPIO_Pin;

        uint8_t Pin_Now;
        uint8_t Pin_Last;

        int16_t Push_Time;
        int16_t Release_Time;

        Key_State State;
} Key_HandleTypeDef;

extern Key_HandleTypeDef hkey1;

void Key_Ctor(Key_HandleTypeDef *hkey, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

void Key_Scan_Task(void);

Key_State Key_Get_State(Key_HandleTypeDef *hkey);

#endif //G4MINI_V3_DRV_KEY_H
