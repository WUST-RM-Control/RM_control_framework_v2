//
// Created by ye on 2026/8/26.
//

#ifndef G4MINI_V3_LED_H
#define G4MINI_V3_LED_H

#include "stm32g4xx_hal.h"
#include "tim.h"

typedef struct
{
        TIM_HandleTypeDef *htim;

        uint8_t R_channel;
        uint8_t R_value;

        uint8_t G_channel;
        uint8_t G_value;

        uint8_t B_channel;
        uint8_t B_value;
} LED_HandleTypeDef;

extern LED_HandleTypeDef hled1;

void LED_Ctor(LED_HandleTypeDef* hled, TIM_HandleTypeDef *htim);

void LED_Init(LED_HandleTypeDef* hled);

//需要设置reload为255
void LED_Set(LED_HandleTypeDef *hled, uint8_t R, uint8_t G, uint8_t B);

#endif //G4MINI_V3_LED_H