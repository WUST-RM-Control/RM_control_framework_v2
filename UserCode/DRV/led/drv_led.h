//
// Created by ye on 2026/8/26.
//

#ifndef G4MINI_V3_LED_H
#define G4MINI_V3_LED_H

#include "stm32g4xx_hal.h"
#include "tim.h"

typedef struct
{
        uint8_t R;
        uint8_t G;
        uint8_t B;
} LED_HandleTypeDef;

extern LED_HandleTypeDef hled1;

//改

__STATIC_INLINE void LED_Init(LED_HandleTypeDef* hled)
{
        if (hled == &hled1)
        {
                HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
                HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
                HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
        }
}

__STATIC_INLINE void LED_Set(LED_HandleTypeDef *hled, uint8_t R, uint8_t G, uint8_t B)
{
        hled->R = R;
        hled->G = G;
        hled->B = B;

}

#endif //G4MINI_V3_LED_H
