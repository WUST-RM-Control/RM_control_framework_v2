//
// Created by ye on 2026/8/26.
//

#include "drv_led.h"

LED_HandleTypeDef hled1 = {};

void LED_Ctor(LED_HandleTypeDef* hled, TIM_HandleTypeDef *htim)
{
        hled->htim      = htim;
        hled->R_channel = TIM_CHANNEL_2;
        hled->G_channel = TIM_CHANNEL_3;
        hled->B_channel = TIM_CHANNEL_1;
}

void LED_Init(LED_HandleTypeDef* hled)
{
        HAL_TIM_PWM_Start(hled->htim, hled->R_channel);
        HAL_TIM_PWM_Start(hled->htim, hled->G_channel);
        HAL_TIM_PWM_Start(hled->htim, hled->B_channel);
}

void LED_Set(LED_HandleTypeDef *hled, uint8_t R, uint8_t G, uint8_t B)
{
        hled->R_value = R;
        hled->G_value = G;
        hled->B_value = B;

        __HAL_TIM_SET_COMPARE(hled->htim, hled->R_channel, hled->R_value);
        __HAL_TIM_SET_COMPARE(hled->htim, hled->G_channel, hled->G_value);
        __HAL_TIM_SET_COMPARE(hled->htim, hled->B_channel, hled->B_value);
}