//
// Created by ye on 2026/8/29.
//

#ifndef G4MINI_V3_VOFA_H
#define G4MINI_V3_VOFA_H

#include <string.h>

#include "hal_dwt.h"
#include "stm32g4xx_hal.h"
#include "usart.h"

void VOFA_Init(void);

void VOFA_Send_Data(uint8_t channel, float data);

#endif //G4MINI_V3_VOFA_H
