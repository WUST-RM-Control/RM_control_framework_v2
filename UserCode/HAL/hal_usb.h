//
// Created by ye on 2026/9/7.
//

#ifndef G4MINI_V3_HAL_USB_H
#define G4MINI_V3_HAL_USB_H
#include "usbd_cdc_if.h"
#include "stm32g4xx_hal.h"

#endif //G4MINI_V3_HAL_USB_H


/**
 * @brief USB发送数据
 */
__STATIC_INLINE void USB_Send(uint8_t *Data, uint8_t Length) { CDC_Transmit_FS(Data, Length); }