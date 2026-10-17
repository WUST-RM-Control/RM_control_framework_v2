//
// Created by ye on 2026/9/22.
//

#include "srv_led_key.h"

#include "drv_key.h"


void LED_Key_Task(void *pvParameters)
{
        TickType_t last_wake = xTaskGetTickCount();

        for (;;)
        {
                Key_Scan_Task();
                //led还没写（）
                vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));
        }
}
