//
// Created by ye on 2026/8/29.
//

#include "vehicle_control.h"

#include "drv_remote.h"




void Vehicle_Control_Task(void *pvParameters)
{
        for (;;)
        {
                Vehicle_Set_Vx(&hvehicle1, Remote_Get_Left_X(&hremote_dt7));
                Vehicle_Set_Vy(&hvehicle1, Remote_Get_Left_Y(&hremote_dt7));

                vTaskDelay(1);
        }
}
