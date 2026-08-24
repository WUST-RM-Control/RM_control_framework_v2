//
// Created by ye on 2026/8/24.
//

#include "ent_main.h"
#include "drv_motor.h"



void main_init()
{
        xTaskCreate(Motor_Control_Task, "Motor", 512, NULL, 10, NULL );
}
