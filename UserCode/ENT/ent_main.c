//
// Created by ye on 2026/8/24.
//

#include "ent_main.h"

#include "drv_buzzer.h"
#include "drv_motor.h"



void main_init()
{
        xTaskCreate(Buzzer_Task, "Buzzer", 256, &hbuzzer1, 5, NULL);
        xTaskCreate(Motor_Control_Task, "Motor", 512, NULL, 30, NULL );
}
