//
// Created by ye on 2026/8/29.
//

#ifndef G4MINI_V3_VEHICLE_CONTROL_H
#define G4MINI_V3_VEHICLE_CONTROL_H

#include "FreeRTOS.h"
#include "task.h"
#include "stm32g4xx_hal.h"

typedef struct
{
        float Pitch;
        float Yaw;

        float Chassis_Vx;
        float Chassis_Vy;
        float Chassis_W;
} Vehicle_HandleTypeDef;

Vehicle_HandleTypeDef hvehicle1;

__STATIC_INLINE void Vehicle_Set_Yaw(Vehicle_HandleTypeDef *vehicle, float Yaw) { vehicle->Yaw = Yaw; }

__STATIC_INLINE void Vehicle_Set_Pitch(Vehicle_HandleTypeDef *vehicle, float Pitch) { vehicle->Pitch = Pitch; }

__STATIC_INLINE void Vehicle_Set_Vx(Vehicle_HandleTypeDef *vehicle, float chassis_Vx) { vehicle->Chassis_Vx = chassis_Vx; }

__STATIC_INLINE void Vehicle_Set_Vy(Vehicle_HandleTypeDef *vehicle, float chassis_Vy) { vehicle->Chassis_Vy = chassis_Vy; }

__STATIC_INLINE void Vehicle_Set_W(Vehicle_HandleTypeDef *vehicle, float chassis_W) { vehicle->Chassis_W = chassis_W; }


#endif //G4MINI_V3_VEHICLE_CONTROL_H
