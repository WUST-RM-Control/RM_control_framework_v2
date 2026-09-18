//
// Created by ye on 2026/8/26.
//

#ifndef G4MINI_V3_DRV_IMU_H
#define G4MINI_V3_DRV_IMU_H

#include "drv_qmi8658a.h"
#include "stm32g4xx_hal.h"
#include "utils.h"

//IMU构造: 绑定 SPI 与片选引脚(实例 ↔ 外设绑定, 由 ENT 调用)
__STATIC_INLINE void IMU_Ctor(IMU_HandleTypedef *himu, SPI_HandleTypeDef *hspi, GPIO_TypeDef *CS_Port, uint16_t CS_Pin)
{
        if (himu == &himu1) QMI8658A_Ctor(hspi, CS_Port, CS_Pin);
}

//此处目前无须多态，故直接调用减少调用开销
__STATIC_INLINE void IMU_Init(IMU_HandleTypedef *himu) { if (himu == &himu1) QMI8658A_Init(); }

__STATIC_INLINE void IMU_Read_Data(IMU_HandleTypedef *himu) { if (himu == &himu1) QMI8658A_Read_Data(); }

__STATIC_INLINE void IMU_Calibration(IMU_HandleTypedef *himu) { if (himu == &himu1) QMI8658A_Calibration(); }

__STATIC_INLINE float *IMU_Get_Gyro(IMU_HandleTypedef *himu) { return himu->Gyro; }

__STATIC_INLINE float *IMU_Get_Accel(IMU_HandleTypedef *himu) { return himu->Accel; }

#endif //G4MINI_V3_DRV_IMU_H
