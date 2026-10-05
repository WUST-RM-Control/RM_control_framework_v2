//
// Created by ye on 2026/8/25.
//

#ifndef G4MINI_V3_QMI8658A_H
#define G4MINI_V3_QMI8658A_H

#include "drv_imu.h"
#include "main.h"

#define GRAVITY 9.8f //标定与灵敏度换算用

/*===| QMI8658A 器件ID |===*/
#define QMI8658A_WHO_AM_I_VALUE 0x05

typedef enum
{
        QMI8658A_Reg_WHO_AM_I      = 0x00,
        QMI8658A_Reg_REVISION_ID   = 0x01,
        QMI8658A_Reg_CTRL1         = 0x02,
        QMI8658A_Reg_CTRL2         = 0x03,
        QMI8658A_Reg_CTRL3         = 0x04,
        QMI8658A_Reg_CTRL5         = 0x06,
        QMI8658A_Reg_CTRL6         = 0x07,
        QMI8658A_Reg_CTRL7         = 0x08,
        QMI8658A_Reg_CTRL8         = 0x09,
        QMI8658A_Reg_CTRL9         = 0x0A,
        QMI8658A_Reg_FIFO_WTM_TH   = 0x13,
        QMI8658A_Reg_FIFO_CTRL     = 0x14,
        QMI8658A_Reg_FIFO_SMPL_CNT = 0x15,
        QMI8658A_Reg_FIFO_STATUS   = 0x16,
        QMI8658A_Reg_FIFO_DATA     = 0x17,
        QMI8658A_Reg_STATUSINT     = 0x2D,
        QMI8658A_Reg_STATUS0       = 0x2E,
        QMI8658A_Reg_STATUS1       = 0x2F,
        QMI8658A_Reg_TEMP_L        = 0x33,
        QMI8658A_Reg_TEMP_H        = 0x34,
        QMI8658A_Reg_AX_L          = 0x35,
        QMI8658A_Reg_AX_H          = 0x36,
        QMI8658A_Reg_AY_L          = 0x37,
        QMI8658A_Reg_AY_H          = 0x38,
        QMI8658A_Reg_AZ_L          = 0x39,
        QMI8658A_Reg_AZ_H          = 0x3A,
        QMI8658A_Reg_GX_L          = 0x3B,
        QMI8658A_Reg_GX_H          = 0x3C,
        QMI8658A_Reg_GY_L          = 0x3D,
        QMI8658A_Reg_GY_H          = 0x3E,
        QMI8658A_Reg_GZ_L          = 0x3F,
        QMI8658A_Reg_GZ_H          = 0x40,
        QMI8658A_Reg_RESET         = 0x60,
} QMI8658A_Reg_EnumTypedef;

/*===| QMI8658A 底层寄存器读写(节点自带片选) |===*/
uint8_t QMI8658A_Read_Reg(IMU_HandleTypeDef *himu, QMI8658A_Reg_EnumTypedef Reg);

void QMI8658A_Read_RegList(IMU_HandleTypeDef *himu, QMI8658A_Reg_EnumTypedef Reg, uint8_t Num, uint8_t *Read_Data);

void QMI8658A_Write_Reg(IMU_HandleTypeDef *himu, QMI8658A_Reg_EnumTypedef Reg, uint8_t Write_Data);

/*===| QMI8658A 驱动默认配置
 * 注意: 片选是板级的(G4mini 上为 SPI1_CS = PB0), 本工程(C型板)不挂该器件 → Port 留空
 * 换板时由 ENT 先设好 Port/Pin 再 IMU_Ctor, 例如:
 *         IMU_Config_StructTypeDef cfg = QMI8658A_Config_Default;
 *         cfg.Port = GPIOB; cfg.Pin = GPIO_PIN_0;
 *         IMU_Ctor(&himu1, cfg);
 |===*/
extern IMU_VTable               QMI8658A_VTable_Default;
extern IMU_Config_StructTypeDef QMI8658A_Config_Default;

#endif //G4MINI_V3_QMI8658A_H
