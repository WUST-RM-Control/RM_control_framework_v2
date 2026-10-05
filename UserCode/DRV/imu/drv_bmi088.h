//
// Created by ye on 2026/10/5.
//

#ifndef G4MINI_V3_DRV_BMI088_H
#define G4MINI_V3_DRV_BMI088_H

#include "drv_imu.h"

/*===| BMI088 量程与灵敏度(与初始化寄存器表配对, 改量程必须同步改灵敏度) |===*/
#define BMI088_ACCEL_6G_SEN  0.00179443359375f      //加速度 ±6g: m/s²/LSB
#define BMI088_GYRO_2000_SEN 0.0010652644360316953f //角速度 ±2000dps: rad/s/LSB

/*===| BMI088 温度换算(加速度计侧 11bit 有符号) |===*/
#define BMI088_TEMP_FACTOR 0.125f
#define BMI088_TEMP_OFFSET 23.0f

/*===| BMI088 初始化/自检错误码(IMU_Check 返回值, 0=正常; 参考官方定义) |===*/
enum
{
        BMI088_NO_ERROR                     = 0x00,
        BMI088_ACC_PWR_CTRL_ERROR           = 0x01,
        BMI088_ACC_PWR_CONF_ERROR           = 0x02,
        BMI088_ACC_CONF_ERROR               = 0x03,
        BMI088_ACC_RANGE_ERROR              = 0x05,
        BMI088_INT1_IO_CTRL_ERROR           = 0x06,
        BMI088_INT_MAP_DATA_ERROR           = 0x07,
        BMI088_GYRO_RANGE_ERROR             = 0x08,
        BMI088_GYRO_BANDWIDTH_ERROR         = 0x09,
        BMI088_GYRO_LPM1_ERROR              = 0x0A,
        BMI088_GYRO_CTRL_ERROR              = 0x0B,
        BMI088_GYRO_INT3_INT4_IO_CONF_ERROR = 0x0C,
        BMI088_GYRO_INT3_INT4_IO_MAP_ERROR  = 0x0D,

        BMI088_NO_SENSOR = 0xFF, //读不到器件(Who Am I 不对)
};

/*===| BMI088 驱动默认配置(C型板: SPI1 + accel片选PA4 + gyro片选PB0) |===*/
extern IMU_VTable               IMU_BMI088_VTable_Default;
extern IMU_Config_StructTypeDef IMU_BMI088_Config_Default;

#endif //G4MINI_V3_DRV_BMI088_H
