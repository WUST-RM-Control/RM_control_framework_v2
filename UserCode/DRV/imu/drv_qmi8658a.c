//============================================================================================================================================================================
//                                              QMI8658A[六轴IMU驱动]
//      单芯片(accel+gyro 共用一条片选) → 只用 SPI 节点 Node[0]
//      片选由SPI节点管理, 驱动内不出现引脚宏; 寄存器/解包公式沿用原驱动
//============================================================================================================================================================================

#include "drv_qmi8658a.h"

#include <math.h>
#include <string.h>

#include "hal_dwt.h"

/*===| 底层SPI访问: 读时地址置 0x80(读标志+地址自增已在CTRL1开启), 数据从 rx[1] 起 |===*/
uint8_t QMI8658A_Read_Reg(IMU_HandleTypeDef *himu, QMI8658A_Reg_EnumTypedef Reg)
{
        uint8_t tx[2] = {0x80 | Reg, 0};
        uint8_t rx[2] = {0};

        SPI_Node_TransmitReceive(&himu->Node[0], tx, rx, 2);

        return rx[1];
}

void QMI8658A_Read_RegList(IMU_HandleTypeDef *himu, QMI8658A_Reg_EnumTypedef Reg, uint8_t Num, uint8_t *Read_Data)
{
        uint8_t tx[16] = {0x80 | Reg, 0};
        uint8_t rx[16] = {0};

        SPI_Node_TransmitReceive(&himu->Node[0], tx, rx, Num + 1);

        memcpy(Read_Data, &rx[1], Num);
}

void QMI8658A_Write_Reg(IMU_HandleTypeDef *himu, QMI8658A_Reg_EnumTypedef Reg, uint8_t Write_Data)
{
        uint8_t tx[2] = {0x7F & Reg, Write_Data};
        uint8_t rx[2] = {0};

        SPI_Node_TransmitReceive(&himu->Node[0], tx, rx, 2);
}

/*===| 初始化寄存器表 |===*/
static uint8_t QMI8658A_Init_RegList[][2] =
{
        {QMI8658A_Reg_CTRL1, 0x60}, // 开启SPI地址自动递增,MSB
        {QMI8658A_Reg_CTRL7, 0x00}, // 关闭加速度计+陀螺仪+温度传感器
        {QMI8658A_Reg_CTRL2, 0x13}, // 加速度计：量程±4g，输出数据率1000Hz
        {QMI8658A_Reg_CTRL3, 0x63}, // 陀螺仪：量程±1024dps，输出数据率896.8Hz
        {QMI8658A_Reg_CTRL5, 0x33}, // 加速度计+陀螺仪低通滤波带宽：ODR/10（抗干扰）
        {QMI8658A_Reg_CTRL8, 0x01}, // 开启温度传感器
        {QMI8658A_Reg_CTRL7, 0x03}, // 开启加速度计+陀螺仪+温度传感器
};

/*===| 读全部数据 |===*/
static void QMI8658A_Read(IMU_HandleTypeDef *himu)
{
        struct
        {
                uint16_t Tempture;
                int16_t  AX;
                int16_t  AY;
                int16_t  AZ;
                int16_t  GX;
                int16_t  GY;
                int16_t  GZ;
        } RAW_Data;

        QMI8658A_Read_RegList(himu, QMI8658A_Reg_TEMP_L, 14, (uint8_t *) &RAW_Data);

        //根据量程解包
        himu->Temperature = (float) RAW_Data.Tempture / 256.0f;
        himu->Accel[0]    = himu->IMU_Config_Struct.Accel_SEN * (float) RAW_Data.AX * himu->AccelScale;
        himu->Accel[1]    = himu->IMU_Config_Struct.Accel_SEN * (float) RAW_Data.AY * himu->AccelScale;
        himu->Accel[2]    = himu->IMU_Config_Struct.Accel_SEN * (float) RAW_Data.AZ * himu->AccelScale;
        himu->Gyro[0]     = himu->IMU_Config_Struct.Gyro_SEN * (float) RAW_Data.GX - himu->Gyro_Offset[0];
        himu->Gyro[1]     = himu->IMU_Config_Struct.Gyro_SEN * (float) RAW_Data.GY - himu->Gyro_Offset[1];
        himu->Gyro[2]     = himu->IMU_Config_Struct.Gyro_SEN * (float) RAW_Data.GZ - himu->Gyro_Offset[2];
}

/*===| 器件自检: 读Who Am I, 返回0=正常 |===*/
static uint8_t QMI8658A_Check(IMU_HandleTypeDef *himu)
{
        return (QMI8658A_Read_Reg(himu, QMI8658A_Reg_WHO_AM_I) == QMI8658A_WHO_AM_I_VALUE) ? 0 : 1;
}

/*===| 初始化: 写寄存器表 + 开机标定 |===*/
static void QMI8658A_Calibration(IMU_HandleTypeDef *himu);

static void QMI8658A_Init(IMU_HandleTypeDef *himu)
{
        //写入初始化寄存器表
        uint8_t reg_num = sizeof(QMI8658A_Init_RegList) / sizeof(QMI8658A_Init_RegList[0]);
        for (uint8_t i = 0; i < reg_num; i++)
        {
                QMI8658A_Write_Reg(himu, QMI8658A_Init_RegList[i][0], QMI8658A_Init_RegList[i][1]);
        }

        //从Flash读校准值，如果没有则启动自校准
        //读flash...

        if (himu->AccelScale < 0.3f)
                QMI8658A_Calibration(himu);
}

/*===| 开机标定(静止取样求零漂与标度, 与 BMI088_Calibration 同语义) |===*/
#define QMI8658A_Calibration_Num 10000.0f

static void QMI8658A_Calibration(IMU_HandleTypeDef *himu)
{
        //初始化校准值
        himu->AccelScale     = 1;
        himu->Gyro_Offset[0] = 0;
        himu->Gyro_Offset[1] = 0;
        himu->Gyro_Offset[2] = 0;

        uint16_t Caled_Num      = 0;
        float    Gravity_Offset = 0;
        float    Gyro_Offset[3] = {0};

        //循环获取数据
        while ((float) Caled_Num < QMI8658A_Calibration_Num)
        {
                QMI8658A_Read(himu);
                //如果检测到运动，则重置校准
                if (fabsf(himu->Gyro[0]) > 0.3f || fabsf(himu->Gyro[1]) > 0.3f || fabsf(himu->Gyro[2]) > 0.3f)
                {
                        //重置校准值
                        Gravity_Offset = 0;
                        Gyro_Offset[0] = 0;
                        Gyro_Offset[1] = 0;
                        Gyro_Offset[2] = 0;
                        Caled_Num      = 0;
                        continue;
                }

                //累加数据
                Gravity_Offset += sqrtf(himu->Accel[0] * himu->Accel[0] + himu->Accel[1] * himu->Accel[1] + himu->Accel[2] * himu->Accel[2]);
                Gyro_Offset[0] += himu->Gyro[0];
                Gyro_Offset[1] += himu->Gyro[1];
                Gyro_Offset[2] += himu->Gyro[2];

                Caled_Num++;
                DWT_Delay(0.0001f);
        }

        //求平均并保存校准值
        himu->AccelScale     = GRAVITY / (Gravity_Offset / QMI8658A_Calibration_Num);
        himu->Gyro_Offset[0] = Gyro_Offset[0] / QMI8658A_Calibration_Num;
        himu->Gyro_Offset[1] = Gyro_Offset[1] / QMI8658A_Calibration_Num;
        himu->Gyro_Offset[2] = Gyro_Offset[2] / QMI8658A_Calibration_Num;

        //保存到flash内
        //写flash...
}

/*=============|OOPC|================*/
IMU_VTable QMI8658A_VTable_Default = {
        .check       = QMI8658A_Check,
        .init        = QMI8658A_Init,
        .read        = QMI8658A_Read,
        .calibration = QMI8658A_Calibration
};

//片选是板级的(G4mini: SPI1_CS = PB0), 本工程不挂该器件 → Port 留空, 换板时由 ENT 填写
IMU_Config_StructTypeDef QMI8658A_Config_Default = {
        .vptr = &QMI8658A_VTable_Default,

        .hspi  = &hspi1,
        .Port  = NULL,
        .Pin   = 0,
        .Port2 = NULL,
        .Pin2  = 0,

        .Accel_SEN = 4.0f * GRAVITY / 32768.0f,         //±4g
        .Gyro_SEN  = 1024.0f * PI / 180.0f / 32768.0f   //±1024dps
};
