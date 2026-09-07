//
// Created by ye on 2026/8/25.
//

#include "stm32g4xx_hal.h"
#include "drv_qmi8658a.h"

#include <math.h>
#include <string.h>
#include "hal_dwt.h"
#include "spi.h"
#include "utils.h"


//QMI8658A数据结构体
IMU_StructTypedef himu1;

//QMI8658A通信配置
#define QMI8658A_SPI               hspi1
#define QMI8658A_CS(PIN_Status)    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, PIN_Status)

//QMI8658A读单个寄存器
uint8_t QMI8658A_Read_Reg(QMI8658A_Reg_EnumTypedef Reg)
{
    uint8_t tx[2] = {0x80 | Reg, 0};
    uint8_t rx[2] = {0};

    QMI8658A_CS(0);
    HAL_SPI_TransmitReceive(&QMI8658A_SPI, tx, rx, 2, 1000);
    QMI8658A_CS(1);

    return rx[1];
}

//QMI8658A读多个寄存器
void QMI8658A_Read_RegList(QMI8658A_Reg_EnumTypedef Reg, uint8_t Num, uint8_t *Read_Data)
{
    uint8_t tx[16] = {0x80 | Reg, 0};
    uint8_t rx[16] = {0};

    QMI8658A_CS(0);
    HAL_SPI_TransmitReceive(&QMI8658A_SPI, tx, rx, Num+1, 1000);
    QMI8658A_CS(1);

    memcpy(Read_Data, &rx[1], Num);
}

//QMI8658A写单个寄存器
void QMI8658A_Write_Reg(QMI8658A_Reg_EnumTypedef Reg, uint8_t Write_Data)
{
    uint8_t tx[2] = {0x7F&Reg, Write_Data};
    uint8_t rx[2] = {0};

    QMI8658A_CS(0);
    HAL_SPI_TransmitReceive(&QMI8658A_SPI, tx, rx, 2, 1000);
    QMI8658A_CS(1);
}

//配置寄存器表
uint8_t QMI8658A_Init_RegList[][2] =
{
    {QMI8658A_Reg_CTRL1     , 0x60}, // 开启SPI地址自动递增,MSB
    {QMI8658A_Reg_CTRL7     , 0x00}, // 关闭加速度计+陀螺仪+温度传感器
    {QMI8658A_Reg_CTRL2     , 0x13}, // 加速度计：量程±4g，输出数据率1000Hz
    {QMI8658A_Reg_CTRL3     , 0x63}, // 陀螺仪：量程±1024dps，输出数据率896.8Hz
    {QMI8658A_Reg_CTRL5     , 0x33}, // 加速度计+陀螺仪低通滤波带宽：ODR/10（抗干扰）
    {QMI8658A_Reg_CTRL8     , 0x01}, // 开启温度传感器
    {QMI8658A_Reg_CTRL7     , 0x03}, // 开启加速度计+陀螺仪+温度传感器};
};

//QMI8658A初始化
void QMI8658A_Init(void)
{
    //写入初始化寄存器表
    uint8_t reg_num = sizeof(QMI8658A_Init_RegList) / sizeof(QMI8658A_Init_RegList[0]);
    for(uint8_t i = 0; i < reg_num; i++)
    {
        QMI8658A_Write_Reg(QMI8658A_Init_RegList[i][0], QMI8658A_Init_RegList[i][1]);
    }

    //从Flash读校准值，如果没有则启动自校准
    //读flash...

    // if(himu1.AccelScale < 0.3f)
    //     QMI8658A_Calibration();
}

//QMI8658A读全部数据
void QMI8658A_Read_Data(void)
{
    struct {
        uint16_t Tempture;
        int16_t AX;
        int16_t AY;
        int16_t AZ;
        int16_t GX;
        int16_t GY;
        int16_t GZ;;
    } RAW_Data;

    QMI8658A_Read_RegList(QMI8658A_Reg_TEMP_L, 14, (uint8_t*)&RAW_Data);

    //根据量程解包
    himu1.Temperature =  (float)RAW_Data.Tempture / 256.0f;
    himu1.Accel[0] = 4.0f * GRAVITY * (float)RAW_Data.AX / 32768.0f * himu1.AccelScale;
    himu1.Accel[1] = 4.0f * GRAVITY * (float)RAW_Data.AY / 32768.0f * himu1.AccelScale;;
    himu1.Accel[2] = 4.0f * GRAVITY * (float)RAW_Data.AZ / 32768.0f * himu1.AccelScale;;
    himu1.Gyro[0]  = 1024.0f * PI / 180.0f * (float)RAW_Data.GX / 32768.0f - himu1.Gyro_Offset[0];
    himu1.Gyro[1]  = 1024.0f * PI / 180.0f * (float)RAW_Data.GY / 32768.0f - himu1.Gyro_Offset[1];
    himu1.Gyro[2]  = 1024.0f * PI / 180.0f * (float)RAW_Data.GZ / 32768.0f - himu1.Gyro_Offset[2];
}

//QMI8658A校准
#define Calibration_Num 10000.0f
void QMI8658A_Calibration(void)
{
    //初始化校准值
    himu1.AccelScale = 1;
    himu1.Gyro_Offset[0] = 0;
    himu1.Gyro_Offset[1] = 0;
    himu1.Gyro_Offset[2] = 0;
    uint16_t Caled_Num = 0;
    float Gravity_Offset = 0;
    float Gyro_Offset[3] = {0};

    //循环获取数据
    while((float)Caled_Num < Calibration_Num)
    {
        QMI8658A_Read_Data();
        //如果检测到运动，则重置校准
        if(fabsf(himu1.Gyro[0]) > 0.3f || fabsf(himu1.Gyro[1]) > 0.3f || fabsf(himu1.Gyro[2]) > 0.3f)
        {
            //重置校准值
            Gravity_Offset = 0;
            Gyro_Offset[0] = 0;
            Gyro_Offset[1] = 0;
            Gyro_Offset[2] = 0;
            Caled_Num = 0;
            continue;
        }

        //累加数据
        Gravity_Offset += sqrtf(himu1.Accel[0]*himu1.Accel[0] + himu1.Accel[1]*himu1.Accel[1] + himu1.Accel[2]*himu1.Accel[2]);
        Gyro_Offset[0] += himu1.Gyro[0];
        Gyro_Offset[1] += himu1.Gyro[1];
        Gyro_Offset[2] += himu1.Gyro[2];

        Caled_Num++;
        DWT_Delay(0.0001f);
    }

    //求平均并保存校准值
    himu1.AccelScale = GRAVITY / (Gravity_Offset / Calibration_Num);
    himu1.Gyro_Offset[0] = Gyro_Offset[0] / Calibration_Num;
    himu1.Gyro_Offset[1] = Gyro_Offset[1] / Calibration_Num;
    himu1.Gyro_Offset[2] = Gyro_Offset[2] / Calibration_Num;

    //保存到flash内
    //写flash...
}