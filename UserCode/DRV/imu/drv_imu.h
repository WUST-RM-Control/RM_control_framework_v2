//
// Created by ye on 2026/10/5.
//

#ifndef G4MINI_V3_DRV_IMU_H
#define G4MINI_V3_DRV_IMU_H

#include "hal_spi.h"
#include "utils.h"

/*===| IMU的SPI节点数(一个从机/一条CS = 一个节点) |===*/
//BMI088 = 2 (accel走第一片选, gyro走第二片选), QMI8658A = 1
#define IMU_SPI_NODE_MAX 2

typedef struct IMU_HandleTypeDef IMU_HandleTypeDef;
typedef struct IMU_VTable        IMU_VTable;

/*===| IMU虚函数表(参考 Motor_VTable) |===*/
struct IMU_VTable
{
        uint8_t (*check)(IMU_HandleTypeDef *himu);       //器件自检(Who Am I): 返回0=正常
        void    (*init)(IMU_HandleTypeDef *himu);        //初始化寄存器
        void    (*read)(IMU_HandleTypeDef *himu);        //读原始数据并解包到 Accel/Gyro/Temperature
        void    (*calibration)(IMU_HandleTypeDef *himu); //零漂/标度标定
};

/*===| IMU配置结构体(SPI句柄 + 片选 + 灵敏度 + vptr, 参考 Motor_Config_StructTypeDef) |===*/
typedef struct
{
        IMU_VTable *vptr;

        SPI_HandleTypeDef *hspi;   //SPI句柄
        GPIO_TypeDef      *Port;   //片选1(主通道, 必有)
        uint16_t           Pin;    //片选1引脚
        GPIO_TypeDef      *Port2;  //片选2(BMI088的gyro用; 单CS设备填NULL)
        uint16_t           Pin2;   //片选2引脚

        float Accel_SEN; //加速度灵敏度[m/s²/LSB]
        float Gyro_SEN;  //角速度灵敏度[rad/s/LSB]
} IMU_Config_StructTypeDef;

/*===| IMU句柄(所有IMU的公共数据) |===*/
struct IMU_HandleTypeDef
{
        SPI_Node_HandleTypeDef Node[IMU_SPI_NODE_MAX]; //SPI节点基类(第一个成员, Node[0]可向上转型)

        IMU_Config_StructTypeDef IMU_Config_Struct;

        float Accel[3];       //加速度[m/s²]
        float Gyro[3];        //角速度[rad/s]
        float Temperature;    //温度[℃]

        float AccelScale;     //标度标定系数
        float Gyro_Offset[3]; //零漂
};

/*===| 板载IMU实例(具体跑哪个驱动, 由 ENT 构造时传入的 Config 决定) |===*/
extern IMU_HandleTypeDef himu1;

/*===| IMU通用构造(参考 Motor_Ctor): 绑定配置 + 按配置构造SPI节点 |===*/
void IMU_Ctor(IMU_HandleTypeDef *himu, IMU_Config_StructTypeDef IMU_Config_Struct);

/*===| vtable转发接口(参考 Motor_Enable / Motor_Send_Torque) |===*/
__STATIC_INLINE uint8_t IMU_Check(IMU_HandleTypeDef *himu)
{
        return himu->IMU_Config_Struct.vptr->check(himu);
}

__STATIC_INLINE void IMU_Init(IMU_HandleTypeDef *himu)
{
        himu->IMU_Config_Struct.vptr->init(himu);
}

__STATIC_INLINE void IMU_Read_Data(IMU_HandleTypeDef *himu)
{
        himu->IMU_Config_Struct.vptr->read(himu);
}

__STATIC_INLINE void IMU_Calibration(IMU_HandleTypeDef *himu)
{
        himu->IMU_Config_Struct.vptr->calibration(himu);
}

__STATIC_INLINE float *IMU_Get_Accel(IMU_HandleTypeDef *himu)
{
        return himu->Accel;
}

__STATIC_INLINE float *IMU_Get_Gyro(IMU_HandleTypeDef *himu)
{
        return himu->Gyro;
}

#endif //G4MINI_V3_DRV_IMU_H
