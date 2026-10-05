//============================================================================================================================================================================
//                                              BMI088[六轴IMU驱动]
//      Bosch BMI088: 一个逻辑IMU + 两个物理从机(accel/gyro 各自一条片选, 共用SPI)
//      → 对应两个SPI节点: Node[0]=accel, Node[1]=gyro; 片选由SPI节点管理, 驱动内不出现引脚宏
//      寄存器细节/解包公式沿用官方 C 板驱动, 去掉全局句柄、板级宏、开机死循环与 INFANTRY_ID 硬编码
//============================================================================================================================================================================

#include "drv_bmi088.h"

#include <math.h>
#include <string.h>

#include "BMI088reg.h"
#include "hal_dwt.h"

/*===| SPI收发缓冲上限(单次最长: gyro从CHIP_ID起读8字节 + 1) |===*/
#define BMI088_SPI_BUF_MAX 16

/*===| 初始化寄存器表(官方表原样保留: {寄存器, 写入值, 错误码}) |===*/
#define BMI088_WRITE_ACCEL_REG_NUM 6
#define BMI088_WRITE_GYRO_REG_NUM  6

static uint8_t BMI088_Accel_Init_RegList[BMI088_WRITE_ACCEL_REG_NUM][3] =
{
        {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
        {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
        {BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
        {BMI088_ACC_RANGE, BMI088_ACC_RANGE_6G, BMI088_ACC_RANGE_ERROR},
        {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, BMI088_INT1_IO_CTRL_ERROR},
        {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}
};

static uint8_t BMI088_Gyro_Init_RegList[BMI088_WRITE_GYRO_REG_NUM][3] =
{
        {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
        {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_2000_230_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR},
        {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
        {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW, BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}
};

/*===| 标定参数 |===*/
#define BMI088_CALI_NUM     10000.0f //标定采样点数
#define BMI088_CALI_GYRO_MAX 0.3f    //运动检测阈值[rad/s], 超过则重新标定
#define BMI088_GRAVITY      9.81f

/*===| 底层SPI访问(片选在SPI节点内, 此处只组帧) |===*/
/* accel: 地址字节 + 1个dummy字节后才能读数据
 * 官方写法是把地址发两遍(第二遍当dummy), 这里保持一致 → 数据取 rx[2] 起 */
static uint8_t BMI088_Accel_Read_Single_Reg(IMU_HandleTypeDef *himu, uint8_t Reg)
{
        uint8_t tx[3] = {Reg | 0x80, Reg | 0x80, 0x55};
        uint8_t rx[3] = {0};

        SPI_Node_TransmitReceive(&himu->Node[0], tx, rx, 3);

        return rx[2];
}

static void BMI088_Accel_Read_Multi_Reg(IMU_HandleTypeDef *himu, uint8_t Reg, uint8_t *Data, uint8_t Num)
{
        uint8_t tx[BMI088_SPI_BUF_MAX] = {0};
        uint8_t rx[BMI088_SPI_BUF_MAX] = {0};

        tx[0] = Reg | 0x80;
        tx[1] = Reg | 0x80; //dummy

        SPI_Node_TransmitReceive(&himu->Node[0], tx, rx, Num + 2);

        memcpy(Data, &rx[2], Num);
}

/* gyro: 地址字节后直接跟数据, 无需dummy → 数据取 rx[1] 起 */
static uint8_t BMI088_Gyro_Read_Single_Reg(IMU_HandleTypeDef *himu, uint8_t Reg)
{
        uint8_t tx[2] = {Reg | 0x80, 0x55};
        uint8_t rx[2] = {0};

        SPI_Node_TransmitReceive(&himu->Node[1], tx, rx, 2);

        return rx[1];
}

static void BMI088_Gyro_Read_Multi_Reg(IMU_HandleTypeDef *himu, uint8_t Reg, uint8_t *Data, uint8_t Num)
{
        uint8_t tx[BMI088_SPI_BUF_MAX] = {0};
        uint8_t rx[BMI088_SPI_BUF_MAX] = {0};

        tx[0] = Reg | 0x80;

        SPI_Node_TransmitReceive(&himu->Node[1], tx, rx, Num + 1);

        memcpy(Data, &rx[1], Num);
}

//写单个寄存器(写协议: 地址不含0x80 + 数据, accel/gyro 相同)
static void BMI088_Write_Single_Reg(SPI_Node_HandleTypeDef *node, uint8_t Reg, uint8_t Data)
{
        uint8_t tx[2] = {Reg & 0x7F, Data};
        uint8_t rx[2] = {0};

        SPI_Node_TransmitReceive(node, tx, rx, 2);
}

/*===| accel/gyro 初始化(单次执行, 不再死循环重试) |===*/
#define BMI088_LONG_DELAY_TIME 80 //软复位后等待[ms]

static void BMI088_Accel_Init(IMU_HandleTypeDef *himu)
{
        //上电默认是I2C模式: 需要一次"带片选的空读"才会切到SPI并保持(官方同样读两遍)
        BMI088_Accel_Read_Single_Reg(himu, BMI088_ACC_CHIP_ID);
        HAL_Delay(1);
        BMI088_Accel_Read_Single_Reg(himu, BMI088_ACC_CHIP_ID);
        HAL_Delay(1);

        //软复位
        BMI088_Write_Single_Reg(&himu->Node[0], BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
        HAL_Delay(BMI088_LONG_DELAY_TIME);

        //复位后通讯检查
        BMI088_Accel_Read_Single_Reg(himu, BMI088_ACC_CHIP_ID);
        HAL_Delay(1);
        BMI088_Accel_Read_Single_Reg(himu, BMI088_ACC_CHIP_ID);
        HAL_Delay(1);

        //写配置寄存器表
        for (uint8_t i = 0; i < BMI088_WRITE_ACCEL_REG_NUM; i++)
        {
                BMI088_Write_Single_Reg(&himu->Node[0], BMI088_Accel_Init_RegList[i][0], BMI088_Accel_Init_RegList[i][1]);
                HAL_Delay(1);
        }
}

static void BMI088_Gyro_Init(IMU_HandleTypeDef *himu)
{
        //通讯检查
        BMI088_Gyro_Read_Single_Reg(himu, BMI088_GYRO_CHIP_ID);
        HAL_Delay(1);
        BMI088_Gyro_Read_Single_Reg(himu, BMI088_GYRO_CHIP_ID);
        HAL_Delay(1);

        //软复位
        BMI088_Write_Single_Reg(&himu->Node[1], BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
        HAL_Delay(BMI088_LONG_DELAY_TIME);

        //复位后通讯检查
        BMI088_Gyro_Read_Single_Reg(himu, BMI088_GYRO_CHIP_ID);
        HAL_Delay(1);
        BMI088_Gyro_Read_Single_Reg(himu, BMI088_GYRO_CHIP_ID);
        HAL_Delay(1);

        //写配置寄存器表
        for (uint8_t i = 0; i < BMI088_WRITE_GYRO_REG_NUM; i++)
        {
                BMI088_Write_Single_Reg(&himu->Node[1], BMI088_Gyro_Init_RegList[i][0], BMI088_Gyro_Init_RegList[i][1]);
                HAL_Delay(1);
        }
}

/*===| 角度/加速度解包 |===*/
//读全部数据: accel 6字节 + gyro 8字节(含chip id) + 温度2字节
static void BMI088_Read(IMU_HandleTypeDef *himu)
{
        uint8_t buf[8] = {0};
        int16_t raw    = 0;

        /*===| 加速度 |===*/
        BMI088_Accel_Read_Multi_Reg(himu, BMI088_ACCEL_XOUT_L, buf, 6);

        raw = (int16_t) ((buf[1] << 8) | buf[0]);
        himu->Accel[0] = (float) raw * himu->IMU_Config_Struct.Accel_SEN * himu->AccelScale;
        raw = (int16_t) ((buf[3] << 8) | buf[2]);
        himu->Accel[1] = (float) raw * himu->IMU_Config_Struct.Accel_SEN * himu->AccelScale;
        raw = (int16_t) ((buf[5] << 8) | buf[4]);
        himu->Accel[2] = (float) raw * himu->IMU_Config_Struct.Accel_SEN * himu->AccelScale;

        /*===| 角速度: 从 GYRO_CHIP_ID 起读8字节, buf[0]=chip id, buf[1]=RATE, buf[2..7]=XYZ |===*/
        BMI088_Gyro_Read_Multi_Reg(himu, BMI088_GYRO_CHIP_ID, buf, 8);

        if (buf[0] == BMI088_GYRO_CHIP_ID_VALUE)
        {
                raw = (int16_t) ((buf[3] << 8) | buf[2]);
                himu->Gyro[0] = (float) raw * himu->IMU_Config_Struct.Gyro_SEN - himu->Gyro_Offset[0];
                raw = (int16_t) ((buf[5] << 8) | buf[4]);
                himu->Gyro[1] = (float) raw * himu->IMU_Config_Struct.Gyro_SEN - himu->Gyro_Offset[1];
                raw = (int16_t) ((buf[7] << 8) | buf[6]);
                himu->Gyro[2] = (float) raw * himu->IMU_Config_Struct.Gyro_SEN - himu->Gyro_Offset[2];
        }

        /*===| 温度(在加速度计侧, 0x22=TEMP_M / 0x23=TEMP_L) |===*/
        BMI088_Accel_Read_Multi_Reg(himu, BMI088_TEMP_M, buf, 2);

        raw = (int16_t) ((buf[0] << 3) | (buf[1] >> 5));
        if (raw > 1023) raw -= 2048;

        himu->Temperature = (float) raw * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}

/*===| 开机标定: 静止取样求零漂与标度(与 QMI8658A_Calibration 同语义)
 * 检测到运动则清零重来, 直到采满 BMI088_CALI_NUM 点 |===*/
static void BMI088_Calibration(IMU_HandleTypeDef *himu)
{
        uint16_t Caled_Num      = 0;
        float    Gravity_Offset = 0;
        float    Gyro_Offset[3] = {0};

        himu->AccelScale     = 1.0f;
        himu->Gyro_Offset[0] = 0;
        himu->Gyro_Offset[1] = 0;
        himu->Gyro_Offset[2] = 0;

        while ((float) Caled_Num < BMI088_CALI_NUM)
        {
                BMI088_Read(himu);

                //检测到运动则重新标定
                if (fabsf(himu->Gyro[0]) > BMI088_CALI_GYRO_MAX ||
                    fabsf(himu->Gyro[1]) > BMI088_CALI_GYRO_MAX ||
                    fabsf(himu->Gyro[2]) > BMI088_CALI_GYRO_MAX)
                {
                        Gravity_Offset = 0;
                        Gyro_Offset[0] = 0;
                        Gyro_Offset[1] = 0;
                        Gyro_Offset[2] = 0;
                        Caled_Num      = 0;
                        continue;
                }

                //累加数据
                Gravity_Offset += sqrtf(himu->Accel[0] * himu->Accel[0] +
                                        himu->Accel[1] * himu->Accel[1] +
                                        himu->Accel[2] * himu->Accel[2]);
                Gyro_Offset[0] += himu->Gyro[0];
                Gyro_Offset[1] += himu->Gyro[1];
                Gyro_Offset[2] += himu->Gyro[2];

                Caled_Num++;
                DWT_Delay(0.0001f);
        }

        //求平均并保存标定值
        himu->AccelScale     = BMI088_GRAVITY / (Gravity_Offset / BMI088_CALI_NUM);
        himu->Gyro_Offset[0] = Gyro_Offset[0] / BMI088_CALI_NUM;
        himu->Gyro_Offset[1] = Gyro_Offset[1] / BMI088_CALI_NUM;
        himu->Gyro_Offset[2] = Gyro_Offset[2] / BMI088_CALI_NUM;

        //保存到flash内
        //写flash...
}

/*===| 器件自检: Who Am I + 初始化寄存器读回校验, 返回0=正常(错误码见 drv_bmi088.h) |===*/
static uint8_t BMI088_Check(IMU_HandleTypeDef *himu)
{
        uint8_t error = BMI088_NO_ERROR;

        if (BMI088_Accel_Read_Single_Reg(himu, BMI088_ACC_CHIP_ID) != BMI088_ACC_CHIP_ID_VALUE) error |= BMI088_NO_SENSOR;
        if (BMI088_Gyro_Read_Single_Reg(himu, BMI088_GYRO_CHIP_ID) != BMI088_GYRO_CHIP_ID_VALUE) error |= BMI088_NO_SENSOR;

        for (uint8_t i = 0; i < BMI088_WRITE_ACCEL_REG_NUM; i++)
        {
                if (BMI088_Accel_Read_Single_Reg(himu, BMI088_Accel_Init_RegList[i][0]) != BMI088_Accel_Init_RegList[i][1])
                {
                        error |= BMI088_Accel_Init_RegList[i][2];
                }
        }

        for (uint8_t i = 0; i < BMI088_WRITE_GYRO_REG_NUM; i++)
        {
                if (BMI088_Gyro_Read_Single_Reg(himu, BMI088_Gyro_Init_RegList[i][0]) != BMI088_Gyro_Init_RegList[i][1])
                {
                        error |= BMI088_Gyro_Init_RegList[i][2];
                }
        }

        return error;
}

/*===| 初始化: 写配置 + 开机标定 |===*/
static void BMI088_Init(IMU_HandleTypeDef *himu)
{
        BMI088_Accel_Init(himu);
        BMI088_Gyro_Init(himu);
        BMI088_Calibration(himu);
}

/*=============|OOPC|================*/
IMU_VTable IMU_BMI088_VTable_Default = {
        .check       = BMI088_Check,
        .init        = BMI088_Init,
        .read        = BMI088_Read,
        .calibration = BMI088_Calibration
};

//C型板绑定: SPI1 + accel(CS1_ACCEL=PA4) + gyro(CS1_GYRO=PB0)
IMU_Config_StructTypeDef IMU_BMI088_Config_Default = {
        .vptr = &IMU_BMI088_VTable_Default,

        .hspi  = &hspi1,
        .Port  = CS1_ACCEL_GPIO_Port,
        .Pin   = CS1_ACCEL_Pin,
        .Port2 = CS1_GYRO_GPIO_Port,
        .Pin2  = CS1_GYRO_Pin,

        .Accel_SEN = BMI088_ACCEL_6G_SEN,
        .Gyro_SEN  = BMI088_GYRO_2000_SEN
};
