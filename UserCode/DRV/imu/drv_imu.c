//============================================================================================================================================================================
//                                              IMU_Driver[IMU驱动基类]
//      包含IMU句柄的实例、通用构造与vtable转发, 各具体IMU驱动见 drv_bmi088 / drv_qmi8658a
//============================================================================================================================================================================

#include "drv_imu.h"

#include <string.h>

/*===| 板载IMU实例(具体跑哪个驱动, 由 ENT 构造时传入的 Config 决定) |===*/
IMU_HandleTypeDef himu1 = {};

/*===| IMU通用构造: 绑定配置 + 按配置构造SPI节点(参考 Motor_Ctor) |===*/
void IMU_Ctor(IMU_HandleTypeDef *himu, IMU_Config_StructTypeDef IMU_Config_Struct)
{
        if (himu == NULL) return;

        memset(himu, 0, sizeof(IMU_HandleTypeDef));

        himu->IMU_Config_Struct = IMU_Config_Struct;

        //节点0: 主通道(必有)
        SPI_Node_Ctor(&himu->Node[0], IMU_Config_Struct.hspi, IMU_Config_Struct.Port, IMU_Config_Struct.Pin, NULL);

        //节点1: 第二片选(BMI088的gyro); 单CS设备(如QMI8658A)不构造
        if (IMU_Config_Struct.Port2 != NULL)
        {
                SPI_Node_Ctor(&himu->Node[1], IMU_Config_Struct.hspi, IMU_Config_Struct.Port2, IMU_Config_Struct.Pin2, NULL);
        }
}
