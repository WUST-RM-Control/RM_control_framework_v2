//
// Created by ye on 2026/8/22.
//

#include "drv_motor_dji.h"
#include "fdcan.h"
#include "hal_can.h"
#include "utils.h"


//电机-大疆-发送电流控制帧（FDCAN，控制帧ID，电机1电流，电机2电流，电机3电流，电机4电流）
void Motor_DJI_SendCurrent(FDCAN_HandleTypeDef *hfdcan, uint16_t CAN_ID, int16_t ID1_Currnet, int16_t ID2_Currnet,int16_t ID3_Currnet, int16_t ID4_Currnet)
{
        //发出去就行，错误处理和回调跟这个没关系
        CAN_Node_HandleTypeDef hcan_node = {
                .herr = {
                        .tick          = 0,
                        .tick_timeout  = 0,
                        .count         = 0,
                        .count_maximum = 0,
                        .handler           = (void(*)(Err_HandleTypeDef *herror))null_function,
                },
                .hfdcan          = hfdcan,
                .CAN_Send_ID     = CAN_ID,
                .CAN_Feedback_ID = 0x000,
                .handler         = (void(*)(CAN_Node_HandleTypeDef *node, const uint8_t *Data)) null_function,
        };

        uint8_t Motor_Tx_Data[8];
        Motor_Tx_Data[0] = ID1_Currnet >> 8;
        Motor_Tx_Data[1] = ID1_Currnet & 0xFF;
        Motor_Tx_Data[2] = ID2_Currnet >> 8;
        Motor_Tx_Data[3] = ID2_Currnet & 0xFF;
        Motor_Tx_Data[4] = ID3_Currnet >> 8;
        Motor_Tx_Data[5] = ID3_Currnet & 0xFF;
        Motor_Tx_Data[6] = ID4_Currnet >> 8;
        Motor_Tx_Data[7] = ID4_Currnet & 0xFF;

        CAN_Send_Data_STD(&hcan_node, Motor_Tx_Data);
        //讨厌一拖四，为什么要四个电机绑定在一起，每个电机单独控制不好吗
}



//电机-大疆-存储电机反馈数据（电机反馈数组，电机数据结构体）
void Motor_DJI_Storage_Data(Motor_HandleTypeDef *hmotor, const uint8_t *Data)
{
        /*===| 协议解包 |===*/
        hmotor->Encoder       = (int16_t) (Data[0] << 8 | Data[1]);
        hmotor->Speed         = (int16_t) (Data[2] << 8 | Data[3]);
        hmotor->Torque = (int16_t) (Data[4] << 8 | Data[5]);
        hmotor->Temperature   = (int8_t) (Data[6]);

        hmotor->Angle         = ((float) hmotor->Encoder - 4096.0f) * 180.0f / 4096.0f;

        /*===| 得到总角度值 |===*/
        if (hmotor->Encoder - hmotor->Encoder_Last > 4096) hmotor->Round--;
        else if (hmotor->Encoder - hmotor->Encoder_Last < -4096) hmotor->Round++;
        hmotor->Total_Angle = 360.0f * ((float)hmotor->Round + (float)hmotor->Encoder / 8192.0f) - hmotor->Total_Angle_Offset;

        /*===| 记录编码器值 |===*/
        hmotor->Encoder_Last = hmotor->Encoder;
        hmotor->Angle_Last   = hmotor->Angle;

        hmotor->Node.herr.tick      = 0;
        hmotor->Node.herr.count     = 0;
        hmotor->Node.herr.If_Online = 1;
}


/*=============|OOPC|================*/
Motor_VTable Motor_DJI_VTable_Default = {
        .enable       = (void(*)(Motor_HandleTypeDef *hmotor))null_function,
        .disable      = (void(*)(Motor_HandleTypeDef *hmotor))null_function,
        .set_zero     = Motor_DJI_Set_Zero,
        .storage_data = Motor_DJI_Storage_Data
};

