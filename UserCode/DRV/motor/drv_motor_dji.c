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
        uint8_t Motor_Tx_Data[8];
        Motor_Tx_Data[0] = ID1_Currnet >> 8;
        Motor_Tx_Data[1] = ID1_Currnet & 0xFF;
        Motor_Tx_Data[2] = ID2_Currnet >> 8;
        Motor_Tx_Data[3] = ID2_Currnet & 0xFF;
        Motor_Tx_Data[4] = ID3_Currnet >> 8;
        Motor_Tx_Data[5] = ID3_Currnet & 0xFF;
        Motor_Tx_Data[6] = ID4_Currnet >> 8;
        Motor_Tx_Data[7] = ID4_Currnet & 0xFF;

        CAN_Send_Data_STD(hfdcan, CAN_ID, Motor_Tx_Data);
}



//电机-大疆-存储电机反馈数据（电机反馈数组，电机数据结构体）
void Motor_DJI_Storage_Data(Motor_HandleTypeDef *hmotor, const uint8_t *Data)
{
        /*===| 协议解包 |===*/
        hmotor->Encoder       = (int16_t) (Data[0] << 8 | Data[1]);
        // hmotor->Speed         = (int16_t) (Data[2] << 8 | Data[3]);
        hmotor->Torque        = (int16_t) (Data[4] << 8 | Data[5]);
        hmotor->Temperature   = (int8_t) (Data[6]);

        hmotor->Angle         = ((float) hmotor->Encoder - 4096.0f) * 180.0f / 4096.0f;

        /*===| 得到总角度值 |===*/
        if (hmotor->Encoder - hmotor->Encoder_Last > 4096) hmotor->Round--;
        else if (hmotor->Encoder - hmotor->Encoder_Last < -4096) hmotor->Round++;
        hmotor->Total_Angle   = 360.0f * ((float)hmotor->Round + (float)hmotor->Encoder / 8192.0f) - hmotor->Total_Angle_Offset;

        //DJI电机统一使用角度差计算速度，精度更高
        Motor_Get_TotalAngle_Speed(hmotor, 0.3f);

        /*===| 记录编码器值 |===*/
        hmotor->Encoder_Last  = hmotor->Encoder;
        hmotor->Angle_Last    = hmotor->Angle;

        hmotor->Ticker        = 0;
        hmotor->If_Online     = 1;
}


/*=============|OOPC|================*/

static Motor_VTable Motor_DJI_VTable_Default = {
        .enable       = (void(*)(Motor_HandleTypeDef *hmotor))null_function,
        .disable      = (void(*)(Motor_HandleTypeDef *hmotor))null_function,
        .set_zero     = Motor_DJI_Set_Zero,
        .storage_data = Motor_DJI_Storage_Data
};

//创建DJI电机对象
void Motor_DJI_Ctor(Motor_HandleTypeDef *hmotor,FDCAN_HandleTypeDef *hfdcan, uint16_t CAN_Send_ID, uint16_t CAN_Feedback_ID)
{
        memset(hmotor, 0, sizeof(Motor_HandleTypeDef));

        CAN_Node_Ctor(&hmotor->Node, hfdcan, CAN_Send_ID, CAN_Feedback_ID);
        hmotor->vptr = &Motor_DJI_VTable_Default;
        hmotor->Error_Code = 1;//默认使能
}
