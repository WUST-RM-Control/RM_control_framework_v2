//
// Created by ye on 2026/8/22.
//

#include "drv_motor_dji.h"

#include "hal_can.h"
#include "utils.h"

/*===| 一拖四发送组 |===*/
void Motor_DJI_Group_Init(Motor_DJIGroup_HandleTypeDef *group, Motor_DJI_HandleTypeDef *hmotor_dji1, Motor_DJI_HandleTypeDef *hmotor_dji2, Motor_DJI_HandleTypeDef *hmotor_dji3, Motor_DJI_HandleTypeDef *hmotor_dji4)
{
        group->member[0] = hmotor_dji1;
        group->member[1] = hmotor_dji2;
        group->member[2] = hmotor_dji3;
        group->member[3] = hmotor_dji4;

        if (group->member[0] == NULL) return;

        for (uint8_t i = 0; i < MOTOR_GROUP_MAX; i++)
        {
                if (group->member[i] != NULL) group->member[i]->Group = group;
        }
}

void Motor_DJI_Ctor(Motor_DJI_HandleTypeDef *hmotor_dji, CAN_HandleTypeDef *hcan, uint32_t CAN_Send_ID, uint32_t CAN_Feedback_ID)
{
        memset(hmotor_dji, 0, sizeof(Motor_DJI_HandleTypeDef));

        Motor_Base_Ctor(&hmotor_dji->hmotor_base, &Motor_DJI_Config_Default.Base);

        CAN_Node_Ctor(&hmotor_dji->hmotor_base.Link.CAN, hcan, CAN_Send_ID, CAN_Feedback_ID,
                      CAN_FEEDBACK_ID_MASK_NONE, Motor_Bus_Node_Handler,
                      MOTOR_TIMEOUT, MOTOR_ERR_COUNT_MAX, Motor_Err_Handler);

        Motor_Loop_Init(&hmotor_dji->Ctrl_Angle, &Motor_DJI_Config_Default.PID_Angle, &Motor_DJI_Config_Default.FFC_Angle);
        Motor_Loop_Init(&hmotor_dji->Ctrl_Speed, &Motor_DJI_Config_Default.PID_Speed, &Motor_DJI_Config_Default.FFC_Speed);
}

/*===| 发送 |===*/
//电机-大疆-发送电流控制帧（CAN，控制帧ID，电机1~4电流）
void Motor_DJI_SendCurrent(CAN_HandleTypeDef *hcan, uint16_t CAN_ID, int16_t ID1_Current, int16_t ID2_Current, int16_t ID3_Current, int16_t ID4_Current)
{
        uint8_t Motor_Tx_Data[8];
        Motor_Tx_Data[0] = ID1_Current >> 8;
        Motor_Tx_Data[1] = ID1_Current & 0xFF;
        Motor_Tx_Data[2] = ID2_Current >> 8;
        Motor_Tx_Data[3] = ID2_Current & 0xFF;
        Motor_Tx_Data[4] = ID3_Current >> 8;
        Motor_Tx_Data[5] = ID3_Current & 0xFF;
        Motor_Tx_Data[6] = ID4_Current >> 8;
        Motor_Tx_Data[7] = ID4_Current & 0xFF;

        CAN_Send_STD(hcan, CAN_ID, Motor_Tx_Data);
}

//DJI只支持力矩: 一拖四由组长一次性组帧, 其余成员空操作
static void Motor_DJI_Send(Motor_Base_HandleTypeDef *hmotor_base, float _pos, float _vel, float _KP, float _KD, float _torq)
{
        Motor_DJI_HandleTypeDef *hmotor_dji = container_of(hmotor_base, Motor_DJI_HandleTypeDef, hmotor_base);

        Motor_DJIGroup_HandleTypeDef *group = hmotor_dji->Group;
        if (group == NULL) return;
        if (group->member[0] != hmotor_dji) return;

        int16_t current[MOTOR_GROUP_MAX] = {0};

        for (uint8_t i = 0; i < MOTOR_GROUP_MAX; i++)
        {
                if (group->member[i] == NULL) continue;

                uint8_t slot  = Motor_Get_FrameSlot(group->member[i]);
                current[slot] = (int16_t) group->member[i]->hmotor_base.Target_Torque;
        }

        Motor_DJI_SendCurrent(group->member[0]->hmotor_base.Link.CAN.hcan,
                              group->member[0]->hmotor_base.Link.CAN.Send_ID,
                              current[0], current[1], current[2], current[3]);
}

/*===| 闭环计算: 角度环 → 速度环 |===*/
static void Motor_DJI_Update(Motor_Base_HandleTypeDef *hmotor_base)
{
        Motor_DJI_HandleTypeDef *hmotor_dji = container_of(hmotor_base, Motor_DJI_HandleTypeDef, hmotor_base);

        if (!hmotor_base->Is_Feedback_Control) return;

        if (hmotor_base->Status_Enum == MOTOR_ANGLE)
        {
                hmotor_base->Target_Speed = PID_Calculate(&hmotor_dji->Ctrl_Angle.PID, hmotor_base->Angle, hmotor_base->Target_Angle)
                                          + Feedforward_Calculate(&hmotor_dji->Ctrl_Angle.FFC, hmotor_base->Target_Angle);
        }

        if (hmotor_base->Status_Enum == MOTOR_SPEED || hmotor_base->Status_Enum == MOTOR_ANGLE)
        {
                hmotor_base->Target_Torque = PID_Calculate(&hmotor_dji->Ctrl_Speed.PID, hmotor_base->Speed, hmotor_base->Target_Speed)
                                           + Feedforward_Calculate(&hmotor_dji->Ctrl_Speed.FFC, hmotor_base->Target_Speed);
        }
}

/*===| 反馈解包 |===*/
static void Motor_DJI_On_Frame(Motor_Base_HandleTypeDef *hmotor_base)
{
        const uint8_t *Data = hmotor_base->Link.Node.Rx_Data;

        hmotor_base->Encoder     = (int16_t) (Data[0] << 8 | Data[1]);
        hmotor_base->Speed       = (int16_t) (Data[2] << 8 | Data[3]);
        hmotor_base->Torque      = (int16_t) (Data[4] << 8 | Data[5]);
        hmotor_base->Temperature = (int8_t) (Data[6]);

        hmotor_base->Angle = ((float) hmotor_base->Encoder - 4096.0f) * 180.0f / 4096.0f;

        if (hmotor_base->Encoder - hmotor_base->Encoder_Last > 4096) hmotor_base->Round--;
        else if (hmotor_base->Encoder - hmotor_base->Encoder_Last < -4096) hmotor_base->Round++;

        hmotor_base->Total_Angle = 360.0f * ((float) hmotor_base->Round + (float) hmotor_base->Encoder / 8192.0f) - hmotor_base->Total_Angle_Offset;

        Motor_Get_TotalAngle_Speed(hmotor_base, 0.3f);

        hmotor_base->Encoder_Last = hmotor_base->Encoder;
        hmotor_base->Angle_Last   = hmotor_base->Angle;

        hmotor_base->Link.Node.herr.tick   = 0;
        hmotor_base->Link.Node.herr.count  = 0;
        hmotor_base->Link.Node.herr.If_Err = 0;
}

/*=============|OOPC|================*/
Motor_VTable Motor_DJI_VTable_Default = {
        .enable   = Motor_Null_Void, //电调上电自使能
        .disable  = Motor_Null_Void,
        .set_zero = Motor_DJI_Set_Zero,
        .update   = Motor_DJI_Update,
        .send     = Motor_DJI_Send,
        .on_frame = Motor_DJI_On_Frame
};

Motor_DJI_Config_StructTypeDef Motor_DJI_Config_Default = {
        .Base = {
                .vptr                = &Motor_DJI_VTable_Default,
                .Motor_Enum          = MOTOR_DJI,
                .Status_Enum         = MOTOR_TORQUE,
                .Is_Feedback_Control = true,
        },

        //Kp/Ki/Kd 待整定
        .PID_Angle = { .MaxOut = 500.0f, .IntegralLimit = 100.0f },
        .PID_Speed = { .MaxOut = 16384.0f, .IntegralLimit = 3000.0f },
};
