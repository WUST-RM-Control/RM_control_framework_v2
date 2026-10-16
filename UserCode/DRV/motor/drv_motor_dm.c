//
// Created by ye on 2026/8/22.
//

#include "drv_motor_dm.h"

#include "arm_math.h"
#include "hal_can.h"
#include "utils.h"

/*===| 命令层 |===*/
void Motor_DM_CMD_Enable(Motor_Base_HandleTypeDef *hmotor_base)
{
        uint8_t DM_Enable_Data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};

        CAN_Send_Data_STD(&hmotor_base->Link.CAN, DM_Enable_Data);
}

void Motor_DM_CMD_Disable(Motor_Base_HandleTypeDef *hmotor_base)
{
        uint8_t DM_Disable_Data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};

        CAN_Send_Data_STD(&hmotor_base->Link.CAN, DM_Disable_Data);
}

void Motor_DM_CMD_SetZero(Motor_Base_HandleTypeDef *hmotor_base)
{
        uint8_t DM_SetZero_Data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};

        CAN_Send_Data_STD(&hmotor_base->Link.CAN, DM_SetZero_Data);
}

void Motor_DM_CMD_ClearErr(Motor_Base_HandleTypeDef *hmotor_base)
{
        uint8_t DM_ClearErr_Data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};

        CAN_Send_Data_STD(&hmotor_base->Link.CAN, DM_ClearErr_Data);
}

//MIT 5 参数: 位置/速度/Kp/Kd/前馈力矩
void Motor_DM_CMD_MIT(Motor_Base_HandleTypeDef *hmotor_base, float _pos, float _vel, float _KP, float _KD, float _torq)
{
        uint16_t pos_tmp = float_to_uint(_pos, P_MIN, P_MAX, 16);
        uint16_t vel_tmp = float_to_uint(_vel, V_MIN, V_MAX, 12);
        uint16_t kp_tmp  = float_to_uint(_KP, KP_MIN, KP_MAX, 12);
        uint16_t kd_tmp  = float_to_uint(_KD, KD_MIN, KD_MAX, 12);
        uint16_t tor_tmp = float_to_uint(_torq, T_MIN, T_MAX, 12);

        uint8_t DM_Send_Data[8] = {0};

        DM_Send_Data[0] = (pos_tmp >> 8);
        DM_Send_Data[1] = pos_tmp;
        DM_Send_Data[2] = (vel_tmp >> 4);
        DM_Send_Data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
        DM_Send_Data[4] = kp_tmp;
        DM_Send_Data[5] = (kd_tmp >> 4);
        DM_Send_Data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
        DM_Send_Data[7] = tor_tmp;

        CAN_Send_Data_STD(&hmotor_base->Link.CAN, DM_Send_Data);
}

void Motor_DM_CMD_Speed(Motor_Base_HandleTypeDef *hmotor_base, float Speed)
{
        float Speed_Buf[2] = {Speed, 0}; //整帧8字节, 补足避免越界读

        CAN_Send_Data_STD(&hmotor_base->Link.CAN, (uint8_t *) Speed_Buf);
}

/*===| 构造 |===*/
void Motor_DM_Ctor(Motor_DM_HandleTypeDef *hmotor_dm, CAN_HandleTypeDef *hcan, uint32_t CAN_Send_ID, uint32_t CAN_Feedback_ID)
{
        memset(hmotor_dm, 0, sizeof(Motor_DM_HandleTypeDef));

        Motor_Base_Ctor(&hmotor_dm->hmotor_base, &Motor_DM_Config_Default.Base);

        CAN_Node_Ctor(&hmotor_dm->hmotor_base.Link.CAN, hcan, CAN_Send_ID, CAN_Feedback_ID,
                      CAN_FEEDBACK_ID_MASK_NONE, Motor_Bus_Node_Handler,
                      MOTOR_TIMEOUT, MOTOR_ERR_COUNT_MAX, Motor_Err_Handler);

        Motor_Loop_Init(&hmotor_dm->Ctrl_Angle, &Motor_DM_Config_Default.PID_Angle, &Motor_DM_Config_Default.FFC_Angle);
        Motor_Loop_Init(&hmotor_dm->Ctrl_Speed, &Motor_DM_Config_Default.PID_Speed, &Motor_DM_Config_Default.FFC_Speed);
}

/*===| 使能: 清错 + 使能 + 复位积分(会引入约2ms延迟) |===*/
static void Motor_DM_Enable(Motor_Base_HandleTypeDef *hmotor_base)
{
        Motor_DM_HandleTypeDef *hmotor_dm = container_of(hmotor_base, Motor_DM_HandleTypeDef, hmotor_base);

        Motor_DM_CMD_ClearErr(hmotor_base);
        vTaskDelay(1);
        Motor_DM_CMD_Enable(hmotor_base);
        vTaskDelay(1);

        PID_Reset(&hmotor_dm->Ctrl_Angle.PID);
        PID_Reset(&hmotor_dm->Ctrl_Speed.PID);
}

/*===| 闭环计算: 角度环 → 速度环 |===*/
static void Motor_DM_Update(Motor_Base_HandleTypeDef *hmotor_base)
{
        Motor_DM_HandleTypeDef *hmotor_dm = container_of(hmotor_base, Motor_DM_HandleTypeDef, hmotor_base);

        if (!hmotor_base->Is_Feedback_Control) return;

        if (hmotor_base->Status_Enum == MOTOR_ANGLE)
        {
                hmotor_base->Target_Speed = PID_Calculate(&hmotor_dm->Ctrl_Angle.PID, hmotor_base->Angle, hmotor_base->Target_Angle)
                                          + Feedforward_Calculate(&hmotor_dm->Ctrl_Angle.FFC, hmotor_base->Target_Angle);
        }

        if (hmotor_base->Status_Enum == MOTOR_SPEED || hmotor_base->Status_Enum == MOTOR_ANGLE)
        {
                hmotor_base->Target_Torque = PID_Calculate(&hmotor_dm->Ctrl_Speed.PID, hmotor_base->Speed, hmotor_base->Target_Speed)
                                           + Feedforward_Calculate(&hmotor_dm->Ctrl_Speed.FFC, hmotor_base->Target_Speed);
        }
}

/*===| 反馈解包 |===*/
static void Motor_DM_On_Frame(Motor_Base_HandleTypeDef *hmotor_base)
{
        const uint8_t *Data = hmotor_base->Link.Node.Rx_Data;

        uint8_t Error_ID    = Data[0] >> 4;
        int     p_int       = (Data[1] << 8) | Data[2];
        int     v_int       = (Data[3] << 4) | (Data[4] >> 4);
        int     t_int       = ((Data[4] & 0xF) << 8) | Data[5];
        int     temperature = Data[7];

        hmotor_base->Error_Code  = Error_ID; //1=正常, 0=失能, 3~E=故障
        hmotor_base->Angle       = uint_to_float(p_int, P_MIN, P_MAX, 16) / 2.0f / PI * 360.0f;
        hmotor_base->Speed       = uint_to_float(v_int, V_MIN, V_MAX, 12) / 2.0f / PI * 360.0f;
        hmotor_base->Torque      = uint_to_float(t_int, T_MIN, T_MAX, 12);
        hmotor_base->Temperature = (int8_t) temperature;

        if (hmotor_base->Angle - hmotor_base->Angle_Last > 180) hmotor_base->Round--;
        else if (hmotor_base->Angle - hmotor_base->Angle_Last < -180) hmotor_base->Round++;

        hmotor_base->Total_Angle = 360.0f * (float) hmotor_base->Round + hmotor_base->Angle - hmotor_base->Total_Angle_Offset;
        hmotor_base->Angle_Last  = hmotor_base->Angle;

        if (Error_ID == 1)
        {
                hmotor_base->Link.Node.herr.tick   = 0;
                hmotor_base->Link.Node.herr.count  = 0;
                hmotor_base->Link.Node.herr.If_Err = 0;
        }
}

/*=============|OOPC|================*/
Motor_VTable Motor_DM_VTable_Default = {
        .enable   = Motor_DM_Enable,
        .disable  = Motor_DM_CMD_Disable,
        .set_zero = Motor_DM_CMD_SetZero,
        .update   = Motor_DM_Update,
        .send     = Motor_DM_CMD_MIT,
        .on_frame = Motor_DM_On_Frame
};

Motor_DM_Config_StructTypeDef Motor_DM_Config_Default = {
        .Base = {
                .vptr                = &Motor_DM_VTable_Default,
                .Motor_Enum          = MOTOR_DM,
                .Status_Enum         = MOTOR_TORQUE,
                .Is_Feedback_Control = true,
        },

        //Kp/Ki/Kd 待整定
        .PID_Angle = { .MaxOut = 500.0f, .IntegralLimit = 100.0f },
        .PID_Speed = { .MaxOut = 16384.0f, .IntegralLimit = 3000.0f },
};
