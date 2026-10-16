//
// Created by ye on 2026/8/22.
//

#include "drv_motor_zdt_emm_V5.h"

#include <math.h>
#include <string.h>

#include "Emm_V5.h"
#include "utils.h"

/*===| 反馈轮询降频(按控制周期计数) |===*/
#define ZDT_POLL_POS_TICKS  3
#define ZDT_POLL_FLAG_TICKS 10

/*===| 传输出口: 按 cmd[0] 的设备地址分发到节点 |===*/
#define ZDT_ADDR_MAX 16

static UART_Node_HandleTypeDef *ZDT_Node_Table[ZDT_ADDR_MAX];

static void Motor_ZDT_Tx(const uint8_t *cmd, uint16_t len)
{
        uint8_t addr = cmd[0];
        if (addr >= ZDT_ADDR_MAX) return;

        UART_Node_HandleTypeDef *node = ZDT_Node_Table[addr];
        if (node == NULL) return;

        UART_Node_Transmit_Queue(node, cmd, len);
}

//帧长按功能码判定: 0x6B 虽为帧尾但数据段可能含 0x6B, 故不能靠扫帧尾
static uint16_t Motor_ZDT_FrameParser(const uint8_t *data, uint16_t length)
{
        //首字节必须是已注册的电机地址, 否则丢1字节重新同步
        if (data[0] == 0 || data[0] >= ZDT_ADDR_MAX || ZDT_Node_Table[data[0]] == NULL) return 1;

        switch (data[1])
        {
        case 0x00: return 4; //错误应答: 地址 + 0x00 + 错误码 + 校验
        case 0x3A: return 4; //状态标志
        case 0x35: return 6; //实时转速
        case 0x36: return 8; //实时位置
        case 0x32: return 8; //输入脉冲数
        case 0x33: return 8; //目标位置
        default:   return 1; //未知功能码
        }
}

void Motor_ZDT_Ctor(Motor_ZDT_HandleTypeDef *hmotor_zdt, UART_HandleTypeDef *huart, uint8_t Addr)
{
        memset(hmotor_zdt, 0, sizeof(Motor_ZDT_HandleTypeDef));

        Motor_Base_Ctor(&hmotor_zdt->hmotor_base, &Motor_ZDT_Config_Default.Base);

        UART_Node_Ctor(&hmotor_zdt->hmotor_base.Link.UART, huart, Addr, Motor_Bus_Node_Handler,
                       MOTOR_TIMEOUT, MOTOR_ERR_COUNT_MAX, Motor_Err_Handler);

        ZDT_Node_Table[Addr] = &hmotor_zdt->hmotor_base.Link.UART;
        Emm_V5_Tx            = Motor_ZDT_Tx;

        UART_Bus_SetFrameParser(huart, Motor_ZDT_FrameParser);
}

/*===| 使能 / 失能 / 置零 |===*/
static void Motor_ZDT_Enable(Motor_Base_HandleTypeDef *hmotor_base)
{
        Emm_V5_En_Control((uint8_t) hmotor_base->Link.Node.Device_ID, true, false);
}

static void Motor_ZDT_Disable(Motor_Base_HandleTypeDef *hmotor_base)
{
        Emm_V5_En_Control((uint8_t) hmotor_base->Link.Node.Device_ID, false, false);
}

static void Motor_ZDT_Set_Zero(Motor_Base_HandleTypeDef *hmotor_base)
{
        hmotor_base->Total_Angle_Offset = 0;

        Emm_V5_Reset_CurPos_To_Zero((uint8_t) hmotor_base->Link.Node.Device_ID);
}

/*===| 轮询反馈帧 |===*/
static void Motor_ZDT_Update(Motor_Base_HandleTypeDef *hmotor_base)
{
        Motor_ZDT_HandleTypeDef *hmotor_zdt = container_of(hmotor_base, Motor_ZDT_HandleTypeDef, hmotor_base);
        uint8_t addr = (uint8_t) hmotor_base->Link.Node.Device_ID;

        hmotor_zdt->Poll_Count++;

        if (hmotor_zdt->Poll_Count % ZDT_POLL_FLAG_TICKS == 0)      Emm_V5_Read_Sys_Params(addr, S_FLAG);
        else if (hmotor_zdt->Poll_Count % ZDT_POLL_POS_TICKS == 0)  Emm_V5_Read_Sys_Params(addr, S_CPOS);
        else if (hmotor_zdt->Poll_Count % ZDT_POLL_POS_TICKS == 1)  Emm_V5_Read_Sys_Params(addr, S_VEL);
}

/*===| 下发: 位置(0xFD) / 速度(0xF6); 目标未变化则不发 |===*/
static void Motor_ZDT_Send(Motor_Base_HandleTypeDef *hmotor_base, float _pos, float _vel, float _KP, float _KD, float _torq)
{
        Motor_ZDT_HandleTypeDef *hmotor_zdt = container_of(hmotor_base, Motor_ZDT_HandleTypeDef, hmotor_base);
        Motor_ZDT_Config_StructTypeDef *cfg = &Motor_ZDT_Config_Default;

        if (hmotor_base->Status_Enum == MOTOR_TORQUE) return; //Emm_V5 无力矩控制

        uint8_t addr = (uint8_t) hmotor_base->Link.Node.Device_ID;
        float   target = (hmotor_base->Status_Enum == MOTOR_SPEED) ? _vel : _pos;

        if (target == hmotor_zdt->Last_Target && hmotor_base->Status_Enum == hmotor_zdt->Last_Status) return;

        hmotor_zdt->Last_Target = target;
        hmotor_zdt->Last_Status = (uint8_t) hmotor_base->Status_Enum;

        if (hmotor_base->Status_Enum == MOTOR_SPEED)
        {
                uint8_t  dir = (_vel < 0.0f) ? 1 : 0;
                uint16_t vel = (uint16_t) fabsf(_vel);

                Emm_V5_Vel_Control(addr, dir, vel, cfg->Default_Acc, false);
        }
        else
        {
                uint16_t vel = (uint16_t) fabsf(_vel);
                if (vel == 0) vel = cfg->Default_Vel;

                int32_t clk = (int32_t) (fabsf(_pos) / 360.0f * (float) cfg->Pulse_Per_Rev);

                //绝对模式(raF=true), 方向由符号位决定故 dir 固定为0
                Emm_V5_Pos_Control(addr, 0, vel, cfg->Default_Acc, (uint32_t) clk, true, false);
        }
}

/*===| 反馈解包: 0x36 实时位置 / 0x35 实时转速 / 0x3A 状态标志 |===*/
static void Motor_ZDT_On_Frame(Motor_Base_HandleTypeDef *hmotor_base)
{
        const uint8_t *Data = hmotor_base->Link.Node.Rx_Data;

        if (hmotor_base->Link.Node.Rx_Length < 4) return;
        if (Data[0] != (uint8_t) hmotor_base->Link.Node.Device_ID) return;

        if (Data[1] == 0x00) //错误应答: 地址 + 0x00 + 错误码 + 校验
        {
                hmotor_base->Error_Code = Data[2];
                return;
        }

        int32_t sign = (Data[2] != 0) ? -1 : 1;

        switch (Data[1])
        {
        case 0x36: //实时位置: 65536/圈, 多圈有符号
        {
                int32_t pos   = (int32_t) ((uint32_t) Data[3] << 24 | (uint32_t) Data[4] << 16 | (uint32_t) Data[5] << 8 | Data[6]);
                float   total = (float) (sign * pos) * 360.0f / 65536.0f;

                hmotor_base->Encoder     = (int16_t) pos;
                hmotor_base->Total_Angle = total - hmotor_base->Total_Angle_Offset;
                hmotor_base->Angle       = fmodf(hmotor_base->Total_Angle, 360.0f);
                hmotor_base->Round       = (int32_t) (total / 360.0f);
                break;
        }
        case 0x35: //实时转速: RPM
        {
                int16_t vel = (int16_t) ((Data[3] << 8) | Data[4]);

                hmotor_base->Speed = (float) (sign * vel);
                break;
        }
        case 0x3A: //状态标志: bit0 使能 / bit1 到位 / bit2 堵转 / bit3 堵转保护
        {
                uint8_t flag = Data[2];

                hmotor_base->Error_Code = (flag & 0x01) ? hmotor_base->Error_Code : 0;
                break;
        }
        default: return;
        }

        hmotor_base->Link.Node.herr.tick   = 0;
        hmotor_base->Link.Node.herr.count  = 0;
        hmotor_base->Link.Node.herr.If_Err = 0;
}

/*=============|OOPC|================*/
Motor_VTable Motor_ZDT_VTable_Default = {
        .enable   = Motor_ZDT_Enable,
        .disable  = Motor_ZDT_Disable,
        .set_zero = Motor_ZDT_Set_Zero,
        .update   = Motor_ZDT_Update,
        .send     = Motor_ZDT_Send,
        .on_frame = Motor_ZDT_On_Frame
};

Motor_ZDT_Config_StructTypeDef Motor_ZDT_Config_Default = {
        .Base = {
                .vptr                = &Motor_ZDT_VTable_Default,
                .Motor_Enum          = MOTOR_ZDT,
                .Status_Enum         = MOTOR_ANGLE,
                .Is_Feedback_Control = false, //电调内闭环
        },

        .Pulse_Per_Rev = 3200, //16细分(200步/圈 × 16)
        .Default_Vel   = 1000,
        .Default_Acc   = 0,
};
