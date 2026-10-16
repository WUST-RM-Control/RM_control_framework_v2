//
// Created by ye on 2026/8/22.
//

#ifndef G4MINI_V3_DRV_MOTOR_DJI_H
#define G4MINI_V3_DRV_MOTOR_DJI_H
#include <stdint.h>

#include "drv_motor.h"

/*===| 一拖四报文槽位约定(DJI类电调) |===*/
#define MOTOR_GROUP_MAX 4
//回报ID = 0x200 + 电调ID; 报文每帧4槽, 槽位 = (电调ID - 1) % 4
#define MOTOR_FEEDBACK_ID_BASE 0x200

typedef struct Motor_DJI_HandleTypeDef   Motor_DJI_HandleTypeDef;
typedef struct Motor_Group_HandleTypeDef Motor_DJIGroup_HandleTypeDef;

struct Motor_Group_HandleTypeDef
{
        Motor_DJI_HandleTypeDef *member[MOTOR_GROUP_MAX];
};

struct Motor_DJI_HandleTypeDef
{
        Motor_Base_HandleTypeDef hmotor_base;

        Motor_DJIGroup_HandleTypeDef *Group; //组(NULL=单发设备); 一拖四设备组内共享, 由 Motor_Group_Init 回填

        Feedforward_t FFC_Angle_Struct;
        PID_t         PID_Angle_Struct;
        Feedforward_t FFC_Speed_Struct;
        PID_t         PID_Speed_Struct;
};

typedef struct
{
        Motor_Enum_TypeDef Motor_Enum;

        Err_HandleTypeDef herr;

        uint32_t CAN_Feedback_ID_Mask; //反馈ID掩码

        Motor_VTable *vptr;

        uint8_t Error_Code; //反馈错误码(1=正常, 0=失能, 3~E=故障; DJI不更新恒为1)

        Motor_Status_TypeDef Status_Enum;
} Motor_DJI_Config_StructTypeDef;

void Motor_DJI_SendCurrent(CAN_HandleTypeDef *hcan, uint16_t CAN_ID, int16_t ID1_Current, int16_t ID2_Current, int16_t ID3_Current, int16_t ID4_Current);

//电机发送组初始化: 回填组内成员的 Group 反向指针(成员数组需先静态初始化好)
//组长 member[0] 为空则忽略(不建组)
void Motor_DJI_Group_Init(Motor_DJIGroup_HandleTypeDef *group, Motor_DJI_HandleTypeDef *hmotor_dji1, Motor_DJI_HandleTypeDef *hmotor_dji2, Motor_DJI_HandleTypeDef *hmotor_dji3, Motor_DJI_HandleTypeDef *hmotor_dji4);

//单电机力矩发送: DJI一拖四, 实际由组长一次性组帧(组内其余成员调用为空操作)
//需先用 Motor_Group_Init 建组, 否则不发送
void Motor_DJI_Send_Torque(Motor_Base_HandleTypeDef *hmotor_base, float torque);

void Motor_DJI_Storage_Data(Motor_Base_HandleTypeDef *hmotor_base, const uint8_t *Data);

//由回报ID推算该电机在组帧报文里的槽位(0~3): 与建组时的数组顺序无关
__STATIC_INLINE uint8_t Motor_Get_FrameSlot(Motor_DJI_HandleTypeDef *hmotor_dji)
{
        return (uint8_t) ((hmotor_dji->hmotor_base.Node.CAN_Feedback_ID - MOTOR_FEEDBACK_ID_BASE - 1U) % MOTOR_GROUP_MAX);
}

extern Motor_VTable               Motor_DJI_VTable_Default;

__STATIC_INLINE void Motor_DJI_Set_Zero(Motor_Base_HandleTypeDef *hmotor) { hmotor->Total_Angle_Offset = hmotor->Total_Angle; }

#endif //G4MINI_V3_DRV_MOTOR_DJI_H
