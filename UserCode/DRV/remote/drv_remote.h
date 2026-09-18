//
// Created by ye on 2026/8/26.
//

#ifndef G4MINI_V3_DRV_REMOTE_H
#define G4MINI_V3_DRV_REMOTE_H

#include "main.h"
#include "err.h"

/** 《遥控器信息图》
    -1.0f ┌───┐ 1.0f
 *        │Sid│
 *        └───┘
 *                         ┌───────────────────────────────────────────┐
 *                         │ ┌───┐1  1.0f                1.0f  ┌───┐1  │
 *                         │ │S 1│3   Λ                    Λ   │S 2│3  │
 *                         │ └───┘2   │                    │   └───┘2  │
 *                         │          │                    │           │
 *                         │ CH2<─────┼─────>1.0fCH0<──────┼─────>1.0f │
 *                         │          │                    │           │
 *                         │          │                    │           │
 *                         │          V                    V           │
 *                         │         CH3                  CH1          │
 *                         └───────────────────────────────────────────┘
 */

/*===| 遥控器相关定义 |===*/
#define SW_Up   1
#define SW_Mid  3
#define SW_Down 2

#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)

/*===| 遥控器错误检测参数(错误监控周期 10ms) |===*/
#define REMOTE_OFFLINE_TIMEOUT  10  //超过 10 个监控周期(100ms)未收到数据则判为失联
#define REMOTE_ERR_COUNT_MAX    10  //错误次数上限(与电机一致)

/*===| 遥控器统一数据(DT7/VT03 解析后都存入此结构) |===*/
typedef __PACKED_STRUCT
{
        float   RC_Right_X;
        float   RC_Right_Y;
        float   RC_Left_X;
        float   RC_Left_Y;
        float   RC_Side;
        uint8_t S1;
        uint8_t S2;

        float   Mouse_Speed_X;
        float   Mouse_Speed_Y;
        float   Mouse_Speed_Z;
        uint8_t Mouse_Press_L;
        uint8_t Mouse_Press_R;

        uint8_t Keyboard_W;
        uint8_t Keyboard_A;
        uint8_t Keyboard_S;
        uint8_t Keyboard_D;
        uint8_t Keyboard_Q;
        uint8_t Keyboard_E;
        uint8_t Keyboard_Shift;
        uint8_t Keyboard_Ctrl;
        uint8_t Keyboard_R;
        uint8_t Keyboard_F;
        uint8_t Keyboard_G;
        uint8_t Keyboard_Z;
        uint8_t Keyboard_X;
        uint8_t Keyboard_C;
        uint8_t Keyboard_V;
        uint8_t Keyboard_B;
} Remote_Data_StructTypeDef;

/*===| VT03 图传链路-键鼠原始数据 |===*/
typedef __PACKED_STRUCT
{
        int16_t  Mouse_Speed_X;
        int16_t  Mouse_Speed_Y;
        int16_t  Mouse_Speed_Z;
        uint8_t  Mouse_Left;
        uint8_t  Mouse_Right;
        uint16_t KeyBoard;
        uint16_t Reserved;
} Keyboard_Mouse_Data_StructTypedef;

/*===| 遥控器句柄(公共, 参考 Motor_HandleTypeDef) |===*/
typedef struct Remote_HandleTypeDef Remote_HandleTypeDef;
typedef struct Remote_VTable        Remote_VTable;

struct Remote_VTable
{
        void (*get_data)(Remote_HandleTypeDef *hremote, const uint8_t *DataBuff); //协议解包
};

struct Remote_HandleTypeDef
{
        Err_HandleTypeDef herr; //错误检测(心跳计时, 参考 Motor_HandleTypeDef 的 Node.herr)

        Remote_VTable *   vptr;

        UART_HandleTypeDef *huart; //串口句柄(DT7使用, VT03为NULL)

        uint8_t             If_Data_New; //是否有新数据

        Remote_Data_StructTypeDef Data;      //当前数据
        Remote_Data_StructTypeDef Data_Last; //上一次数据(按键边沿检测)
};

/*===| 两个"遥控器"实例: DT7遥控器 / VT03图传 |===*/
extern Remote_HandleTypeDef hremote_dt7;
extern Remote_HandleTypeDef hremote_vt03;

/*===| 遥控器通用构造(参考 Motor_Ctor) |===*/
//绑定串口句柄 + 协议 vtable + 错误检测参数
void Remote_Ctor(Remote_HandleTypeDef *hremote, UART_HandleTypeDef *huart, Remote_VTable *Remote_VTable, uint16_t err_tick_timeout, uint16_t err_count_maximum, err_handler err_handler);

//启动遥控器接收(对象由 ENT 调用 Remote_Ctor 构造)
//内部注册: UART空闲回调 + UART错误回调(per-handle)
void Remote_Init(Remote_HandleTypeDef *hremote);

//遥控器错误回调: 只负责检测(超时判定 + 计数), 不做任何恢复动作
void Remote_Err_Handler(Err_HandleTypeDef *herr);

//统一接收入口: 通过 vptr 分发到对应协议解包, 并清除错误状态
__STATIC_INLINE void Remote_Rx_Handle(Remote_HandleTypeDef *hremote, const uint8_t *DataBuff)
{
        //收到数据即清零错误状态(参考 Motor_DJI_Storage_Data)
        hremote->herr.tick   = 0;
        hremote->herr.count  = 0;
        hremote->herr.If_Err = 0;

        hremote->vptr->get_data(hremote, DataBuff);

        hremote->If_Data_New = 1;
}


__STATIC_INLINE float Remote_Get_Right_X(Remote_HandleTypeDef *hremote) { return hremote->Data.RC_Right_X; }
__STATIC_INLINE float Remote_Get_Right_Y(Remote_HandleTypeDef *hremote) { return hremote->Data.RC_Right_Y; }
__STATIC_INLINE float Remote_Get_Left_X(Remote_HandleTypeDef *hremote)  { return hremote->Data.RC_Left_X; }
__STATIC_INLINE float Remote_Get_Left_Y(Remote_HandleTypeDef *hremote)  { return hremote->Data.RC_Left_Y; }
__STATIC_INLINE float Remote_Get_Side(Remote_HandleTypeDef *hremote)    { return hremote->Data.RC_Side; }
__STATIC_INLINE float Remote_Get_Mouse_X(Remote_HandleTypeDef *hremote) { return hremote->Data.Mouse_Speed_X; }
__STATIC_INLINE float Remote_Get_Mouse_Y(Remote_HandleTypeDef *hremote) { return hremote->Data.Mouse_Speed_Y; }
__STATIC_INLINE float Remote_Get_Mouse_Z(Remote_HandleTypeDef *hremote) { return hremote->Data.Mouse_Speed_Z; }
__STATIC_INLINE uint8_t Remote_Get_S1(Remote_HandleTypeDef *hremote)    { return hremote->Data.S1; }
__STATIC_INLINE uint8_t Remote_Get_S2(Remote_HandleTypeDef *hremote)    { return hremote->Data.S2; }

#define Remote_Key_Shift_Single_Press(hremote)    ((hremote)->Data.Keyboard_Shift == 1 && (hremote)->Data_Last.Keyboard_Shift == 0)
#define Remote_Key_Ctrl_Single_Press(hremote)     ((hremote)->Data.Keyboard_Ctrl  == 1 && (hremote)->Data_Last.Keyboard_Ctrl  == 0)
#define Remote_Key_Q_Single_Press(hremote)        ((hremote)->Data.Keyboard_Q     == 1 && (hremote)->Data_Last.Keyboard_Q     == 0)
#define Remote_Key_E_Single_Press(hremote)        ((hremote)->Data.Keyboard_E     == 1 && (hremote)->Data_Last.Keyboard_E     == 0)
#define Remote_Key_R_Single_Press(hremote)        ((hremote)->Data.Keyboard_R     == 1 && (hremote)->Data_Last.Keyboard_R     == 0)
#define Remote_Key_F_Single_Press(hremote)        ((hremote)->Data.Keyboard_F     == 1 && (hremote)->Data_Last.Keyboard_F     == 0)
#define Remote_Key_G_Single_Press(hremote)        ((hremote)->Data.Keyboard_G     == 1 && (hremote)->Data_Last.Keyboard_G     == 0)
#define Remote_Key_Z_Single_Press(hremote)        ((hremote)->Data.Keyboard_Z     == 1 && (hremote)->Data_Last.Keyboard_Z     == 0)
#define Remote_Key_X_Single_Press(hremote)        ((hremote)->Data.Keyboard_X     == 1 && (hremote)->Data_Last.Keyboard_X     == 0)
#define Remote_Key_C_Single_Press(hremote)        ((hremote)->Data.Keyboard_C     == 1 && (hremote)->Data_Last.Keyboard_C     == 0)
#define Remote_Key_V_Single_Press(hremote)        ((hremote)->Data.Keyboard_V     == 1 && (hremote)->Data_Last.Keyboard_V     == 0)
#define Remote_Key_B_Single_Press(hremote)        ((hremote)->Data.Keyboard_B     == 1 && (hremote)->Data_Last.Keyboard_B     == 0)
#define Remote_Mouse_L_Single_Press(hremote)      ((hremote)->Data.Mouse_Press_L  == 1 && (hremote)->Data_Last.Mouse_Press_L  == 0)
#define Remote_Mouse_R_Single_Press(hremote)      ((hremote)->Data.Mouse_Press_R  == 1 && (hremote)->Data_Last.Mouse_Press_R  == 0)

#define Remote_Key_Shift_Single_Release(hremote)  ((hremote)->Data.Keyboard_Shift == 0 && (hremote)->Data_Last.Keyboard_Shift == 1)
#define Remote_Key_Ctrl_Single_Release(hremote)   ((hremote)->Data.Keyboard_Ctrl  == 0 && (hremote)->Data_Last.Keyboard_Ctrl  == 1)
#define Remote_Key_Q_Single_Release(hremote)      ((hremote)->Data.Keyboard_Q     == 0 && (hremote)->Data_Last.Keyboard_Q     == 1)
#define Remote_Key_E_Single_Release(hremote)      ((hremote)->Data.Keyboard_E     == 0 && (hremote)->Data_Last.Keyboard_E     == 1)
#define Remote_Key_R_Single_Release(hremote)      ((hremote)->Data.Keyboard_R     == 0 && (hremote)->Data_Last.Keyboard_R     == 1)
#define Remote_Key_F_Single_Release(hremote)      ((hremote)->Data.Keyboard_F     == 0 && (hremote)->Data_Last.Keyboard_F     == 1)
#define Remote_Key_G_Single_Release(hremote)      ((hremote)->Data.Keyboard_G     == 0 && (hremote)->Data_Last.Keyboard_G     == 1)
#define Remote_Key_Z_Single_Release(hremote)      ((hremote)->Data.Keyboard_Z     == 0 && (hremote)->Data_Last.Keyboard_Z     == 1)
#define Remote_Key_X_Single_Release(hremote)      ((hremote)->Data.Keyboard_X     == 0 && (hremote)->Data_Last.Keyboard_X     == 1)
#define Remote_Key_C_Single_Release(hremote)      ((hremote)->Data.Keyboard_C     == 0 && (hremote)->Data_Last.Keyboard_C     == 1)
#define Remote_Key_V_Single_Release(hremote)      ((hremote)->Data.Keyboard_V     == 0 && (hremote)->Data_Last.Keyboard_V     == 1)
#define Remote_Key_B_Single_Release(hremote)      ((hremote)->Data.Keyboard_B     == 0 && (hremote)->Data_Last.Keyboard_B     == 1)
#define Remote_Mouse_L_Single_Release(hremote)    ((hremote)->Data.Mouse_Press_L  == 0 && (hremote)->Data_Last.Mouse_Press_L  == 1)
#define Remote_Mouse_R_Single_Release(hremote)    ((hremote)->Data.Mouse_Press_R  == 0 && (hremote)->Data_Last.Mouse_Press_R  == 1)

#define Remote_Key_Shift_Press(hremote)           ((hremote)->Data.Keyboard_Shift == 1)
#define Remote_Key_Ctrl_Press(hremote)            ((hremote)->Data.Keyboard_Ctrl  == 1)
#define Remote_Key_Q_Press(hremote)               ((hremote)->Data.Keyboard_Q     == 1)
#define Remote_Key_E_Press(hremote)               ((hremote)->Data.Keyboard_E     == 1)
#define Remote_Key_R_Press(hremote)               ((hremote)->Data.Keyboard_R     == 1)
#define Remote_Key_F_Press(hremote)               ((hremote)->Data.Keyboard_F     == 1)
#define Remote_Key_G_Press(hremote)               ((hremote)->Data.Keyboard_G     == 1)
#define Remote_Key_Z_Press(hremote)               ((hremote)->Data.Keyboard_Z     == 1)
#define Remote_Key_X_Press(hremote)               ((hremote)->Data.Keyboard_X     == 1)
#define Remote_Key_C_Press(hremote)               ((hremote)->Data.Keyboard_C     == 1)
#define Remote_Key_V_Press(hremote)               ((hremote)->Data.Keyboard_V     == 1)
#define Remote_Key_B_Press(hremote)               ((hremote)->Data.Keyboard_B     == 1)
#define Remote_Mouse_L_Press(hremote)             ((hremote)->Data.Mouse_Press_L  == 1)
#define Remote_Mouse_R_Press(hremote)             ((hremote)->Data.Mouse_Press_R  == 1)

#endif //G4MINI_V3_DRV_REMOTE_H
