//
// Created by ye on 2026/8/22.
//

#ifndef G4MINI_V3_HAL_CAN_H
#define G4MINI_V3_HAL_CAN_H

#include "hal_bus.h"
#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include "can.h"
#include "utils.h"
#include "FreeRTOS.h"
#include "queue.h"

//掩码预设
#define CAN_FEEDBACK_ID_MASK_NONE 0xFFFF  //精确匹配( 默认)
#define CAN_FEEDBACK_ID_MASK_LOW8 0xFF00  //忽略ID低8 位

/*===| CAN总线资源分配(各设备使用的总线与ID, 使用前需包含can.h) |===*/

//两条总线(hcan1/hcan2 由 can.h 声明); STM32F407 双bxCAN共享28个过滤bank, CAN1用0~13, CAN2用14~27

typedef struct CAN_Node_HandleTypeDef CAN_Node_HandleTypeDef;

/*===| CAN节点: 继承 Bus_Node(第一成员) |===*/
struct CAN_Node_HandleTypeDef
{
        Bus_Node_HandleTypeDef Base;

        CAN_HandleTypeDef *hcan;
        uint32_t           Send_ID;
        uint32_t           Feedback_ID;
        uint32_t           Feedback_ID_Mask;
};

typedef struct {
        CAN_HandleTypeDef   *hcan;
        CAN_TxHeaderTypeDef CAN_TxMsg;
        uint8_t             txData[8]; //恒定8字节(HAL_CAN_AddTxMessage 无条件读8字节)
}CAN_Send_Frame_StructTypeDef;


/*===| 基类访问接口 |===*/
__STATIC_INLINE CAN_HandleTypeDef *CAN_Node_GetHcan(CAN_Node_HandleTypeDef *node)
{
        return node->hcan;
}

__STATIC_INLINE uint32_t CAN_Node_GetSendID(CAN_Node_HandleTypeDef *node)
{
        return node->Send_ID;
}

__STATIC_INLINE uint32_t CAN_Node_GetFeedbackID(CAN_Node_HandleTypeDef *node)
{
        return node->Feedback_ID;
}

void CAN_Node_Ctor(CAN_Node_HandleTypeDef *hcan_node, CAN_HandleTypeDef *hcan, uint32_t CAN_Send_ID, uint32_t CAN_Feedback_ID, uint32_t CAN_Feedback_ID_Mask, Bus_Node_Handler node_handler, uint16_t err_tick_timeout, uint16_t err_count_maximum, err_handler err_handler);

/*===| CAN节点分发框架 |===*/

//注册节点(重复注册则更新回调)
void CAN_Node_Register(CAN_Node_HandleTypeDef *hcan_node);

//注销节点
void CAN_Node_UnRegister(CAN_Node_HandleTypeDef *hcan_node);

/*===| CAN总线错误处理 |===*/
//总线错误/总线关闭: ABOM=ENABLE 由硬件自动恢复, 另在 HAL_CAN_ErrorCallback 里记录错误码供观察

//总线错误码快照[0]=CAN1, [1]=CAN2 (HAL_CAN_ErrorCallback 更新, 供调试观察)
extern volatile uint32_t CAN_Error_Code[2];

//总线离线判定阈值(监控周期10ms时为100ms)
#define CAN_OFFLINE_TICK 10

//CAN总线重启: 停止→去初始化→重新初始化→启动→重配过滤器/中断 (手动处理总线错误/总线关闭)
HAL_StatusTypeDef CAN_Restart(CAN_HandleTypeDef *hcan);

void CAN_Send_Data_STD(CAN_Node_HandleTypeDef *hcan_node, const uint8_t *TX_Data);

void CAN_Send_Data_EXD(CAN_Node_HandleTypeDef *hcan_node, uint8_t *TX_Data, uint8_t Length);

//底层发送: 不依赖节点(供一拖四组帧等临时发送用)
void CAN_Send_STD(CAN_HandleTypeDef *hcan, uint32_t Send_ID, const uint8_t *TX_Data);

//入队待发(内部走消息缓冲)
void CAN_Send_Data_Queue(CAN_Send_Frame_StructTypeDef *CAN_Send_Packet_Struct);

//CAN过滤器初始化: CAN1→bank0, CAN2→bank14(FMR.CAN2SB=14), 全通过→FIFO0
void CAN_Filter_Init(CAN_HandleTypeDef *hcan);

void CAN_Init();

//单条CAN总线初始化: 启动 + 配置过滤器/接收中断
void CAN_Bus_Init(CAN_HandleTypeDef *hcan);

#endif //G4MINI_V3_HAL_CAN_H
