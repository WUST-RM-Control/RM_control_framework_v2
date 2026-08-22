//
// Created by ye on 2026/8/22.
//

#include "hal_can.h"

#include "fdcan.h"


uint16_t CAN1_Err_Ticker;
uint16_t CAN2_Err_Ticker;
uint16_t CAN3_Err_Ticker;


uint8_t CAN_RX_Data1[8];

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
        FDCAN_RxHeaderTypeDef RxHeader;
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, CAN_RX_Data1);

        if (hfdcan == &hfdcan1) CAN1_Err_Ticker = 0;
        if (hfdcan == &hfdcan2) CAN2_Err_Ticker = 0;
        if (hfdcan == &hfdcan3) CAN3_Err_Ticker = 0;


        uint16_t CAN_RX_ID = RxHeader.Identifier;

        switch ((uint32_t)hfdcan->Instance)
        {
        case (uint32_t)FDCAN1:

                break;
        case (uint32_t)FDCAN2:
                break;
        case (uint32_t)FDCAN3:
                break;
        }
}

//CAN-发送标准帧(FDCAN, CANID，发送数据数组（八字节））
void CAN_Send_Data_STD(FDCAN_HandleTypeDef *hfdcan, const uint16_t ID, const uint8_t *TX_Data)
{
        FDCAN_TxHeaderTypeDef TxHeader;
        TxHeader.Identifier          = ID;
        TxHeader.IdType              = FDCAN_STANDARD_ID;
        TxHeader.TxFrameType         = FDCAN_DATA_FRAME;
        TxHeader.DataLength          = FDCAN_DLC_BYTES_8;
        TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        TxHeader.BitRateSwitch       = FDCAN_BRS_OFF;
        TxHeader.FDFormat            = FDCAN_CLASSIC_CAN;
        TxHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
        TxHeader.MessageMarker       = 0;

        uint32_t OverTick = 0;
        while (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) == 0)
        {
                OverTick++;
                if (OverTick > 1000) break;
        }
        HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, TX_Data);
}

//CAN-发送拓展帧(FDCAN, CANID，发送数据数组，发送数据长度）
void CAN_Send_Data_EXD(FDCAN_HandleTypeDef *hfdcan, uint32_t ID, uint8_t *TX_Data, uint8_t Length)
{
        FDCAN_TxHeaderTypeDef TxHeader;
        TxHeader.Identifier          = ID;
        TxHeader.IdType              = FDCAN_EXTENDED_ID;
        TxHeader.TxFrameType         = FDCAN_DATA_FRAME;
        TxHeader.DataLength          = Length;
        TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        TxHeader.BitRateSwitch       = FDCAN_BRS_OFF;
        TxHeader.FDFormat            = FDCAN_CLASSIC_CAN;
        TxHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
        TxHeader.MessageMarker       = 0;

        uint32_t OverTick = 0;
        while (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) == 0)
        {
                OverTick++;
                if (OverTick > 1000) break;
        }
        HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, TX_Data);


}

//CAN初始化
void CAN_Init(void)
{
        HAL_FDCAN_Start(&hfdcan1);
        HAL_FDCAN_Start(&hfdcan2);
        HAL_FDCAN_Start(&hfdcan3);
        CAN_Filter_Init(&hfdcan1);
        CAN_Filter_Init(&hfdcan2);
        CAN_Filter_Init(&hfdcan3);
}

//CAN过滤器初始化
void CAN_Filter_Init(FDCAN_HandleTypeDef *hfdcan)
{
        FDCAN_FilterTypeDef sFilterConfig;

        sFilterConfig.IdType       = FDCAN_STANDARD_ID;
        sFilterConfig.FilterIndex  = 0;
        sFilterConfig.FilterType   = FDCAN_FILTER_RANGE;
        sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
        sFilterConfig.FilterID1    = 0x00;
        sFilterConfig.FilterID2    = 0x7FF;

        HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig);
        HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
        HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}
