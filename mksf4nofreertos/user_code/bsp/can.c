#include "main.h"
#include "stm32f4xx_hal.h"
#include "can.h"

extern CAN_HandleTypeDef hcan1;

void can_filter_init(void)
{
    //CAN总线恢复功能
    // hcan1.Init.AutoBusOff = ENABLE;
    // hcan2.Init.AutoBusOff = ENABLE

    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    // can_filter_st.SlaveStartFilterBank = 14;
    // can_filter_st.FilterBank = 14;

}

void CAN1_RX0_IRQHandler(void)        
{
    HAL_CAN_IRQHandler(&hcan1);         // 由 HAL 统一分发并清标志
}

/* main.c 或用户文件 —— 回调函数中读取数据 */
CAN_RxHeaderTypeDef CanRxHeader;        // 对应标准库的 ID/DLC/IDE/RTR 等头信息
uint8_t CanRxData[8];                   // 对应标准库的 Data[8]

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CanRxHeader, CanRxData);
        CAN_RxDone = 1;
    }
}
