/**
  ****************************(C) COPYRIGHT 2026 ${COMPANY}****************************
  * @file       can.h
  * @brief      ${DESCRIPTION}
  *             can 初始化等相关操作的头文件
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     04-10-2026     youg             1. 初始版本
  *
  @verbatim
  ==============================================================================
    ${USAGE_NOTES}
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 ${COMPANY}****************************
  */
#ifndef _CAN
#define _CAN
#include "stm32f4xx_hal.h"
extern CAN_RxHeaderTypeDef CanRxHeader;        // ID/DLC/IDE/RTR 等头信息
extern uint8_t CanRxData[8];                   // Data[8]

extern void can_filter_init(void);
extern uint8_t CAN_RxDone;

#endif //_CAN
