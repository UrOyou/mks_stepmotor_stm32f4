/**
  ****************************(C) COPYRIGHT 2026 ${COMPANY}****************************
  * @file       mksmotor.h
  * @brief      ${DESCRIPTION}
  *             mks步进电机驱动hal库f4可用版本
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     03-18-2026     youg          1. 初始版本
  *
  @verbatim
  ==============================================================================
    ${USAGE_NOTES}
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 ${COMPANY}****************************
  */
#ifndef MKSMOT_H
#define MKSMOT_H
#include "stm32f4xx_hal.h"


uint8_t Motor_ID;
uint8_t txBuffer[8];      //电机待发送数据数组

uint8_t waitingForACK(uint32_t delayTime);//等待从机响应
extern uint8_t getCheckSum(uint8_t *buffer,uint8_t size);  
void NVIC_INIT(void);
void runFail(void);
void runOK(void);

//速度模式 疑似用不上
extern void speedModeRun(uint8_t slaveAddr,uint8_t dir,uint16_t speed,uint8_t acc);
//位置控制
extern void positionMode1Run(uint8_t slaveAddr,uint8_t dir,uint16_t speed,uint8_t acc,uint32_t pulses);
extern void positionMode2Run(uint8_t slaveAddr,uint16_t speed,uint8_t acc,int32_t relAxis);


#endif //MKSMOT_H
