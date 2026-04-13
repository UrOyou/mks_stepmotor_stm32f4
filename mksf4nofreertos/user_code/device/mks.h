/**
  ****************************(C) COPYRIGHT 2026 ${COMPANY}****************************
  * @file       mks.h
  * @brief      ${DESCRIPTION}
  *             mks步进电机驱动hal库f4可用版本
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     04-10-2026      youg            1. 初始版本
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


/*---------定义脉冲接口端口------------*/
#define En_PORT	GPIOA					 
#define En_PIN	GPIO_Pin_4		//En  PA4
#define Stp_PORT	GPIOA
#define Stp_PIN	GPIO_Pin_5		//Stp	PA5
#define Dir_PORT	GPIOA
#define Dir_PIN	GPIO_Pin_6		//Dir	PA6

extern uint8_t stpStatus;

extern void mksPulseInit(void);
extern void mksPulseRun(void);

/*-----------------------------------*/
typedef unsigned char boolean_t;

extern uint8_t txBuffer[8];      //待发送数据数组

//读取实时位置
const int32_t *readRealTimeLocation(uint8_t slaveAddr);
//等待从机响应
boolean_t waitingForACK(void);
extern uint8_t getCheckSum(uint8_t *buffer,uint8_t size);  
void NVIC_INIT(void);
void runFail(void);
void runOK(void);

//速度模式
extern void speedModeRun(uint8_t slaveAddr,uint8_t dir,uint16_t speed,uint8_t acc);
//位置控制 
//TODO待补完
extern void positionMode1Run(uint8_t slaveAddr,uint8_t dir,uint16_t speed,uint8_t acc,uint32_t pulses);
extern void positionMode2Run(uint8_t slaveAddr,uint16_t speed,uint8_t acc,int32_t relAxis);
extern void positionMode3Run(uint8_t slaveAddr,uint16_t speed,uint8_t acc,int32_t absAxis);

#endif //MKSMOT_H
