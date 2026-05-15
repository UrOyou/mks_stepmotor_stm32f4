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
#include "stm32f4xx_hal_gpio.h"

//TODO
//重新cube配置 gpio 接口
//修改函数标准库 转换。

/*---------定义脉冲接口端口------------*/
#define En_PORT     GPIOB
#define En_PIN      GPIO_PIN_6     // En  → PB6（未连接任何设备，完全独立）
#define Stp_PORT    GPIOB
#define Stp_PIN     GPIO_PIN_7     // Stp → PB7（DCMI_VSYNC，不插摄像头时完全独立）
#define Dir_PORT    GPIOB
#define Dir_PIN     GPIO_PIN_8     // Dir → PB8（DCMI_D6，不插摄像头时完全独立）

extern uint8_t stpStatus;

//电机脉冲初始化
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
extern void NVIC_INIT(void);
extern void runFail(void);
extern void runOK(void);
//电机使能
extern void setMotorEnable(uint8_t slaveAddr, uint8_t enable);


//速度模式
extern void speedModeRun(uint8_t slaveAddr,uint8_t dir,uint16_t speed,uint8_t acc);
//位置控制 
//TODO待补完
extern void positionMode1Run(uint8_t slaveAddr,uint8_t dir,uint16_t speed,uint8_t acc,uint32_t pulses);
extern void positionMode2Run(uint8_t slaveAddr,uint16_t speed,uint8_t acc,int32_t relAxis);
extern void positionMode3Run(uint8_t slaveAddr,uint16_t speed,uint8_t acc,int32_t absAxis);

//运动模式测试
extern void speedtimemode(void);

//多机协动测试
extern void multimotor(void);

#endif //MKSMOT_H
