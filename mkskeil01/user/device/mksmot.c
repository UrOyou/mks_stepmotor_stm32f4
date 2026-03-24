/**
  ****************************(C) COPYRIGHT 2026 ${COMPANY}****************************
  * @file       mksmotor.cpp
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

#include "mksmot.h"
#include "delay.h"
#include "can.h"
#include "led.h"

uint16_t  CAN_ID;

typedef unsigned char boolean_t;


/**
 * @brief   发送位置模式1运行指令（MKS电机协议）
 * @param   slaveAddr   从机地址（CAN帧ID）
 * @param   dir         运行方向（0=正转，1=反转）
 * @param   speed       运行速度（RPM，范围0-3000）
 * @param   acc         加速度（单位根据电机固件）
 * @param   pulses      目标脉冲数（24位有效，最大0xFFFFFF）
 * @retval  无
 * @note    数据格式：FD + [DIR+SPEED_H] + [SPEED_L] + [ACC] + [PULSES_23:16] + [PULSES_15:8] + [PULSES_7:0] + [CRC]
//  */
void positionMode1Run(uint8_t slaveAddr, uint8_t dir, uint16_t speed, uint8_t acc, uint32_t pulses)
{
  CAN_ID = slaveAddr;                 // 设置全局CAN ID，供CRC计算和发送使用
  txBuffer[0] = 0xFD;                 // 功能码：0xFD = 位置模式1指令（MKS协议定义）
  txBuffer[1] = (dir << 7) | ((speed >> 8) & 0x0F);
  // 位域：[7]方向位（1=反转，0=正转），[6:4]保留，[3:0]速度高4位
  txBuffer[2] = speed & 0xFF;         // 速度低8位，与txBuffer[1]的低4位组成12位速度值（0-4095，实际用0-3000 RPM）
  txBuffer[3] = acc;                  // 加速度参数，数值越大加速越快（电机固件解析）
  txBuffer[4] = (pulses >> 16) & 0xFF;    // 脉冲数第3字节（bit23:16）
  txBuffer[5] = (pulses >> 8) & 0xFF;     // 脉冲数第2字节（bit15:8）
  txBuffer[6] = pulses & 0xFF;            // 脉冲数第1字节（bit7:0）
  CanTransfer(txBuffer, 8);           // 发送8字节帧（实际数据7字节+1字节CRC）
}

/**
 * @brief   配置NVIC控制器（CAN1 RX0中断）
 * @param   无
 * @retval  无
 * @note    抢占优先级2位，响应优先级2位（共4位优先级）
 */
void NVIC_INIT(void)
{
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_2);  // 设置优先级分组：2位抢占，2位子优先级
  HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);  // 设置中断优先级：抢占=0，子优先级=0
  // F4中CAN1_RX0_IRQn对应原USB_LP_CAN1_RX0_IRQn
  HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);          // 使能中断
}

/**
 * @brief   等待下位机应答，设置超时时间为3000ms
 * @param   delayTime   等待时间(ms)，为0时无限等待
 * @retval  位置模式1控制开始    1
 *          位置模式1控制完成    2
 *          位置模式1控制失败    0
 *          超时无应答           0
 * @note    数据格式：FD + [DIR+SPEED_H] + [SPEED_L] + [ACC] + [PULSES_23:16] + [PULSES_15:8] + [PULSES_7:0] + [CRC]
 */
uint8_t waitingForACK(uint32_t delayTime)
{
  boolean_t retVal = 0; //返回值
  unsigned long sTime;  		//计时起始时刻
  unsigned long time;  			//当前时刻
  uint8_t rxByte;      
	
  sTime = HAL_GetTick();    //获取当前时刻
  while(1)
  {
		if(CAN_RxDone == 1)  //CAN接收到数据
		{
			CAN_RxDone = 0;
			rxByte = CanRxHeader.DLC;
			CAN_ID = CanRxHeader.StdId;
			if(CanRxData[rxByte-1] == canCRC_ATM(CanRxData,rxByte-1))
			{
				retVal = CanRxData[1];   //校验正确
				break;
			}				
		}

    time = HAL_GetTick();
    if((delayTime != 0) && ((time - sTime) > delayTime))   //判断是否超时
    {
      retVal = 0;
      break;                    //超时，退出while(1)
    }
  }
  return(retVal);
}

//LED指示灯相关
//运行失败
void runFail(void)
{
  while(1)                //快速闪红灯，提示运行失败
  {
    LED0(0);
    delay_ms(200);
    LED0(1);
    delay_ms(200);
  }
}

//运行成功
void runOK(void)
{
  while(1)                //慢速闪绿灯，提示运行成功
  {
    LED1(0);
    delay_ms(1000);
    LED1(1);
    delay_ms(1000);
  }
}