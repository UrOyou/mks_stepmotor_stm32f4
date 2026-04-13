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
#include "core_cm4.h"
#include "stm32f4xx_hal.h"
#include "delay.h"
#include "led.h"
#include "bsp_can.h"


typedef unsigned char boolean_t;
uint16_t  CAN_ID;
extern CAN_HandleTypeDef hcan1;
CAN_TxHeaderTypeDef motor_can_tx_msg;
CAN_RxHeaderTypeDef motor_can_rx_msg;
uint8_t txBuffer[8];
uint8_t rxBuffer[8];
// boolean_t CAN_RxDone = 0;

// int32_t realTimeLocation;		//电机实时位置
// /*
// 功能：读取实时位置信息
// 输入：slaveAddr 从机地址
// 输出：无
//  */
// void readRealTimeLocation(uint8_t slaveAddr)
// {
//   uint32_t send_mail_box;
//   motor_can_tx_msg.StdId = slaveAddr;
//   motor_can_tx_msg.IDE = CAN_ID_STD;
//   motor_can_tx_msg.RTR = CAN_RTR_DATA;
//   motor_can_tx_msg.DLC = 0x02;
// 	// CAN_ID = slaveAddr;				//ID
//   txBuffer[0] = 0x31;       //功能码
//   // CanTransfer(txBuffer,2);
  
//   HAL_CAN_AddTxMessage(&hcan1, &motor_can_tx_msg, txBuffer, &send_mail_box);
//   int32_t realTimeLocation = (int32_t)(rxBuffer[3]<<24 | rxBuffer[4]<<16 | rxBuffer[5]<<8 | rxBuffer[6]<<0);
//   // return realTimeLocation;
// }
//返回位置参数指针
//return 电机参数结构体...

const int32_t *readRealTimeLocation(uint8_t slaveAddr)
{
    static int32_t realTimeLocation;  // 静态变量，函数返回后仍然有效
    uint32_t send_mail_box;
    // 参数检查  无效地址 
    if(slaveAddr == 0) {return NULL;  }
    // 配置 CAN 消息
    motor_can_tx_msg.StdId = slaveAddr;
    motor_can_tx_msg.IDE = CAN_ID_STD;
    motor_can_tx_msg.RTR = CAN_RTR_DATA;
    motor_can_tx_msg.DLC = 0x02;
    txBuffer[0] = 0x31;  // 功能码：读取位置
    
    // 发送请求
    if(HAL_CAN_AddTxMessage(&hcan1, &motor_can_tx_msg, txBuffer, &send_mail_box) != HAL_OK) {
        return NULL;  // 发送失败
    }
    
    // 等待接收完成（需要超时机制）
    uint32_t timeout = HAL_GetTick() + 100;  // 100ms 超时
    while(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &motor_can_rx_msg, rxBuffer) != HAL_OK) {
        if(HAL_GetTick() > timeout) {
            return NULL;  // 接收超时
        }
    }
    
    // 解析数据（修正字节拼接，避免符号扩展）
    realTimeLocation = (int32_t)(
        ((uint32_t)rxBuffer[3] << 24) | 
        ((uint32_t)rxBuffer[4] << 16) | 
        ((uint32_t)rxBuffer[5] << 8)  | 
        ((uint32_t)rxBuffer[6])
    );
    return &realTimeLocation;  // 返回静态变量的地址
}




void readrealtimelocationrx(){
}

/*
功能：串口发送速度模式运行指令
输入：slaveAddr 从机地址
      dir       运行方向
      speed     运行速度
      acc       加速度
*/
void speedModeRun(uint8_t slaveAddr,uint8_t dir,uint16_t speed,uint8_t acc)
{
  uint32_t send_mail_box;
  motor_can_tx_msg.StdId = slaveAddr;
  motor_can_tx_msg.IDE = CAN_ID_STD;
  motor_can_tx_msg.RTR = CAN_RTR_DATA;
  motor_can_tx_msg.DLC = 0x05;
	//CAN_ID = slaveAddr;				//ID

  txBuffer[0] = 0xF6;       //功能码
  txBuffer[1] = (dir<<7) | ((speed>>8)&0x0F); //方向和速度高4位
  txBuffer[2] = speed&0x00FF;   //速度低8位
  txBuffer[3] = acc;            //加速度
	// CanTransfer(txBuffer,5);
  HAL_CAN_AddTxMessage(&hcan1, &motor_can_tx_msg, txBuffer, &send_mail_box);

}

/**
 * @brief   发送位置模式1运行指令（MKS电机协议）
 * @param   slaveAddr   从机地址（CAN帧ID）
 * @param   dir         运行方向（0=正转，1=反转）
 * @param   speed       运行速度（RPM，范围0-3000）
 * @param   acc         加速度（单位根据电机固件）
 * @param   pulses      目标脉冲数（24位有效，最大0xFFFFFF）
 * @retval  无
 * @note    数据格式：FD + [DIR+SPEED_H] + [SPEED_L] + [ACC] + [PULSES_23:16] + [PULSES_15:8] + [PULSES_7:0] + [CRC]
 */
void positionMode1Run(uint8_t slaveAddr, uint8_t dir, uint16_t speed, uint8_t acc, uint32_t pulses)
{
  
  uint32_t send_mail_box;
  motor_can_tx_msg.StdId = slaveAddr;
  motor_can_tx_msg.IDE = CAN_ID_STD;
  motor_can_tx_msg.RTR = CAN_RTR_DATA;
  motor_can_tx_msg.DLC = 0x08;

  // CAN_ID = slaveAddr;                 // 设置全局CAN ID，供CRC计算和发送使用
  txBuffer[0] = 0xFD;                 // 功能码：0xFD = 位置模式1指令（MKS协议定义）
  txBuffer[1] = (dir << 7) | ((speed >> 8) & 0x0F);
  // 位域：[7]方向位（1=反转，0=正转），[6:4]保留，[3:0]速度高4位
  txBuffer[2] = speed & 0xFF;         // 速度低8位，与txBuffer[1]的低4位组成12位速度值（0-4095，实际用0-3000 RPM）
  txBuffer[3] = acc;                  // 加速度参数，数值越大加速越快（电机固件解析）
  txBuffer[4] = (pulses >> 16) & 0xFF;    // 脉冲数第3字节（bit23:16）
  txBuffer[5] = (pulses >> 8) & 0xFF;     // 脉冲数第2字节（bit15:8）
  txBuffer[6] = pulses & 0xFF;            // 脉冲数第1字节（bit7:0）
  // CanTransfer(txBuffer, 8);           // 发送8字节帧（实际数据7字节+1字节CRC）
  HAL_CAN_AddTxMessage(&hcan1, &motor_can_tx_msg, txBuffer, &send_mail_box);

}

  /*
  功能：串口发送位置模式2运行指令
  输入：slaveAddr 从机地址
        speed     运行速度
        acc       加速度
        relAxis   相对坐标
  */
  // void positionMode2Run(uint8_t slaveAddr,uint16_t speed,uint8_t acc,int32_t relAxis)
  // {
  //   CAN_ID = slaveAddr;				//ID
  //   txBuffer[0] = 0xF4;       //功能码
  //   txBuffer[1] = (speed>>8)&0x00FF; //速度高8位
  //   txBuffer[2] = speed&0x00FF;     //速度低8位
  //   txBuffer[3] = acc;            //加速度
  //   txBuffer[4] = (relAxis >> 16)&0xFF;  //相对坐标 bit23 - bit16
  //   txBuffer[5] = (relAxis >> 8)&0xFF;   //相对坐标 bit15 - bit8
  //   txBuffer[6] = (relAxis >> 0)&0xFF;   //相对坐标 bit7 - bit0
    
  //   // CanTransfer(txBuffer,8);
  // }

  /*
  功能：串口发送位置模式3运行指令
  输入：slaveAddr 从机地址
        speed     运行速度
        acc       加速度
        absAxis   绝对坐标
  // */
  // void positionMode3Run(uint8_t slaveAddr,uint16_t speed,uint8_t acc,int32_t absAxis)
  // {
  //   CAN_ID = slaveAddr;				//ID
  //   txBuffer[0] = 0xF5;       //功能码
  //   txBuffer[1] = (speed>>8)&0x00FF; //速度高8位
  //   txBuffer[2] = speed&0x00FF;     //速度低8位
  //   txBuffer[3] = acc;            //加速度
  //   txBuffer[4] = (absAxis >> 16)&0xFF;  //绝对坐标 bit23 - bit16
  //   txBuffer[5] = (absAxis >> 8)&0xFF;   //绝对坐标 bit15 - bit8
  //   txBuffer[6] = (absAxis >> 0)&0xFF;   //绝对坐标 bit7 - bit0
    
  //   // CanTransfer(txBuffer,8);
  // }

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


// 计算校验和（算法不变，仅类型调整）
uint8_t canCRC_ATM(uint8_t *buf, uint8_t len)
{
    uint32_t sum = 0;
    for (uint8_t i = 0; i < len; i++)
    {
        sum += buf[i];
    }
    sum += CAN_ID;
    return (uint8_t)(sum & 0xFF);
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
boolean_t waitingForACK(void)
{
  boolean_t retVal = 0; //返回值
  unsigned long sTime;  		//计时起始时刻
  unsigned long time;  			//当前时刻
  uint32_t delayTime = 3000;
  uint8_t rxByte;      
	
  sTime = HAL_GetTick();    //获取当前时刻
  while(1)
  {
		if(CAN_RxDone == 1)  //CAN接收到数据
		{
			CAN_RxDone = 0;
			rxByte = motor_can_rx_msg.DLC;
			CAN_ID = motor_can_rx_msg.StdId;
			if(rxBuffer[rxByte-1] == canCRC_ATM(rxBuffer,rxByte-1))
			{
				retVal = rxBuffer[1];   //校验正确
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

// //LED指示灯相关
// //运行失败
// void runFail(void)
// {
//   while(1)                //快速闪红灯，提示运行失败
//   {
//     LED0(0);
//     delay_ms(200);
//     LED0(1);
//     delay_ms(200);
//   }
// }

// //运行成功
// void runOK(void)
// {
//   while(1)                //慢速闪绿灯，提示运行成功
//   {
//     LED1(0);
//     delay_ms(1000);
//     LED1(1);
//     delay_ms(1000);
//   }
// }
