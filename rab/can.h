/**
 ****************************************************************************************************
 * @file        can.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2022-4-20
 * @brief       CAN 实验
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 阿波罗 F429开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 ****************************************************************************************************
 */

#ifndef __CAN_H
#define __CAN_H

// #include "./SYSTEM/sys/sys.h"
#include "sys.h"

/******************************************************************************************/
/* CAN 引脚 定义 */

#define CAN_RX_GPIO_PORT                GPIOA
#define CAN_RX_GPIO_PIN                 GPIO_PIN_11
#define CAN_RX_GPIO_CLK_ENABLE()        do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)         /* PA口时钟使能 */

#define CAN_TX_GPIO_PORT                GPIOA
#define CAN_TX_GPIO_PIN                 GPIO_PIN_12
#define CAN_TX_GPIO_CLK_ENABLE()        do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)         /* PA口时钟使能 */

/******************************************************************************************/

/* CAN1接收RX0中断使能 */
#define CAN1_RX0_INT_ENABLE   0         /* 0,不使能;1,使能 */

uint8_t can_init(uint32_t tsjw, uint32_t tbs2, uint32_t tbs1, uint16_t brp, uint32_t mode);  /* CAN初始化 */
uint8_t can_send_msg(uint32_t id, uint8_t *msg, uint8_t len);                                /* 发送数据 */
uint8_t can_receive_msg(uint32_t id, uint8_t *buf);                                          /* 接收数据 */

/************************************************/

//结构声明：
extern CAN_RxHeaderTypeDef uCanRxHeader;  // HAL接收帧头结构体
extern uint8_t uCanRxData[8];             // 分离的数据缓冲区 HAL接收数据缓冲
extern CAN_TxHeaderTypeDef uCanTxHeader;  // HAL发送帧头结构体
extern uint8_t uCanTxData[8];             // 分离的数据缓冲区 HAL发送数据缓冲

extern  volatile uint8_t CAN_RxDone = 0; //判断can接受是否成功
extern uint16_t CAN_ID;
typedef unsigned char bool_t;

// 原：CanRxMsg CanRxBuf;（标准库结构体）
// 原：CanTxMsg CanTxBuf;（标准库结构体）

extern CAN_HandleTypeDef hcan1;

uint8_t canCRC_ATM(uint8_t *buf, uint8_t len);//计算校验和
void CAN_INIT(void);
void CanTransfer(uint8_t *buf, uint8_t len);


/**************************************************/


#endif //__CAN_H

