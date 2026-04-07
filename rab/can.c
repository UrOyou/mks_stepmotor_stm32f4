/**
  ****************************(C) COPYRIGHT 2026 ${COMPANY}****************************
  * @file       bsp_can.cpp
  * @brief      ${DESCRIPTION}
  *             can 初始化等相关操作
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     03-11-2026     youg          1. 初始版本
  *
  @verbatim
  ==============================================================================
    ${USAGE_NOTES}
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 ${COMPANY}****************************
  */
#include "can.h"
#include "led.h"
#include "delay.h"
#include "string.h"

typedef unsigned char boolean_t;
CAN_HandleTypeDef       g_canx_handle;                  /* CAN1句柄 */
CAN_TxHeaderTypeDef     g_canx_txheade;                 /* 发送消息 */
CAN_RxHeaderTypeDef     g_canx_rxheade;                 /* 接收消息 */

// CAN_RxHeaderTypeDef uCanRxHeader;  // HAL接收帧头结构体
// uint8_t uCanRxData[8];             // 分离的数据缓冲区 HAL接收数据缓冲
// CAN_TxHeaderTypeDef uCanTxHeader;  // HAL发送帧头结构体
// uint8_t uCanTxData[8];             // 分离的数据缓冲区 HAL发送数据缓冲
// uint8_t CAN_RxDone = 0; //判断can接受是否成功

//示例：can_init(CAN_SJW_1TQ, CAN_BS2_6TQ, CAN_BS1_8TQ, 6, CAN_MODE_LOOPBACK);      /* CAN初始化, 环回模式, 波特率500Kbps */
/**
 * @brief       CAN初始化
 * @param       tsjw:重新同步跳跃时间单元.范围:CAN_SJW_1TQ~CAN_SJW_4TQ
 * @param       tbs2:时间段2的时间单元.   范围:CAN_BS2_1TQ~CAN_BS2_8TQ;
 * @param       tbs1:时间段1的时间单元.   范围:CAN_BS1_1TQ~CAN_BS1_16TQ
 * @param       brp :波特率分频器.范围:1~1024; tq=(brp)*tpclk1
 * @note        波特率=Fpclk1/((tbs1+tbs2+1)*brp); 
 *              CAN挂在APB1上面, 其输入时钟频率为 Fpclk1 = PCLK1 = 45Mhz
 *              如果设置CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_8tq,6,CAN_MODE_LOOPBACK);
 *              则波特率为:45M/((6+8+1)*6)=500Kbps
 * @param       mode:CAN_MODE_NORMAL,普通模式;CAN_MODE_LOOPBACK,回环模式;
 * @retval      返回值:0,初始化成功; 其他,初始化失败; 
 */
uint8_t can_init(uint32_t tsjw, uint32_t tbs2, uint32_t tbs1, uint16_t brp, uint32_t mode) 
{
    g_canx_handle.Instance = CAN1; 
    g_canx_handle.Init.Prescaler = brp;                 /* 分频系数(Fdiv)为brp+1 */
    g_canx_handle.Init.Mode = mode;                     /* 模式设置  */
    g_canx_handle.Init.SyncJumpWidth = tsjw;            /* 重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 CAN_SJW_1TQ~CAN_SJW_4TQ */
    g_canx_handle.Init.TimeSeg1 = tbs1;                 /* tbs1范围CAN_BS1_1TQ~CAN_BS1_16TQ */
    g_canx_handle.Init.TimeSeg2 = tbs2;                 /* tbs2范围CAN_BS2_1TQ~CAN_BS2_8TQ */
    g_canx_handle.Init.TimeTriggeredMode = DISABLE;     /* 非时间触发通信模式  */
    g_canx_handle.Init.AutoBusOff = DISABLE;            /* 软件自动离线管理 */
    g_canx_handle.Init.AutoWakeUp = DISABLE;            /* 睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位) */
    g_canx_handle.Init.AutoRetransmission = ENABLE;     /* 禁止报文自动传送  */
    g_canx_handle.Init.ReceiveFifoLocked = DISABLE;     /* 报文不锁定,新的覆盖旧的  */
    g_canx_handle.Init.TransmitFifoPriority = DISABLE;  /* 优先级由报文标识符决定  */

    if (HAL_CAN_Init(&g_canx_handle) != HAL_OK) 
    {
        return 1;   /* 初始化 */
    }

#if CAN1_RX0_INT_ENABLE
    __HAL_CAN_ENABLE_IT(&g_canx_handle, CAN_IT_RX_FIFO0_MSG_PENDING); /* FIFO0消息挂起中断允许. */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 1, 0);                        /* 抢占优先级1，子优先级2 */
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);                                /* 使能中断 */
#endif

    CAN_FilterTypeDef sFilterConfig;

    sFilterConfig.FilterIdHigh = 0X0000;                        /* 32位ID */
    sFilterConfig.FilterIdLow = 0X0000;
    sFilterConfig.FilterMaskIdHigh = 0X0000;                    /* 32位MASK */
    sFilterConfig.FilterMaskIdLow = 0X0000;  
    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;      /* 过滤器0关联到FIFO0 */
    sFilterConfig.FilterBank = 0;                               /* 过滤器0 */
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterActivation = CAN_FILTER_ENABLE;         /* 激活滤波器0 */
    sFilterConfig.SlaveStartFilterBank = 14;

     /* 过滤器配置 */
    if (HAL_CAN_ConfigFilter(&g_canx_handle, &sFilterConfig) != HAL_OK) 
    {
        return 2;                                               /* 滤波器初始化 */
    }

    /* 启动CAN外围设备 */
    if (HAL_CAN_Start(&g_canx_handle) != HAL_OK)
    {
        return 3;
    }

    return 0;
}


#if CAN1_RX0_INT_ENABLE                                     /* 使能RX0中断 */

/**
 * @brief       CAN中断服务函数
 * @param       无
 * @retval      无;
 */
void CAN1_RX0_IRQHandler(void)
{
    uint8_t rxbuf[8];
    uint32_t id;
    can_receive_msg(id, rxbuf);
    printf("id:%d\r\n", g_canx_rxheade.StdId);
    printf("ide:%d\r\n", g_canx_rxheade.IDE);
    printf("rtr:%d\r\n", g_canx_rxheade.RTR);
    printf("len:%d\r\n", g_canx_rxheade.DLC);

    printf("rxbuf[0]:%d\r\n", rxbuf[0]);
    printf("rxbuf[1]:%d\r\n", rxbuf[1]);
    printf("rxbuf[2]:%d\r\n", rxbuf[2]);
    printf("rxbuf[3]:%d\r\n", rxbuf[3]);
    printf("rxbuf[4]:%d\r\n", rxbuf[4]);
    printf("rxbuf[5]:%d\r\n", rxbuf[5]);
    printf("rxbuf[6]:%d\r\n", rxbuf[6]);
    printf("rxbuf[7]:%d\r\n", rxbuf[7]);
}

#endif

/**
 * @brief       CAN 发送一组数据
 * @note        发送格式固定为: 标准ID, 数据帧
 * @param       id   : 标准ID(11位)
 * @param       msg  : 数据指针,最大为8个字节.
 * @param       len  : 数据长度(最大为8)
 * @retval      发送状态 0, 成功; 1, 失败;
 */
uint8_t can_send_msg(uint32_t id, uint8_t *msg, uint8_t len)
{
    uint8_t waittime = 0;
    
    uint32_t TxMailbox = CAN_TX_MAILBOX0;
    g_canx_txheade.StdId = id;                 /* 标准标识符 */
    g_canx_txheade.ExtId = id;                 /* 扩展标识符(29位) */
    g_canx_txheade.IDE = CAN_ID_STD;           /* 使用标准帧 */
    g_canx_txheade.RTR = CAN_RTR_DATA;         /* 数据帧 */
    g_canx_txheade.DLC = len;

    if (HAL_CAN_AddTxMessage(&g_canx_handle, &g_canx_txheade, msg, &TxMailbox) != HAL_OK) /* 发送消息 */
    {
        return 1;
    }
    while(HAL_CAN_GetTxMailboxesFreeLevel(&g_canx_handle) != 3)                            /* 等待发送完成,所有邮箱为空 */
    {
        waittime++;
        
        if( waittime > 30)
        {
            return 1;
        }
        delay_ms(100);
    }
    return 0;
}

/**
 * @brief       CAN 接收数据查询
 * @note        接收数据格式固定为: 标准ID, 数据帧
 * @param       id      : 要查询的 标准ID(11位)
 * @param       buf     : 数据缓存区
 * @retval      接收结果
 *   @arg       0   , 无数据被接收到;
 *   @arg       其他, 接收的数据长度
 */
uint8_t can_receive_msg(uint32_t id, uint8_t *buf)
{
    if (HAL_CAN_GetRxFifoFillLevel(&g_canx_handle, CAN_RX_FIFO0) == 0)                                              /* 没有接收到数据 */
    {
        return 0;
    }

    if (HAL_CAN_GetRxMessage(&g_canx_handle, CAN_RX_FIFO0, &g_canx_rxheade, buf) != HAL_OK)                         /* 读取数据 */
    {
        return 0;
    }

    if (g_canx_rxheade.StdId != id || g_canx_rxheade.IDE != CAN_ID_STD || g_canx_rxheade.RTR != CAN_RTR_DATA)       /* 接收到的ID不对（不是标准帧、数据帧） */
    {
        return 0;
    }
    // CAN_RxDone = 1;
    return g_canx_rxheade.DLC;
}

/**
 * @brief   初始化CAN
 * @param   无
 * @retval  无
 */
void CAN_INIT(void)
{
    CAN_FilterTypeDef sFilterConfig = {0};
    sFilterConfig.FilterBank = 1;                           // 原：CAN_FilterNumber = 1
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;       // 原：CAN_FilterMode_IdMask
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;      // 原：CAN_FilterScale_32bit

    sFilterConfig.FilterIdHigh = 0x5678 << 5;               // 标准ID左移5位放入高16位
    sFilterConfig.FilterIdLow = 0x0000;                     // IDE=0(标准帧), RTR=0(数据帧)

    sFilterConfig.FilterMaskIdHigh = 0xFFE0;                // 完全匹配标准ID（11位有效）
    sFilterConfig.FilterMaskIdLow = 0x0000;

    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;  // 原：CAN_FIFO0
    sFilterConfig.FilterActivation = ENABLE;                // 原：ENABLE
    sFilterConfig.SlaveStartFilterBank = 14;                // F4特有：从CAN2过滤器组起始（CAN1用0-13）

    HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);           // 原：CAN_FilterInit(&CAN_FilterInitStructure);

    // 启动CAN（HAL新增步骤，标准库自动启动）
    HAL_CAN_Start(&hcan1);

    // 原：CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);  // 使能FIFO0消息挂起中断
}

/**
 * @brief   CAN1_RX0接收中断回调（HAL机制）
 * @param   hcan: CAN句柄指针
 * @retval  无
 */
// 原：void USB_LP_CAN1_RX0_IRQHandler(void)
// 注：F4的CAN1_RX0中断向量不同，且HAL使用统一中断入口，回调函数名固定
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1)  // 判断是CAN1
    {
        HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &uCanRxHeader, uCanRxData);
        // CAN_RxDone = 1;//ture
    }
}

// CAN发出标准帧
void CanTransfer(uint8_t *buf, uint8_t len)
{
    uCanTxHeader.StdId = CAN_ID;             // 原：CanTxBuf.StdId
    uCanTxHeader.ExtId = 0x00;               // 扩展ID（标准帧不用）
    uCanTxHeader.IDE = CAN_ID_STD;           // 原：CAN_ID_STD
    uCanTxHeader.RTR = CAN_RTR_DATA;         // 原：CAN_RTR_DATA
    uCanTxHeader.DLC = len;                  // 原：CanTxBuf.DLC
    uCanTxHeader.TransmitGlobalTime = DISABLE;

    memcpy(uCanTxData, buf, len - 1);
    uCanTxData[len - 1] = canCRC_ATM(buf, len - 1);

    uint32_t TxMailbox;
    HAL_CAN_AddTxMessage(&hcan1, &uCanTxHeader, uCanTxData, &TxMailbox);
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
