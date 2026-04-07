/**
  ****************************(C) COPYRIGHT 2026 ${COMPANY}****************************
  * @file       bsp_can.c
  * @brief      ${DESCRIPTION}
  *             can 初始化等相关操作的头文件
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     03-25-2026     youg             1. 初始版本
  *
  @verbatim
  ==============================================================================
    ${USAGE_NOTES}
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 ${COMPANY}****************************
  */
#include "bsp_can.h"

//声明
extern CAN_HandleTypeDef hcan1;
//变量定义
CAN_HandleTypeDef       g_canx_handle;                  /* CAN1句柄 */
uint8_t CAN_RxDone = 0; //判断can接受是否成功



//初始化函数
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


    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    // HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    // HAL_CAN_Start(&hcan2);
    // HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

}

//接收


//发送


//中断回调

