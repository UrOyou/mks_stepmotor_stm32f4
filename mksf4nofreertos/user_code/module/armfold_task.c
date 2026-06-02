//包含整机运动的中断函数（测试阶段暂时由按键触发）
#include "armfold_task.h"
#include "stm32f4xx_hal.h"


void arm_Extension(void){


}

void arm_Flexion(void){

    
}
//================== 初始化 ========================================


TIM_HandleTypeDef htim6;
MotorCtrl_t g_motors[MOTOR_COUNT];
volatile uint8_t g_keyDebounce = 0;
volatile uint8_t g_systemRunning = 0;

/* 电机驱动函数（用户已实现，此处仅声明） */
extern void MotorA_Start(void);
extern void MotorA_Stop(void);
extern void MotorB_Start(void);
extern void MotorB_Stop(void);
extern void MotorC_Start(void);
extern void MotorC_Stop(void);

/* 函数原型 */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM6_Init(void);
static void KEY_EXTI_Init(void);

// while (1)
//     {
//         /* 运行指示灯：PG13（F429  Discovery 板载 LED） */
//         HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, 
//                           g_systemRunning ? GPIO_PIN_SET : GPIO_PIN_RESET);
//         HAL_Delay(100);      // 主循环 leisurely，不干扰实时控制
//     }



/* ========== TIM6 初始化：1ms 中断时基 ========== */
static void MX_TIM6_Init(void)
{
    __HAL_RCC_TIM6_CLK_ENABLE();

    htim6.Instance = TIM6;
    /* 
     * TIM6CLK = APB1 Timer clock = 84MHz (42MHz × 2)
     * 84MHz / (8399+1) = 10kHz
     * 10kHz / (9+1) = 1kHz → 1ms 周期
     */
    htim6.Init.Prescaler = 8399;
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = 9;
    htim6.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&htim6);

    /* NVIC：抢占优先级3，子优先级0 */
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

/* ========== 按键外部中断初始化（PA0） ========== */
static void KEY_EXTI_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;      // 上升沿触发
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);         // 按键优先级高于定时器
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}


void Error_Handler(void)
{
    while(1);
}


//多个电机控制中断回调函数
void armfold_init(void)
{
    
    MX_TIM6_Init();          // 初始化 TIM6（1ms 中断，默认不启动）
    KEY_EXTI_Init();         // 初始化按键外部中断

    /* 填充电机控制表：时长分别为 3s、5s、8s */
    g_motors[0] = (MotorCtrl_t){0, 3000, MOTOR_STOP, MotorA_Start, MotorA_Stop};
    g_motors[1] = (MotorCtrl_t){0, 5000, MOTOR_STOP, MotorB_Start, MotorB_Stop};
    g_motors[2] = (MotorCtrl_t){0, 8000, MOTOR_STOP, MotorC_Start, MotorC_Stop};

    /* 上电初始状态：全部停止 */
    for (int i = 0; i < MOTOR_COUNT; i++) {
        g_motors[i].Stop();
    }

}



//======================================================================================

/* 外部全局变量引用 */
extern MotorCtrl_t g_motors[MOTOR_COUNT];
extern volatile uint8_t g_keyDebounce;
extern volatile uint8_t g_systemRunning;
extern TIM_HandleTypeDef htim6;

/* ============================================================
 *  一、中断服务函数（ISR）—— 硬件向量表入口
 * ============================================================ */

/**
  * @brief  外部中断 0 服务函数（PA0 按键）
  * @note   固定函数名，调用 HAL 库统一入口
  */
void EXTI0_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

/**
  * @brief  TIM6 + DAC 共用中断服务函数
  * @note   STM32F429 中 TIM6 与 DAC 下溢共用此向量
  */
void TIM6_DAC_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim6);
}

/* ============================================================
 *  二、HAL 弱回调函数（中断业务逻辑）
 * ============================================================ */

/**
  * @brief  按键中断回调（EXTI 触发后由 HAL 调用）
  * @param  GPIO_Pin: 触发中断的引脚
  * @note   消抖 + 启动所有电机 + 开启 TIM6 1ms 心跳
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_0)
    {
        /* 软件消抖：运行期间屏蔽重复触发 */
        if (g_keyDebounce == 0)
        {
            g_keyDebounce = 1;
            uint32_t now = HAL_GetTick();

            /* 同时启动所有电机，记录基准时间 */
            for (int i = 0; i < MOTOR_COUNT; i++)
            {
                g_motors[i].startTime_ms = now;
                g_motors[i].state = MOTOR_RUN;
                g_motors[i].Start();            // 调用电机启动函数
            }

            g_systemRunning = 1;

            /* 启动 TIM6 定时器中断（1ms 周期开始计数） */
            HAL_TIM_Base_Start_IT(&htim6);
        }
    }
}

/**
  * @brief  定时器周期中断回调（TIM6 每 1ms 进入一次）
  * @param  htim: 触发中断的定时器句柄
  * @note   检查各电机是否到达设定时长，全部完成后关闭中断
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        uint32_t now = HAL_GetTick();
        uint8_t allStopped = 1;

        for (int i = 0; i < MOTOR_COUNT; i++)
        {
            if (g_motors[i].state == MOTOR_RUN)
            {
                /* 判断是否到达设定时长 */
                if ((now - g_motors[i].startTime_ms) >= g_motors[i].duration_ms)
                {
                    g_motors[i].Stop();         // 关闭该电机
                    g_motors[i].state = MOTOR_STOP;
                }
                else
                {
                    allStopped = 0;             // 仍有电机在转
                }
            }
        }

        /* 全部停止后：关闭定时器中断，重置系统状态，允许再次按键 */
        if (allStopped)
        {
            HAL_TIM_Base_Stop_IT(&htim6);       // 停止 TIM6 计数 + 关闭中断
            g_systemRunning = 0;
            g_keyDebounce = 0;                  // 解除按键屏蔽
        }
    }
}
