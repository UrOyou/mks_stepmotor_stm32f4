#include "hmoto.h"
#include "stm32f4xx_hal.h"


void hmoto_Init(void)
{

    /* ====== 1. 使能时钟 ====== */
    // RCC_APB2PeriphClockCmd(MOTOR_A_ENA_RCC | MOTOR_A_IN1_RCC | MOTOR_A_IN2_RCC |
    //                        MOTOR_B_ENB_RCC | MOTOR_B_IN3_RCC | MOTOR_B_IN4_RCC |
    //                        RCC_APB2Periph_AFIO, ENABLE);   // AFIO 用于引脚重映射（备用）
    // RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3, ENABLE);

    // /* ====== 2. 方向控制引脚（IN1~IN4）配置为推挽输出 ====== */
    // GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    // GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    // /* IN1、IN2（电机 A 方向） */
    // GPIO_InitStructure.GPIO_Pin = MOTOR_A_IN1_PIN | MOTOR_A_IN2_PIN;
    // GPIO_Init(MOTOR_A_IN1_GPIO, &GPIO_InitStructure);

    // /* IN3、IN4（电机 B 方向） */
    // GPIO_InitStructure.GPIO_Pin = MOTOR_B_IN3_PIN | MOTOR_B_IN4_PIN;
    // GPIO_Init(MOTOR_B_IN3_GPIO, &GPIO_InitStructure);

    /* 初始状态：全部拉低（停止） */
    HAL_GPIO_WritePin(MOTOR_A_IN1_GPIO,  MOTOR_A_IN1_PIN,  GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_A_IN2_GPIO,  MOTOR_A_IN2_PIN,  GPIO_PIN_RESET);



}

/* ================================================================
 *                         电机 A 控制函数
 * ================================================================ */


/**
 * @brief  设置电机 A 的 PWM 占空比
 * @param  speed: 0 ~ PWM_PERIOD(999)，0 = 停止，999 = 全速
 */
// void MotorA_SetSpeed(uint16_t speed)     //TODO
// {
//     if (speed > PWM_PERIOD)
//         speed = PWM_PERIOD;
//     TIM_SetCompare1(TIM2, speed);   // 更新 TIM2 通道 1 的比较值
// }

/**
 * @brief  电机 A 正转 伸长a高b低
 */
void MotorA_Forward(uint16_t speed)
{
    HAL_GPIO_WritePin(MOTOR_A_IN1_GPIO,  MOTOR_A_IN1_PIN,  GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_A_IN2_GPIO,  MOTOR_A_IN2_PIN,  GPIO_PIN_RESET);
    // MotorA_SetSpeed(speed);
}

/**
 * @brief  电机 A 反转 回收a低b高
 */
void MotorA_Backward(uint16_t speed)
{
    HAL_GPIO_WritePin(MOTOR_A_IN1_GPIO,  MOTOR_A_IN1_PIN,  GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_A_IN2_GPIO,  MOTOR_A_IN2_PIN,  GPIO_PIN_SET);
    // MotorA_SetSpeed(speed);
}

/**
 * @brief  电机 A 停止
 */
void MotorA_Stop(void)
{
    HAL_GPIO_WritePin(MOTOR_A_IN1_GPIO,  MOTOR_A_IN1_PIN,  GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_A_IN2_GPIO,  MOTOR_A_IN2_PIN,  GPIO_PIN_RESET);
    // MotorA_SetSpeed(0);
}
/* ================================================================
 *                         电机 B 控制函数
 * ================================================================ */
//TODO
