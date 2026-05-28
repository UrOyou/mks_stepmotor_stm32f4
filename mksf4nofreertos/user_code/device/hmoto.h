#ifndef HMOTO_H
#define HMOTO_H
#include "stm32f4xx_hal.h"

/* ---------- 引脚宏定义（可根据实际接线修改） ---------- */

/* 电机 A（ IN1 / IN2） */

#define MOTOR_A_IN1_GPIO       GPIOB
#define MOTOR_A_IN1_PIN        GPIO_PIN_6

#define MOTOR_A_IN2_GPIO       GPIOB
#define MOTOR_A_IN2_PIN        GPIO_PIN_7

/* PWM 定时器定义 */
#define MOTOR_A_TIM           TIM2          // ENA 使用 TIM2_CH1 (PA0)
#define MOTOR_B_TIM           TIM3          // ENB 使用 TIM3_CH1 (PA6)
#define MOTOR_A_PWM_CHANNEL   TIM_OCMode_PWM1
#define MOTOR_B_PWM_CHANNEL   TIM_OCMode_PWM1

#define PWM_PERIOD            999           // PWM 周期 = (PSC+1) × (PERIOD+1) / 72M
                                            // PSC=71, PERIOD=999 → PWM频率 = 1kHz

/* ---------- 公开接口 ---------- */

extern void hmoto_Init(void);                     // 初始化所有 GPIO 和定时器 PWM

/* 电机 A 控制 */
extern void MotorA_Forward(uint16_t speed);
extern void MotorA_Backward(uint16_t speed);

extern void MotorA_Stop(void);                     // 停止

#endif
