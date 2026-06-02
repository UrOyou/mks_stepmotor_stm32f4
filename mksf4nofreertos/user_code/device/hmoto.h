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
static int16_t abs(int16_t num);

/* 电机 A 控制 */
extern void MotorA_Forward(uint16_t speed);
extern void MotorA_Backward(uint16_t speed);

extern void MotorPower_Set(uint16_t duty);
extern void MotorPower_Start(void);
extern void MotorA_Stop(void);                     // 停止
extern void Motor_Brake(void);



/* 满占空比对应 ARR=7199，但代码里用 7200 做限幅，由 ARR 寄存器自动截断 */
#define MOTOR_PWM_MAX        7200

/* 斜坡参数：每 10ms 增加 36，则 0→7200 约 2 秒完成软启动 */
#define MOTOR_RAMP_STEP      36
#define MOTOR_RAMP_PERIOD    10   /* Motor_Process() 调用周期，单位 ms */

/* 方向切换死区：降到 0 后刹车等待 50ms，确保电机机械停转、反电动势消退 */
#define MOTOR_DIR_DEAD_MS    50

/* 引脚：PB5=TIM3_CH2(PWM)，PB6/7=方向/刹车（同组 GPIO，方便管理） */
#define MOTOR_PWM_TIM        &htim3
#define MOTOR_PWM_CH         TIM_CHANNEL_2

typedef enum {
    MOTOR_IDLE = 0,         /* 静止刹车 */
    MOTOR_RAMP,             /* 斜坡跟踪目标（加速或减速） */
    MOTOR_DIR_BRAKE         /* 方向切换中：已降到 0，正在死区等待 */
} MotorState_t;

typedef struct {
    int16_t      target;        /* 目标速度：-7200 ~ +7200，符号代表方向 */
    int16_t      current;       /* 当前实际输出占空比：0 ~ 7200（绝对值） */
    uint8_t      dir;           /* 当前实际方向：0=正转，1=反转 */
    uint8_t      target_dir;    /* 目标方向 */
    MotorState_t state;
    uint32_t     last_tick;     /* 上次斜坡更新时刻 */
    uint32_t     brake_tick;    /* 进入刹车死区的时刻 */
} MotorHandle_t;

/* 用户接口 */
extern void Motor_Set(int16_t speed);          /* 设置目标速度（带全部保护逻辑） */
extern void Motor_Process(void);               /* 须在主循环或 SysTick 中每 10ms 调用 */
extern int16_t Motor_GetCurrent(void);         /* 获取当前实际输出（带符号） */

#endif
