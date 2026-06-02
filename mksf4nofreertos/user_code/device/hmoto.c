#include "hmoto.h"
#include "stm32f4xx_hal.h"

extern TIM_HandleTypeDef htim3;
static MotorHandle_t g_motor = {0};


static int16_t abs(int16_t num){
    if (num < 0){
        return -num; 
    }else {
        return num;
    }
}

void hmoto_Init(void)
{

    Motor_Brake();
    __HAL_TIM_SET_COMPARE(MOTOR_PWM_TIM, MOTOR_PWM_CH, 0);
    HAL_TIM_PWM_Start(MOTOR_PWM_TIM, MOTOR_PWM_CH);

    /* 初始状态：全部拉低（停止） */
    HAL_GPIO_WritePin(MOTOR_A_IN1_GPIO,  MOTOR_A_IN1_PIN,  GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_A_IN2_GPIO,  MOTOR_A_IN2_PIN,  GPIO_PIN_RESET);

}

/* ================================================================
 *                         电机 A 控制函数
 * ================================================================ */

/* 启动 PWM 输出（在主函数中调用一次） */
void MotorPower_Start(void)
{
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);   // 启动 TIM3_CH2 的 PWM
}

/* 设置电机功率：duty 范围 0 ~ 7200（对应 0% ~ 100%） */
void MotorPower_Set(uint16_t duty)
{
    if (duty > 7200) duty = 7200;               // 限幅保护
    /* 直接修改 CCR2 寄存器，即时生效 */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, duty);
}


// /**
//  * @brief  设置电机 A 的 PWM 占空比
//  * @param  speed: 0 ~ PWM_PERIOD(999)，0 = 停止，999 = 全速
//  */
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
    MotorPower_Set(speed);
}

/**
 * @brief  电机 A 反转 回收a低b高
 */
void MotorA_Backward(uint16_t speed)
{
    HAL_GPIO_WritePin(MOTOR_A_IN1_GPIO,  MOTOR_A_IN1_PIN,  GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_A_IN2_GPIO,  MOTOR_A_IN2_PIN,  GPIO_PIN_SET);
    // MotorA_SetSpeed(speed);
    MotorPower_Set(speed);
}

/**
 * @brief  电机 A 停止
 */
void MotorA_Stop(void)
{
    HAL_GPIO_WritePin(MOTOR_A_IN1_GPIO,  MOTOR_A_IN1_PIN,  GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_A_IN2_GPIO,  MOTOR_A_IN2_PIN,  GPIO_PIN_RESET);
    // MotorA_SetSpeed(0);
    MotorPower_Set(0);
}

/**
 * @brief  电机 A 停止
 */
static void Motor_Brake(void)
{
    HAL_GPIO_WritePin(MOTOR_A_IN1_GPIO,  MOTOR_A_IN1_PIN,  GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_A_IN2_GPIO,  MOTOR_A_IN2_PIN,  GPIO_PIN_RESET);
    // MotorA_SetSpeed(0);
    MotorPower_Set(0);
}

static void Motor_SetDir(uint8_t dir)
{
    if (dir == 0) {                         /* 正转：IN1=1, IN2=0 */
        HAL_GPIO_WritePin(MOTOR_A_IN1_GPIO,  MOTOR_A_IN1_PIN,  GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_A_IN2_GPIO,  MOTOR_A_IN2_PIN,  GPIO_PIN_RESET);
    } else {                                /* 反转：IN1=0, IN2=1 */
        HAL_GPIO_WritePin(MOTOR_A_IN1_GPIO,  MOTOR_A_IN1_PIN,  GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_A_IN2_GPIO,  MOTOR_A_IN2_PIN,  GPIO_PIN_SET);
    }
}



/*============================== 设置目标速度 ==============================*/
void Motor_Set(int16_t speed)
{
    /* 限幅保护 */
    if (speed >  MOTOR_PWM_MAX) speed =  MOTOR_PWM_MAX;
    if (speed < -MOTOR_PWM_MAX) speed = -MOTOR_PWM_MAX;

    g_motor.target = speed;

    /* 解析目标方向：正数=正转，负数=反转，0保持原方向（用于停止后同向再启动） */
    if (speed > 0)       g_motor.target_dir = 0;
    else if (speed < 0)  g_motor.target_dir = 1;
    /* speed==0 时不改写 target_dir，避免停止后丢失方向记忆 */

    /* 目标为 0：进入减速流程 */
    if (speed == 0) {
        g_motor.state = MOTOR_RAMP;
        return;
    }

    /* 当前静止：直接设方向并启动斜坡 */
    if (g_motor.current == 0) {
        g_motor.dir = g_motor.target_dir;
        Motor_SetDir(g_motor.dir);
        g_motor.state = MOTOR_RAMP;
        return;
    }

    /* 当前在转且方向不一致：必须先降到 0 才能切方向 */
    if (g_motor.dir != g_motor.target_dir) {
        /* 状态设为 RAMP，让 Process() 把 current 降到 0；
           降到 0 后 Process() 会自动进入 MOTOR_DIR_BRAKE 死区 */
        g_motor.state = MOTOR_RAMP;
    }
    /* 同方向或已在降速：state 保持 MOTOR_RAMP，Process() 会自动跟踪 target */
}

/*============================== 斜坡处理核心（非阻塞） ==============================*/
void Motor_Process(void)
{
    uint32_t now = HAL_GetTick();
    if ((now - g_motor.last_tick) < MOTOR_RAMP_PERIOD) return;
    g_motor.last_tick = now;

    int16_t target_abs = abs(g_motor.target);

    switch (g_motor.state) {
        /*---------------- 静止 ----------------*/
        case MOTOR_IDLE:
            g_motor.current = 0;
            __HAL_TIM_SET_COMPARE(MOTOR_PWM_TIM, MOTOR_PWM_CH, 0);
            break;

        /*---------------- 斜坡跟踪（加速/减速/降到 0 换向） ----------------*/
        case MOTOR_RAMP: {
            /* current 向 target_abs 靠近 */
            if (g_motor.current < target_abs) {
                g_motor.current += MOTOR_RAMP_STEP;
                if (g_motor.current > target_abs) g_motor.current = target_abs;
            } else if (g_motor.current > target_abs) {
                g_motor.current -= MOTOR_RAMP_STEP;
                if (g_motor.current < target_abs) g_motor.current = target_abs;
                if (g_motor.current < 0)          g_motor.current = 0;
            }

            __HAL_TIM_SET_COMPARE(MOTOR_PWM_TIM, MOTOR_PWM_CH, (uint16_t)g_motor.current);

            /* 情况 A：已降到 0，且目标也是 0 → 完全停止 */
            if (g_motor.current == 0 && g_motor.target == 0) {
                Motor_Brake();
                g_motor.state = MOTOR_IDLE;
            }
            /* 情况 B：已降到 0，但目标非 0 且方向不同 → 进入方向切换死区 */
            else if (g_motor.current == 0 && g_motor.target != 0 && g_motor.dir != g_motor.target_dir) {
                Motor_Brake();                      /* 刹车短接，快速消耗反电动势 */
                g_motor.dir = g_motor.target_dir;   /* 预置新方向 */
                Motor_SetDir(g_motor.dir);          /* 先切换 GPIO（此时 PWM=0，H 桥无电流） */
                g_motor.state = MOTOR_DIR_BRAKE;
                g_motor.brake_tick = now;
            }
            /* 情况 C：已降到 0，目标同向 → 继续加速（理论上不会发生，因为同向不会先降到0） */
            /* 情况 D：达到目标值 → 保持运行（state 保持 RAMP，持续跟踪 target） */
            break;
        }

        /*---------------- 方向切换死区：PWM=0，已切方向，等待电机机械停转 ----------------*/
        case MOTOR_DIR_BRAKE: {
            __HAL_TIM_SET_COMPARE(MOTOR_PWM_TIM, MOTOR_PWM_CH, 0);
            /* 死区时间到后，才允许 PWM 从 0 开始爬升 */
            if ((now - g_motor.brake_tick) >= MOTOR_DIR_DEAD_MS) {
                g_motor.state = MOTOR_RAMP;         /* 回到斜坡，current 从 0 向 target_abs 增加 */
            }
            break;
        }

        default:
            break;
    }
}

/*============================== 获取当前实际输出（带符号） ==============================*/
int16_t Motor_GetCurrent(void)
{
    return (g_motor.dir == 0) ? g_motor.current : -g_motor.current;
}
