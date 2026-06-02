#ifndef ARM_FOLD_H
#define ARM_FOLD_H
#include "stm32f4xx_hal.h"

extern void arm_Extension(void);        //手臂伸展
extern void arm_Flexion(void);      // 手臂折叠

#define MOTOR_COUNT     3

typedef enum {
    MOTOR_STOP = 0,
    MOTOR_RUN
} MotorState_t;

typedef struct {
    uint32_t startTime_ms;      // 启动时刻基准
    uint32_t duration_ms;       // 设定转动时长
    MotorState_t state;         // 当前运行状态
    void (*Start)(void);        // 电机启动函数指针
    void (*Stop)(void);         // 电机关闭函数指针
} MotorCtrl_t;

/* 全局变量（跨文件使用） */
extern MotorCtrl_t g_motors[MOTOR_COUNT];
extern volatile uint8_t g_keyDebounce;
extern volatile uint8_t g_systemRunning;
extern TIM_HandleTypeDef htim6;
extern extern void armfold_init(void);

#endif
