/**
 * @file    balance.h
 * @brief   板球系统平衡控制算法头文件
 */

#ifndef __BALANCE_H
#define __BALANCE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* PID 参数结构体 */
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float Kff;//前馈增益
    float error_sum;
    float last_error;
    float out_max;
    float integral_max;
} Balance_PID_t;

extern Balance_PID_t g_balance_pid;

/**
 * @brief 初始化平衡控制系统 (包含电机初始化)
 */
void Balance_Init(void);

/**
 * @brief  平衡控制核心任务 (需放入 10ms 或 20ms 的定时调度中)
 * @param  target_pos  小球的目标位置
 * @param  current_pos 视觉模块反馈的小球当前位置
 * @param  gyro_rate   陀螺仪当前输出的角速度 (例如绕横滚轴的角速度)
 */
void Balance_Task(float target_pos, float current_pos, float gyro_rate);

#ifdef __cplusplus
}
#endif

#endif /* __BALANCE_H */
