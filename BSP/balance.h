#ifndef __BALANCE_H
#define __BALANCE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ZDT_X42S_Driver.h"
#include <stdint.h>
#include <stdbool.h>

/* PID 参数结构体 (保持不变) */
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float Kff;
    float error_sum;
    float last_error;
    float out_max;
    float integral_max;
} Balance_PID_t;

#define BALANCE_ANGLE_MIN_DEFAULT_DEG    (-35.0f)
#define BALANCE_ANGLE_MAX_DEFAULT_DEG    (50.0f)
#define BALANCE_RETURN_SPEED_DEFAULT     (15)
#define BALANCE_ZERO_TOLERANCE_DEG       (0.2f)


extern volatile Balance_PID_t g_pos_pid;    // 位置外环
extern volatile Balance_PID_t g_vel_pid;    // 速度中环
extern volatile Balance_PID_t g_angle_pid;  // 角度内环


extern volatile float g_balance_motor_angle_deg;
extern volatile bool g_balance_angle_limit_active;
extern volatile bool g_balance_motor_feedback_valid;
extern volatile uint32_t g_balance_motor_feedback_tick;

void Balance_Init(void);
int Balance_MotorSetSpeed(int16_t speed);
void Balance_SetAngleLimits(float min_angle_deg, float max_angle_deg);
float Balance_GetMotorAngle(void);
void Balance_MotorRXByteCallback(uint8_t data);
void Balance_MotorFeedbackTask(void);
void Balance_StartReturnToZero(int16_t speed);
void Balance_CancelReturnToZero(void);
bool Balance_IsReturningToZero(void);
bool Balance_ReturnToZeroTask(void);

// 串级pid集成
void Balance_PID();

/**
 * @brief  平衡控制核心任务 (含延时补偿与真实速度微分)
 * @param  target_pos  小球的目标位置 (mm)
 * @param  vision_pos  视觉模块反馈的小球位置 (mm)
 * @param  vision_vel  视觉模块反馈的小球速度 (mm/s)
 * @param  age_ms      该帧视觉数据的年龄/延时 (ms)
 * @param  gyro_rate   陀螺仪当前输出的角速度
 */
/* Generic PID; all gains and runtime state come from the passed object. */
float PID_Calcula(volatile Balance_PID_t *pid, float target,
                  float feedback, float feedback_rate,
                  float feedforward, float dt_s);

#ifdef __cplusplus
}
#endif

#endif /* __BALANCE_H */
