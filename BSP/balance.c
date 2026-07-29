/**
 * @file    balance.c
 * @brief   板球系统平衡控制算法实现 (加入陀螺仪角速度前馈)
 */

#include "balance.h"
#include "ti_msp_dl_config.h"  // 必须引入，为了使用 UART_Motor_INST 和 DMA_CH0_CHAN_ID
#include "ZDT_X42S_Driver.h"   // 引入电机驱动底层[cite: 1]
#include "delay.h"             // delay_ms / delay_us

/* ================== 私有变量 ================== */
/* 将步进电机实例完全封装在本文件中，对外隐藏[cite: 1] */
static ZDT_MotorTypeDef balance_motor;

/* ================== 全局变量 ================== */
Balance_PID_t g_balance_pid;


/**
 * @brief 初始化平衡系统 (含电机硬件初始化)
 */
void Balance_Init(void)
{
    /* ------ 1. 硬件初始化部分 ------ */
    /* 初始化电机结构体 (假设 ID 为 1)[cite: 1] */
    ZDT_Motor_Init(&balance_motor, UART_Motor_INST, DMA_CH0_CHAN_ID, 1);

    /* 上电延时 500ms 等待电机驱动板就绪 (初始化阶段, 不影响主循环) */
    delay_ms(500);

    /* 使能电机锁轴 */
    ZDT_Motor_Enable(&balance_motor, true);
    delay_ms(10);


    /* ------ 2. PID 参数初始化部分 ------ */
    g_balance_pid.Kp = 15.0f;
    g_balance_pid.Ki = 0.0f;
    g_balance_pid.Kd = 5.0f;
    
    /* 新增：前馈控制系数 (具体值需上板调试，通常与电机转速响应有关) */
    g_balance_pid.Kff = 2.5f; 
    
    g_balance_pid.error_sum = 0.0f;
    g_balance_pid.last_error = 0.0f;
    
    g_balance_pid.out_max = 200.0f;      // 限速 200 RPM[cite: 1]
    g_balance_pid.integral_max = 50.0f;  // 积分限幅[cite: 1]
}

/**
 * @brief 平衡控制核心任务 (PID + 陀螺仪前馈 计算并输出速度)
 * @param target_pos 目标位置
 * @param current_pos 当前小球位置
 * @param gyro_rate 陀螺仪当前输出的角速度 (例如绕横滚轴的角速度)
 * @todo  无显性时间参，目前适配10ms周期，后续更改周期kikd需要修改[cite: 1]
 */
void Balance_PID(float target_pos, float current_pos, float gyro_rate)
{
    float error;
    float derivative;
    float feedforward;
    float output;
    int16_t motor_speed;

    /* 1. 计算误差[cite: 1] */
    error = target_pos - current_pos;

    /* 2. 积分计算 (抗积分饱和)[cite: 1] */
    g_balance_pid.error_sum += error;
    if (g_balance_pid.error_sum > g_balance_pid.integral_max) {
        g_balance_pid.error_sum = g_balance_pid.integral_max;
    } else if (g_balance_pid.error_sum < -g_balance_pid.integral_max) {
        g_balance_pid.error_sum = -g_balance_pid.integral_max;
    }

    /* 3. 微分计算[cite: 1] */
    derivative = error - g_balance_pid.last_error;

    /* 4. 前馈计算 (基于陀螺仪角速度) */
    feedforward = g_balance_pid.Kff * gyro_rate;

    /* 5. 复合输出：位置式 PID + 角速度前馈 */
    output = (g_balance_pid.Kp * error) + 
             (g_balance_pid.Ki * g_balance_pid.error_sum) + 
             (g_balance_pid.Kd * derivative) + 
             feedforward;

    /* 6. 记录历史误差[cite: 1] */
    g_balance_pid.last_error = error;

    /* 7. 输出量限幅[cite: 1] */
    if (output > g_balance_pid.out_max) {
        output = g_balance_pid.out_max;
    } else if (output < -g_balance_pid.out_max) {
        output = -g_balance_pid.out_max;
    }

    /* 8. 转换类型并下发指令给电机[cite: 1] */
    motor_speed = (int16_t)output;
    ZDT_Motor_SetSpeed(&balance_motor, motor_speed);
}
