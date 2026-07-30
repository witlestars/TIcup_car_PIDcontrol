/**
 * @file    motor.c
 * @brief   双电机驱动 — 四路驱动板 软件I2C方案
 * 
 * 使用厂商提供的 IOI2C + bsp_motor_iic 驱动
 * 通过 GPIO 模拟 I2C 控制四路电机驱动板 (地址 0x26)
 *
 * 接线:
 *   MSPM0 PA17(SDA) → 驱动板 SDA
 *   MSPM0 PA15(SCL) → 驱动板 SCL
 *   GND ↔ GND
 * 
 * 电机映射 (驱动板丝印):
 *   M1=左前, M2=左后, M3=右前, M4=右后
 *   两驱: M1=左轮, M3=右轮, M2/M4=0
 */

#include "motor.h"
#include "bsp_motor_iic.h"
#include "delay.h"

/* ────────────── 全局目标速度 ────────────── */
motor_target_t g_motor = {0};

/* ────────────── 对外 API ────────────── */

/**
 * @brief 发送速度指令 (I2C 写寄存器 0x06)
 * @param m1~m4  各电机速度 (-1000~1000)
 */
void Motor_Send_Speed(int16_t m1, int16_t m2, int16_t m3, int16_t m4)
{
    control_speed(m1, m2, m3, m4);
}

/**
 * @brief 急停
 */
void Motor_Stop(void)
{
    control_speed(0, 0, 0, 0);
    g_motor.l = 0;
    g_motor.r = 0;
}

/**
 * @brief 配置驱动板 PID (通过串口, I2C无此寄存器)
 */
void Motor_Set_PID(float kp, float ki, float kd)
{
    (void)kp; (void)ki; (void)kd;
    /* I2C 方案下 PID 只能通过串口设置 */
}

/**
 * @brief 初始化: 通过 I2C 配置驱动板参数
 */
void Motor_Init(void)
{
    delay_ms(500);
    
    /* 电机类型: 520 */
    Set_motor_type(1);
    delay_ms(100);
    
    /* 磁环线数: 11 */
    Set_Pluse_line(11);
    delay_ms(100);
    
    /* 减速比: 40 */
    Set_Pluse_Phase(40);
    delay_ms(100);
    
    /* 轮径: 65mm */
    Set_Wheel_dis(65.0f);
    delay_ms(100);
    
    /* 死区: 1900 (520电机默认) */
    Set_motor_deadzone(1900);
    delay_ms(100);
}

/**
 * @brief 读取 M4 编码器 10ms 实时脉冲 (左轮)
 * @note  Read_10_Enconder() 由调用方 (odometry.c) 统一调用, 此处直接读缓存
 */
int16_t Motor_Read_Encoder_L(void)
{
    return (int16_t)Encoder_Offset[3];  /* M4 */
}

/**
 * @brief 读取 M2 编码器 10ms 实时脉冲 (右轮)
 */
int16_t Motor_Read_Encoder_R(void)
{
    return (int16_t)Encoder_Offset[1];  /* M2 */
}
