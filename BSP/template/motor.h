/**
 * @file    motor.h
 * @brief   双轮差速电机驱动 — 四路驱动板 I2C 方案
 * 
 * 硬件:
 *   四路电机驱动板 (板载PID + 编码器读取)
 *   通信: I2C (SCL=PA12, SDA=PA13, 地址 0x26)
 * 
 * 架构 (大幅简化):
 *   MSPM0 只负责:
 *     1. 灰度巡线 → 计算左右目标速度
 *     2. 通过 I2C 写寄存器 0x06 控制速度
 *     3. 驱动板自带 PID 闭环控制电机
 * 
 * SysConfig 配置:
 *   1. 删掉 PWM_DRIVER (不再需要)
 *   2. 添加 GPIO, 命名 MOTOR_I2C (不能叫 I2C_MOTOR, pin 名 SDA/SCL 会和
 *      OLED/IMU 用的 I2C 实例重名, SysConfig 报 Duplicate name)
 *   3. SCL=PB12, SDA=PB11, 软件模拟 I2C, 与 OLED/IMU 物理隔离
 * 
 * 电机接口 (驱动板丝印):
 *   M1=左前, M2=左后, M3=右前, M4=右后
 *   两驱车: 用 M1(左) + M3(右), M2/M4 给0
 */

#ifndef __MOTOR_H
#define __MOTOR_H
#include "stdlib.h"
#include "ti_msp_dl_config.h"

/* ────────────── 物理参数 (信息保留) ────────────── */
#define MOTOR_SPEED_RERATIO  40
#define PULSE_PRE_ROUND      11
#define RADIUS_OF_TYRE       6.5f

/* ────────────── 速度范围 ────────────── */
#define MOTOR_SPEED_MAX      1000
#define MOTOR_SPEED_MIN     -1000

/* ────────────── 全局目标速度 (由 track.c 写入) ────────────── */
extern int16_t g_motor_l_speed;
extern int16_t g_motor_r_speed;

/* ────────────── API ────────────── */

/** 初始化: I2C 配置驱动板参数 */
void Motor_Init(void);

/** 发送速度指令 (I2C 写寄存器 0x06) */
void Motor_Send_Speed(int16_t m1, int16_t m2, int16_t m3, int16_t m4);

/** 停车 */
void Motor_Stop(void);

/** 配置驱动板 PID (暂不实现, I2C无PID寄存器) */
void Motor_Set_PID(float kp, float ki, float kd);

/** 读取电池电压 (V) */
float Motor_Read_Battery(void);

/** 读取左轮编码器 10ms 脉冲 */
int16_t Motor_Read_Encoder_L(void);

/** 读取右轮编码器 10ms 脉冲 */
int16_t Motor_Read_Encoder_R(void);

#endif
