/**
 * @file    cmd.h
 * @brief   UART 命令解析 — VOFA+ 远程调参
 * 
 * 命令格式 (通过 WiFi→ESP32→UART1):
 *   b12     BASE_RPM = 12
 *   p18     TURN_GAIN_P = 18
 *   d1.5    TURN_GAIN_D = 1.5
 *   k4      speed kp = 4.0
 *   i0.5    speed ki = 0.5
 *   j0.5    speed kd = 0.5
 *   stop    停车
 *   go      恢复
 *   ?       回传当前参数
 */

#ifndef __CMD_H
#define __CMD_H
#include <stdint.h>

/* 全局模式/目标 */
extern uint8_t g_mode;       /* 0=巡线, 1=空转, 3=不倒翁(IMU yaw自稳) */
extern float   g_target_rpm; /* 空转目标 RPM */
extern uint8_t g_running;    /* 1=运行, 0=停止 (上电默认0, 发'g'启动, 's'停止) */
extern uint8_t g_imu_uart_echo; /* 1=IMU 串口原始字节回显到 PC */

void CMD_Init(void);
void CMD_Poll(void);
void CMD_SendText(const char *text);

/**
 * @brief 切换 IMU↔OLED (共用 PA17/PA15 软件 I2C, 互斥访问)
 * @param use_imu 1=启用IMU(禁用OLED), 0=启用OLED(禁用IMU)
 * @note 切换时会先停用另一方, 再重新初始化启用方, 并发文本通知电脑
 */
void switch_mode(uint8_t use_imu);

/* ─── m3 正方形行进 / T 命令 状态机 (实现合并自原 square.c) ───
 * 状态机: 0=直行 1=转弯 2=完成 3=刹车(停车200ms消惯性)
 * 左转=yaw+, T命令(g_one_shot_turn=1)优先级最高 */
extern uint8_t g_square_state;      /* 0=直行 1=转弯 2=完成 3=刹车 */
extern uint8_t g_square_edge;       /* 已完成边数 (0~4) */
extern uint8_t g_one_shot_turn;     /* T 命令单次转弯标志 */
extern float   g_square_yaw_base;   /* 当前边直行目标朝向 */
extern float   g_turn_start_yaw;    /* 当前转弯起点 yaw */
extern float   g_one_shot_angle;    /* T 命令目标角度 */

/** 切 m3 时调用: 锁当前 yaw, 复位状态机和里程计 */
void Square_Init(void);

/** T 命令: 转任意角度后停车 (正=左转 yaw+, 负=右转 yaw-)
 *  会切到 m3 模式并自动启动, IMU 离线或角度超 ±360° 拒绝 */
void Square_Turn(float angle);

/** m3 模式主循环 10ms 调用: 执行状态机, 设置 g_motor_l/r_speed
 *  内含 IMU 离线保护, T 命令优先级最高 */
void Square_Loop(void);

#endif
