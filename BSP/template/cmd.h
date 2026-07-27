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
extern uint8_t g_uart_debug_echo; /* 1=UART_DEBUG 原始字节回显到 PC */

void CMD_Init(void);
void CMD_Poll(void);
void CMD_SendText(const char *text);

/**
 * @brief 切换 IMU↔OLED (共用 PA17/PA15 软件 I2C, 互斥访问)
 * @param use_imu 1=启用IMU(禁用OLED), 0=启用OLED(禁用IMU)
 * @note 切换时会先停用另一方, 再重新初始化启用方, 并发文本通知电脑
 */
void switch_mode(uint8_t use_imu);

#endif
