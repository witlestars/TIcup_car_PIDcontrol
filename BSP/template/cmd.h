/**
 * @file    cmd.h
 * @brief   UART 命令解析 (精简版, H题)
 *
 * 命令格式 (通过 WiFi→ESP32→软件串口 PB8/PB10):
 *   b200    base_speed = 200 (驱动板单位)
 *   p18     turn_p = 18
 *   d1.5    turn_d = 1.5
 *   s       急停
 *   g       恢复
 *   N       切换题目 (N=下一题, N3=第3题)
 *   ?       回传当前状态
 *   B       回显启动状态
 */

#ifndef __CMD_H
#define __CMD_H
#include <stdint.h>

/* 1ms 时基 (定义在 main.c) */
extern volatile uint32_t g_sys_tick;

void CMD_Init(void);
void CMD_Poll(void);
void CMD_SendText(const char *text);

#endif
