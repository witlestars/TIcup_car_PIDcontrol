/**
 * @file    k230.h
 * @brief   庐山派 K230 视觉模块对接 (GPIO 信号方案)
 *
 * 硬件: 庐山派 K230 CanMV 开发板
 *   通信: GPIO 信号 (K230→MSPM0, 拉高=检测到钢珠)
 *   引脚: PA14 (MSPM0 输入, PULL_DOWN, K230 GPIO 输出)
 *
 * 为什么用 GPIO 而非 UART:
 *   MSPM0G3507 的 UART3 所有 TX/RX 引脚组合均被占用
 *   (PA13/PA14, PA25/PA26, PB2/PB3 各有一端被电机/灰度占用)
 *   故采用最简 GPIO 信号方案, 1 根线即可完成"识别到钢珠→通知小车"
 *
 * 工作原理:
 *   K230 端: 识别到钢珠 → GPIO 拉高 (MicroPython: Pin.out(1))
 *   MSPM0 端: 10ms 轮询 PA14, 拉高则设 bead_detected 标志
 *   应用层:   查询 K230_IsBeadDetected() 决定停车/拾取等动作
 *
 * K230 端 MicroPython 示例 (队友写):
 *   from machine import Pin
 *   sig = Pin(14, Pin.OUT, value=0)  # K230 的 GPIO14
 *   # 识别到钢珠:
 *   sig.value(1)
 *   # 处理完:
 *   sig.value(0)
 *
 * 接线:
 *   K230 GPIO (输出) ──── PA14 (MSPM0 输入)
 *   GND ──────────────── GND (共地)
 *   (3.3V 直连, 无需电平转换)
 */

#ifndef __K230_H
#define __K230_H
#include <stdint.h>

/* K230 状态 (诊断 + 控制) */
typedef struct {
    uint8_t  bead_detected;  /* 1=K230报告检测到钢珠 (轮询设, 应用层清) */
    uint32_t detect_cnt;     /* 累计检测上升沿次数 (诊断) */
    uint8_t  sig_level;      /* 当前 PA14 电平 (0/1, 诊断用) */
} k230_state_t;
extern k230_state_t g_k230;

/* 初始化: 清状态 (GPIO 由 SysConfig 配置, 无需额外代码) */
void K230_Init(void);

/* 主循环 10ms 调: 轮询 PA14, 检测上升沿设标志 */
void K230_Poll(void);

/* 查询是否检测到钢珠, 0=无 1=有 (不自动清, 需 K230_Clear() ) */
uint8_t K230_IsBeadDetected(void);

/* 清除检测标志 (处理完钢珠后调) */
void K230_Clear(void);

/* 读 PA14 原始电平 (调试用) */
uint8_t K230_Get_Signal(void);

#endif
