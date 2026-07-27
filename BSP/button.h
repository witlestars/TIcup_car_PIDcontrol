/**
 * @file    button.h
 * @brief   4个按钮 GPIO 中断驱动 + 事件队列
 *
 * 硬件接线 (按钮一端接 MSPM0 pin, 另一端接 GND, 内部上拉):
 *   BTN_START  PA7  (A07)  开始/停止 (切换 g_running)
 *   BTN_LAP_UP PA18 (A18)  目标圈数 +1
 *   BTN_MODE   PB1  (B01)  切换 IMU 解析开关
 *   BTN_RESET  PB14 (B14)  清零里程计 + IMU归零 + 圈数清零
 *
 * 按下=低电平, 松开=高电平 (内部上拉)
 * 消抖: SysConfig 硬件 8 cycles 消抖 + ISR 内 20ms 软件消抖兜底
 * 事件: 下降沿中断触发, push 到 FIFO, 主循环 Button_Get_Event 取出
 *
 * 架构 (中断驱动, 2026-07-27 从轮询改为中断):
 *   SysConfig 配置 4 个按钮下降沿中断 + 8 周期硬件消抖
 *   GROUP0_IRQHandler 处理 GPIOA + GPIOB pending, 20ms 消抖, push 事件
 *   主循环 Button_Get_Event 取事件执行业务 (ISR 不做业务)
 */

#ifndef __BUTTON_H
#define __BUTTON_H
#include <stdint.h>

/* 按钮事件 */
typedef enum {
    BTN_EVENT_NONE = 0,
    BTN_EVENT_START,      /* START 按下 */
    BTN_EVENT_LAP_UP,     /* 圈数+1 */
    BTN_EVENT_MODE,       /* 切换 IMU 解析开关 */
    BTN_EVENT_RESET,      /* 复位 */
} btn_event_t;

/** 初始化 (GPIO 中断已由 SysConfig 启用, 这里只清事件队列) */
void Button_Init(void);

/** 取出下一个事件 (FIFO, 容量8), 无事件返回 BTN_EVENT_NONE
 *  事件由 GROUP0_IRQHandler (GPIO下降沿中断) push 到队列 */
btn_event_t Button_Get_Event(void);

/** 读取4个按钮当前电平 (调试用, 给OLED显示)
 *  返回值 bit0~bit3 对应 START/LAP_UP/MODE/RESET
 *  1=按下(低电平), 0=松开(高电平) */
uint8_t Button_Get_Raw_Level(void);

#endif
