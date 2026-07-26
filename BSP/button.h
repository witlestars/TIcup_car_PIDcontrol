/**
 * @file    button.h
 * @brief   4个按钮输入 + 软件消抖
 *
 * 硬件接线 (按钮一端接 MSPM0 pin, 另一端接 GND, 内部上拉):
 *   BTN_START  PA7  (A07)  开始/停止 (切换 g_running)
 *   BTN_LAP_UP PA18 (A18)  目标圈数 +1
 *   BTN_MODE   PB1  (B01)  切换 IMU↔OLED (同一I2C总线互斥)
 *   BTN_RESET  PB14 (B14)  清零里程计 + IMU归零 + 圈数清零
 *
 * 注意: PB1 原为电机 AIN_2, 已将 AIN_2 移到 PA13 (代码未使用, 仅保留配置)
 *       原 LAP_DN(圈数-1) 改为 MODE 切换, 圈数-1 走串口 'L' 命令
 *
 * 按下=低电平, 松开=高电平 (内部上拉)
 * 消抖: 连续3次(30ms)读到低电平判为按下, 释放同理
 * 事件: 按下边沿触发一次 (不连发)
 */

#ifndef __BUTTON_H
#define __BUTTON_H
#include <stdint.h>

/* 按钮事件 */
typedef enum {
    BTN_EVENT_NONE = 0,
    BTN_EVENT_START,      /* START 按下 */
    BTN_EVENT_LAP_UP,     /* 圈数+1 */
    BTN_EVENT_MODE,       /* 切换 IMU↔OLED */
    BTN_EVENT_RESET,      /* 复位 */
} btn_event_t;

/** 初始化 (GPIO 已由 SysConfig 配置, 这里只清状态) */
void Button_Init(void);

/** 每10ms调用: 消抖 + 事件检测 */
void Button_Poll(void);

/** 取出下一个事件 (FIFO, 容量4), 无事件返回 BTN_EVENT_NONE */
btn_event_t Button_Get_Event(void);

/** 读取4个按钮当前电平 (调试用, 给OLED显示)
 *  返回值 bit0~bit3 对应 START/LAP_UP/LAP_DN/RESET
 *  1=按下(低电平), 0=松开(高电平)
 */
uint8_t Button_Get_Raw_Level(void);

#endif
