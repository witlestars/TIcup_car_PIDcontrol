/**
 * @file    button.c
 * @brief   4按钮 GPIO 中断驱动 + 标志位 + 业务处理
 *          ISR 设 g_btn_flag, 主循环 Button_HandleEvents() 执行业务
 *
 * 教训/历程见 DEVELOPMENT_NOTES.txt
 */

#include "button.h"
#include "ti_msp_dl_config.h"
#include "cmd.h"
#include "motor.h"
#include "imu.h"
#include "odometry.h"
#include "track.h"
#include "k230.h"
#include "oled.h"
#include <stdio.h>

/* 按钮标志位 (ISR 设, HandleEvents 清) */
#define BTN_BIT_START   0x01
#define BTN_BIT_LAP_UP  0x02
#define BTN_BIT_MODE    0x04
#define BTN_BIT_RESET   0x08
static volatile uint8_t g_btn_flag = 0;

#define DEBOUNCE_MS  20   /* 软件消抖 (硬件8 cycles 消抖的兜底) */
static uint32_t s_last_tick[4];

void Button_Init(void)
{
    g_btn_flag = 0;
    for (uint8_t i = 0; i < 4; i++) s_last_tick[i] = 0;

    /* SysConfig 只开了 GPIO 外设级中断 (DL_GPIO_enableInterrupt), NVIC 没开。
     * MSPM0G3507 的 GPIO 中断走 GROUP1 (IRQn=1), 不是 GROUP0 —
     * 之前误用 GROUP0_IRQHandler 导致按钮中断从不触发。这里补 NVIC 使能 */
    NVIC_ClearPendingIRQ(GPIO_BUTTON_GPIOA_INT_IRQN);
    NVIC_ClearPendingIRQ(GPIO_BUTTON_GPIOB_INT_IRQN);
    NVIC_EnableIRQ(GPIO_BUTTON_GPIOA_INT_IRQN);
    NVIC_EnableIRQ(GPIO_BUTTON_GPIOB_INT_IRQN);
}

/* GPIO GROUP1 中断: MSPM0G3507 的 GPIOA/GPIOB 中断都走 GROUP1 (IRQn=1)
 * 同时检查 GPIOA(PA7/PA18) 和 GPIOB(PB1/PB14), ISR 只设标志位 */
void GROUP1_IRQHandler(void)
{
    uint32_t now = g_sys_tick;

    switch (DL_GPIO_getPendingInterrupt(GPIOA)) {
    case GPIO_BUTTON_BTN_START_IIDX:
        if ((now - s_last_tick[0]) >= DEBOUNCE_MS) {
            s_last_tick[0] = now; g_btn_flag |= BTN_BIT_START;
        }
        break;
    case GPIO_BUTTON_BTN_LAP_UP_IIDX:
        if ((now - s_last_tick[1]) >= DEBOUNCE_MS) {
            s_last_tick[1] = now; g_btn_flag |= BTN_BIT_LAP_UP;
        }
        break;
    default: break;
    }

    switch (DL_GPIO_getPendingInterrupt(GPIOB)) {
    case GPIO_BUTTON_BTN_LAP_DN_IIDX:   /* syscfg叫LAP_DN, 实际是MODE */
        if ((now - s_last_tick[2]) >= DEBOUNCE_MS) {
            s_last_tick[2] = now; g_btn_flag |= BTN_BIT_MODE;
        }
        break;
    case GPIO_BUTTON_BTN_RESET_IIDX:
        if ((now - s_last_tick[3]) >= DEBOUNCE_MS) {
            s_last_tick[3] = now; g_btn_flag |= BTN_BIT_RESET;
        }
        break;
    default: break;
    }
}

/* 主循环10ms调: 检查标志位执行业务 (ISR设标志后, 这里安全调用CMD_SendText等)
 *
 * H题按钮分配:
 *   START  (PA7):  启动/停止切换
 *   LAP_UP (PA18): 切换题目号 1→2→3→4→5→1, 同步OLED显示+发K230题号
 *   MODE   (PB1):  空置 (预留, H题暂未用)
 *   RESET  (PB14): 空置 (预留, H题暂未用)
 */
void Button_HandleEvents(void)
{
    uint8_t flag = g_btn_flag;
    g_btn_flag = 0;

    if (flag & BTN_BIT_START) {
        if (g_running) {
            g_running = 0; Motor_Stop();
            if (g_oled_present) OLED_PrintfAt(1, 0, "Task: %d STOP", g_task_id);
            CMD_SendText("[MSPM0] BTN STOP\n");
        } else {
            g_running = 1; g_track_locked = 0;
            if (g_oled_present) OLED_PrintfAt(1, 0, "Task: %d RUN ", g_task_id);
            CMD_SendText("[MSPM0] BTN START\n");
        }
    }

    if (flag & BTN_BIT_LAP_UP) {
        /* 切换题目 1→2→3→4→5→1 */
        g_task_id = (g_task_id >= 5) ? 1 : (g_task_id + 1);
        /* OLED 更新显示 */
        if (g_oled_present) {
            OLED_ClearArea(0, 0, 16);
            OLED_PrintfAt(0, 0, "Task: %d", g_task_id);
        }
        /* 发题号给 K230 */
        K230_SendTask(g_task_id);
        char msg[32];
        snprintf(msg, sizeof(msg), "[MSPM0] task=%d\n", g_task_id);
        CMD_SendText(msg);
    }

    /* MODE (PB1): 空置 — H题暂未分配功能 */
    /* if (flag & BTN_BIT_MODE) { } */

    /* RESET (PB14): 空置 — H题暂未分配功能 */
    /* if (flag & BTN_BIT_RESET) { } */
}

/* 调试用: bit0=START(PA7) bit1=LAP_UP(PA18) bit2=MODE(PB1) bit3=RESET(PB14) */
uint8_t Button_Get_Raw_Level(void)
{
    uint8_t r = 0;
    if ((DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_7)  == 0)) r |= 0x01;
    if ((DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_18) == 0)) r |= 0x02;
    if ((DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_1)  == 0)) r |= 0x04;
    if ((DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_14) == 0)) r |= 0x08;
    return r;
}
