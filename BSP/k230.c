/**
 * @file    k230.c
 * @brief   庐山派 K230 视觉模块对接 (GPIO 信号方案, 轮询)
 *          K230 识别到钢珠 → PA14 拉高 → MSPM0 轮询检测 → 设标志
 *
 * 为什么轮询而非中断:
 *   button.c 已占用 GROUP1_IRQHandler (GPIOA/GPIOB 中断),
 *   在同一中断里加 K230 判断会破坏模块隔离。
 *   10ms 轮询对视觉检测足够快 (人眼/机械动作延迟远大于10ms)。
 */

#include "k230.h"
#include "ti_msp_dl_config.h"

/* ─── 状态 ─── */
k230_state_t g_k230 = {0};

/* 上一次电平 (用于上升沿检测) */
static uint8_t s_last_level = 0;

void K230_Init(void)
{
    g_k230.bead_detected = 0;
    g_k230.detect_cnt    = 0;
    g_k230.sig_level     = 0;
    s_last_level         = 0;
}

/* 主循环 10ms 调: 轮询 PA14, 检测上升沿 (0→1) 设标志 */
void K230_Poll(void)
{
    uint8_t level = (DL_GPIO_readPins(GPIO_K230_PORT,
                                      GPIO_K230_K230_SIG_PIN) != 0) ? 1 : 0;
    g_k230.sig_level = level;

    /* 上升沿检测: 上次低, 本次高 = 新检测到钢珠 */
    if (level == 1 && s_last_level == 0) {
        g_k230.bead_detected = 1;
        g_k230.detect_cnt++;
    }
    /* 下降沿检测: K230 拉低 = 处理完/丢失, 可选清标志
     * 这里不清 bead_detected, 让应用层显式 K230_Clear() */
    s_last_level = level;
}

uint8_t K230_IsBeadDetected(void)
{
    return g_k230.bead_detected;
}

void K230_Clear(void)
{
    g_k230.bead_detected = 0;
}

uint8_t K230_Get_Signal(void)
{
    return (uint8_t)((DL_GPIO_readPins(GPIO_K230_PORT,
                                       GPIO_K230_K230_SIG_PIN) != 0) ? 1 : 0);
}
