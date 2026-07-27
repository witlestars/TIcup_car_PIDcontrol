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

extern volatile uint32_t g_sys_tick;

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
}

/* GPIO GROUP0 中断: MSPM0G3507 默认所有 GPIO 中断走 GROUP0
 * 同时检查 GPIOA(PA7/PA18) 和 GPIOB(PB1/PB14), ISR 只设标志位 */
void GROUP0_IRQHandler(void)
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

/* 主循环10ms调: 检查标志位执行业务 (ISR设标志后, 这里安全调用CMD_SendText等) */
void Button_HandleEvents(void)
{
    uint8_t flag = g_btn_flag;
    g_btn_flag = 0;

    if (flag & BTN_BIT_START) {
        if (g_laps_done) {
            g_laps_done = 0; g_current_lap = 0; g_corner_count = 0;
            g_running = 0; Motor_Stop();
            CMD_SendText("[MSPM0] LAPS DONE, reset\n");
        } else if (g_running) {
            g_running = 0; Motor_Stop();
            CMD_SendText("[MSPM0] BTN STOP\n");
        } else {
            g_running = 1; g_track_locked = 0;
            CMD_SendText("[MSPM0] BTN START\n");
        }
    }

    if (flag & BTN_BIT_LAP_UP) {
        if (!g_running && g_target_laps < 9) {
            g_target_laps++;
            CMD_SendText("[MSPM0] target_laps+1\n");
        }
    }

    if (flag & BTN_BIT_MODE) {
        switch_mode(!g_use_imu);
    }

    if (flag & BTN_BIT_RESET) {
        g_running = 0; Motor_Stop();
        if (g_imu_present) IMU_Calibrate_Z();
        Odom_Reset();
        g_current_lap = 0; g_corner_count = 0; g_laps_done = 0;
        CMD_SendText("[MSPM0] BTN RESET\n");
    }
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
