/**
 * @file    main.c
 * @brief   巡线小车主程序 — MSPM0G3507 + 四路驱动板
 *          灰度巡线 + IMU + 编码器里程 + 蓝牙调参
 *          非阻塞调度: IMU解析每轮 + 巡线/电机/按钮 10ms
 */

#include "ti_msp_dl_config.h"
#include "BSP/track.h"
#include "BSP/template/motor.h"
#include "BSP/template/cmd.h"
#include "BSP/template/uart_bluetooth.h"
#include "BSP/imu.h"
#include "BSP/odometry.h"
#include "BSP/oled.h"
#include "BSP/button.h"
#include "delay.h"

/* SysTick 1ms 时基 (非阻塞调度用; delay.c 用 delay_cycles 不冲突) */
volatile uint32_t g_sys_tick = 0;
void SysTick_Handler(void) { g_sys_tick++; }

/* IMU 解析启用/暂停切换 (按钮和 'E' 命令共用; IMU已改串口, 与OLED不互斥) */
void switch_mode(uint8_t use_imu)
{
    g_use_imu = use_imu;
    CMD_SendText(use_imu ? "[MSPM0] IMU enabled (parsing on)\n"
                         : "[MSPM0] IMU paused (parsing off)\n");
}

/* 运行模式分发: 按 g_mode 调对应模块, 设置 g_motor_l/r_speed
 *  mode 0=巡线  1=空转  3=正方形/T命令 */
static void Mode_RunStep(void)
{
    if (!g_running) {
        g_motor_l_speed = 0; g_motor_r_speed = 0;
        return;
    }
    switch (g_mode) {
    case 0:
        Track_Loop();
        Odom_Update();
        break;
    case 3:
        Square_Loop();
        break;
    default:   /* mode 1 空转 */
        g_motor_l_speed = (int16_t)g_target_rpm;
        g_motor_r_speed = (int16_t)g_target_rpm;
        break;
    }
}

int main(void)
{
    SYSCFG_DL_init();
    SysTick_Config(32000);   /* 1ms 时基 (32MHz/1000) */

    CMD_SendText("[MSPM0] init: track\n");  Track_Init();
    CMD_SendText("[MSPM0] init: cmd\n");    CMD_Init();
    CMD_SendText("[MSPM0] init: motor\n");  Motor_Init();

    /* IMU: UART_IMU 9600bps, 等500ms让JY61P冷启动, 启用RX中断 */
    delay_ms(500);
    IMU_EnableRxIRQ();
    CMD_SendText("[MSPM0] init: imu\n");
    CMD_SendText(IMU_Init() == 0 ? "[MSPM0] IMU JY61P OK\n" : "[MSPM0] IMU JY61P FAIL\n");

    CMD_SendText("[MSPM0] init: odom\n");   Odom_Init();
    CMD_SendText("[MSPM0] init: oled\n");   OLED_Init();
    if (g_oled_present) {
        OLED_Clear();
        OLED_PrintfAt(0, 0, "==TI CUP==");
        OLED_PrintfAt(1, 0, "ready");
    }

    CMD_SendText("[MSPM0] init: button\n"); Button_Init();
    CMD_SendText("[MSPM0] init done\n");

    uint32_t last_10ms = 0;
    while (1) {
        CMD_Poll();
        IMU_EchoTick();
        if (g_use_imu) IMU_Poll();   /* 每轮解析降低 yaw 延迟 */

        /* 10ms 节拍: 按钮 + 模式分发 + 电机指令 */
        if ((uint32_t)(g_sys_tick - last_10ms) >= 10) {
            last_10ms = g_sys_tick;

            Button_HandleEvents();   /* ISR 设标志, 这里执行业务 */

            if (g_laps_done && g_running) {
                g_running = 0; Motor_Stop();
                CMD_SendText("[MSPM0] LAPS DONE! auto-stop\n");
            }

            Mode_RunStep();          /* 按当前模式驱动电机 */
            Motor_Send_Speed(0, -g_motor_r_speed, 0, -g_motor_l_speed);
        }
    }
}
