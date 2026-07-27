/**
 * @file    main.c
 * @brief   巡线小车主程序 — 四路驱动板方案
 *
 * 架构:
 *   MSPM0 只负责:
 *     1. 灰度巡线 → Track_Loop() 计算左右速度
 *     2. 发送 $spd 指令给四路驱动板 (自带PID)
 *     3. 接收 ESP32 转发的调参命令
 *
 * 初始化流程:
 *   1. SysConfig + SysTick 1ms 时基
 *   2. 配置驱动板参数 (电机类型/减速比/磁环线/轮径)
 *   3. 进入主循环: 非阻塞调度 (巡线/电机 10ms + IMU 50ms)
 *
 * 命令 (通过 WiFi→ESP32→UART):
 *   m0      巡线模式
 *   m1      空转模式
 *   m3      正方形行进 (纯 IMU + 编码器, 非灰度)
 *   b200    基础速度
 *   p18/d1.5  转向PD
 *   k/i/j   驱动板PID
 *   s/g     停/走
 */

#include "ti_msp_dl_config.h"
#include "BSP/track.h"
#include "BSP/template/motor.h"
#include "BSP/template/cmd.h"
#include "BSP/template/uart_bluetooth.h"
#include "BSP/template/imu_uart.h"
#include "BSP/imu.h"
#include "BSP/odometry.h"
#include "BSP/oled.h"
#include "BSP/button.h"
#include "delay.h"

/* ─── SysTick 1ms 时基 (用于非阻塞调度, 代替 delay_ms 阻塞) ───
 * 32MHz 主频 / 1000 = 32000 计数 → 每 1ms 触发一次中断
 * SysTick_Handler 覆盖启动文件里的弱符号; delay.c 用 delay_cycles 不占 SysTick, 无冲突 */
volatile uint32_t g_sys_tick = 0;
void SysTick_Handler(void)
{
    g_sys_tick++;
}

/* ─── IMU 启用/暂停切换 ───
 * 按钮和串口 'E' 命令都走这个函数, 保证切换逻辑一致
 * (IMU 已改用串口 UART_DEBUG, 与 OLED 的 I2C 物理隔离, 不再互斥)
 * use_imu: 1=启用 IMU 解析, 0=暂停 IMU 解析 (OLED 始终独立工作) */
void switch_mode(uint8_t use_imu)
{
    g_use_imu = use_imu;
    if (use_imu) {
        CMD_SendText("[MSPM0] IMU enabled (parsing on)\n");
    } else {
        CMD_SendText("[MSPM0] IMU paused (parsing off, OLED unaffected)\n");
    }
}

/* ─── 正方形行进状态 (m3 模式) ───
 * 状态机: 0=直行 1=转弯 2=完成
 * edge: 已完成的边数 (0~4)
 * yaw_base: 当前边的目标朝向 (度) */
uint8_t g_square_state  = 0;
uint8_t g_square_edge   = 0;
float   g_square_yaw_base = 0.0f;

int main(void)
{
    SYSCFG_DL_init();
    /* 启用 SysTick 1ms 中断 (32MHz → 32000 计数/ms) */
    SysTick_Config(32000);

    /* 初始化模块 */
    CMD_SendText("[MSPM0] init: track\n");
    Track_Init();
    CMD_SendText("[MSPM0] init: cmd\n");
    CMD_Init();

    /* 配置驱动板 */
    CMD_SendText("[MSPM0] init: motor\n");
    Motor_Init();

    /* ── IMU 初始化 (串口 UART_DEBUG, PA10/PA11, 9600bps) ──
     * IMU 和 OLED 物理隔离, 不再互斥, 可同时工作
     * 延时 500ms 等 JY61P 上电启动 (JY61P 冷启动约需 200-500ms 才开始输出帧) */
    delay_ms(500);
    /* 启用 UART_DEBUG RX 中断: JY61P 字节到来自动进 ISR 搬到环形缓冲
     * 不再依赖主循环轮询 PollRx, 不会因主循环阻塞丢字节 */
    IMU_UART_EnableRxIRQ();
    CMD_SendText("[MSPM0] init: imu (UART_DEBUG 9600bps, RX IRQ enabled)\n");
    if (IMU_Init() == 0) {
        CMD_SendText("[MSPM0] IMU JY61P OK\n");
    } else {
        CMD_SendText("[MSPM0] IMU JY61P FAIL (skipped)\n");
    }

    CMD_SendText("[MSPM0] init: odom\n");
    Odom_Init();

    /* ── OLED 初始化 (PA17/PA15 I2C, 与 IMU 串口物理隔离) ── */
    CMD_SendText("[MSPM0] init: oled\n");
    OLED_Init();
    if (g_oled_present) {
        OLED_Clear();
        OLED_PrintfAt(0, 0, "==TI CUP==");
        OLED_PrintfAt(1, 0, "ready");
    }

    /* ── 按钮初始化: 纯 GPIO, 无 I2C ── */
    CMD_SendText("[MSPM0] init: button\n");
    Button_Init();
    CMD_SendText("[MSPM0] init done\n");

    /* 非阻塞调度: 高频任务每轮跑, 周期任务用 SysTick 时间戳
     * UART_DEBUG 已改中断接收, RX 字节自动进 ISR 搬到环形缓冲, 主循环不再需要 PollRx
     * 按钮消抖依赖 ~10ms 周期, 不能全速轮询, 故 Button_Poll 放 10ms 节拍 */
    uint32_t last_10ms = 0;   /* 按钮/PID/电机 节拍 */
    uint32_t last_50ms = 0;   /* IMU 解析节拍 */

    while (1) {
        /* ── 高频任务: 每轮执行, 不漏调参命令 ──
         * (JY61P 字节接收已由 UART0_IRQHandler 自动处理, 这里不需要 PollRx) */
        CMD_Poll();            /* 串口调参命令 */
        IMU_UART_EchoTick();   /* echo 诊断输出 (echo 开启时才工作, 非阻塞) */

        /* ── 10ms 节拍: 按钮消抖 + 巡线PID + 电机指令 ── */
        if ((uint32_t)(g_sys_tick - last_10ms) >= 10) {
            last_10ms = g_sys_tick;

            Button_Poll();
            btn_event_t evt;
            while ((evt = Button_Get_Event()) != BTN_EVENT_NONE) {
                switch (evt) {
                case BTN_EVENT_START:
                    if (g_laps_done) {
                        /* 跑完后按 START 重置圈数 */
                        g_laps_done = 0;
                        g_current_lap = 0;
                        g_corner_count = 0;
                        g_running = 0;
                        Motor_Stop();
                        CMD_SendText("[MSPM0] LAPS DONE, reset\n");
                    } else if (g_running) {
                        g_running = 0;
                        Motor_Stop();
                        CMD_SendText("[MSPM0] BTN STOP\n");
                    } else {
                        g_running = 1;
                        g_track_locked = 0;
                        CMD_SendText("[MSPM0] BTN START\n");
                    }
                    break;
                case BTN_EVENT_LAP_UP:
                    if (!g_running && g_target_laps < 9) {
                        g_target_laps++;
                        CMD_SendText("[MSPM0] target_laps+1\n");
                    }
                    break;
                case BTN_EVENT_MODE:
                    /* 切换 IMU 解析开关 (IMU 改串口后不再与 OLED 互斥, 此按钮只切 IMU 解析) */
                    switch_mode(!g_use_imu);
                    break;
                case BTN_EVENT_RESET:
                    g_running = 0;
                    Motor_Stop();
                    if (g_imu_present) IMU_Calibrate_Z();
                    Odom_Reset();
                    g_current_lap = 0;
                    g_corner_count = 0;
                    g_laps_done = 0;
                    CMD_SendText("[MSPM0] BTN RESET\n");
                    break;
                default: break;
                }
            }

            if (g_laps_done && g_running) {
                g_running = 0;
                Motor_Stop();
                CMD_SendText("[MSPM0] LAPS DONE! auto-stop\n");
            }

            /* ── 运行/停止/模式控制 ── */
            if (!g_running) {
                g_motor_l_speed = 0;
                g_motor_r_speed = 0;
            } else if (g_mode == 0) {
                Track_Loop();
                Odom_Update();   /* 巡线模式也推里程计 (过弯检测/圈数用) */
            } else if (g_mode == 3) {
                /* ── 正方形行进 (非灰度, 纯 IMU + 编码器里程) ──
                 * 切 m3 时在 cmd.c 锁定起点 yaw_target + Odom_Reset
                 * 状态机: 直线走 1500mm → 原地左转 90° → 下一条边, 走完 4 条边停车
                 * 左转 = 逆时针 = yaw 增加 (JY61P 约定) */
                extern uint8_t g_square_state;   /* 0=直行 1=转弯 2=完成 */
                extern uint8_t g_square_edge;
                extern float   g_square_yaw_base;
                #define SQ_EDGE_LEN    1500.0f   /* 边长 mm */
                #define SQ_TURN_THRESH 5.0f      /* 转弯到位阈值 (度) */
                #define SQ_BASE_SPD    200       /* 直行基础速度 */
                #define SQ_TURN_SPD    200       /* 原地转弯速度 */
                #define SQ_YAW_KP       30.0f    /* 直行 yaw 修正 P (驱动板单位/度) */

                float yaw_now = IMU_Get_Yaw_Cached();
                g_yaw_now = yaw_now;

                if (g_square_state == 2) {
                    /* 已完成 4 条边, 停车 */
                    g_motor_l_speed = 0;
                    g_motor_r_speed = 0;
                    g_running = 0;
                    Motor_Stop();
                    CMD_SendText("[MSPM0] SQUARE DONE! auto-stop\n");
                } else if (g_square_state == 0) {
                    /* 直行阶段: 走直线, 用 yaw 误差做差速保持直行 */
                    float err = g_square_yaw_base - yaw_now;
                    if (err > 180.0f)  err -= 360.0f;
                    if (err < -180.0f) err += 360.0f;
                    g_yaw_err = err;
                    float correction = err * SQ_YAW_KP;
                    if (correction > 150.0f)  correction = 150.0f;
                    if (correction < -150.0f) correction = -150.0f;
                    g_motor_l_speed = (int16_t)(SQ_BASE_SPD - correction);
                    g_motor_r_speed = (int16_t)(SQ_BASE_SPD + correction);
                    /* 判断是否走完一条边 */
                    if (Odom_Get_Edge_Dist() >= SQ_EDGE_LEN) {
                        g_square_state = 1;
                        /* 转弯目标 = 当前朝向 + 90° (左转, 逆时针为正) */
                        g_square_yaw_base = yaw_now + 90.0f;
                        if (g_square_yaw_base > 180.0f) g_square_yaw_base -= 360.0f;
                        CMD_SendText("[MSPM0] SQUARE: edge done, turning\n");
                    }
                } else {
                    /* 转弯阶段: 原地左转 (左轮反转, 右轮正转) */
                    float err = g_square_yaw_base - yaw_now;
                    if (err > 180.0f)  err -= 360.0f;
                    if (err < -180.0f) err += 360.0f;
                    g_yaw_err = err;
                    /* 未到位: 继续转 */
                    if (err > SQ_TURN_THRESH) {
                        /* 目标在当前朝向左侧 (yaw 需增加), 左转 */
                        g_motor_l_speed = -SQ_TURN_SPD;
                        g_motor_r_speed =  SQ_TURN_SPD;
                    } else if (err < -SQ_TURN_THRESH) {
                        /* 过冲, 右转修正 */
                        g_motor_l_speed =  SQ_TURN_SPD;
                        g_motor_r_speed = -SQ_TURN_SPD;
                    } else {
                        /* 到位, 进下一条边 */
                        g_square_state = 0;
                        g_square_edge++;
                        g_square_yaw_base = yaw_now;   /* 锁新朝向 */
                        Odom_Reset_Edge();
                        if (g_square_edge >= 4) {
                            g_square_state = 2;   /* 4 条边走完 */
                        }
                        CMD_SendText("[MSPM0] SQUARE: turn done, next edge\n");
                    }
                }
            } else {
                g_motor_l_speed = (int16_t)g_target_rpm;
                g_motor_r_speed = (int16_t)g_target_rpm;
            }

            /* ── 发送速度指令给驱动板 ── */
            Motor_Send_Speed(0, -g_motor_r_speed, 0, -g_motor_l_speed);
        }

        /* ── 50ms 节拍: IMU 帧解析 (JY61P 10Hz 输出, 50ms 够用) ──
         *    字节已由 UART0_IRQHandler (RX 中断) 自动搬进环形缓冲, 这里只做帧解析 */
        if ((uint32_t)(g_sys_tick - last_50ms) >= 50) {
            last_50ms = g_sys_tick;
            if (g_use_imu) {
                IMU_Poll();
            }
        }
    }
}
