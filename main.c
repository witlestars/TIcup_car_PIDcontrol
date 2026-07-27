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
 * 状态机: 0=直行 1=转弯 2=完成 3=刹车(停车200ms等惯性消除)
 * edge: 已完成的边数 (0~4)
 * g_square_yaw_base: 当前边直行的目标朝向 (度, 用于直行保持)
 * g_turn_start_yaw:  当前转弯开始时的 yaw (度, 转弯用相对变化幅度判定)
 *                    delta = yaw_now - g_turn_start_yaw, 目标 delta=90° (左转, yaw+)
 *                    (JY61P 倒扣安装: 从车顶看左转=逆时针=yaw增加)
 * g_one_shot_turn:   T 命令触发的单次转弯 (1=待执行), 完成后自动停车
 * g_one_shot_angle:  T 命令目标角度 (正=左转 yaw+, 负=右转 yaw-)
 * g_brake_start:     刹车开始的 SysTick 时间戳 (ms) */
uint8_t g_square_state    = 0;
uint8_t g_square_edge     = 0;
float   g_square_yaw_base = 0.0f;
float   g_turn_start_yaw  = 0.0f;
uint8_t g_one_shot_turn   = 0;
float   g_one_shot_angle  = 0.0f;
uint32_t g_brake_start    = 0;

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
                /* ── 正方形行进 / 单次转弯 T 命令 (非灰度, 纯 IMU + 编码器里程) ──
                 * 切 m3 时在 cmd.c 锁定起点 yaw_target + Odom_Reset
                 * 状态机: 0=直行 1=转弯 2=完成 3=刹车(停车200ms)
                 * 直线走 1500mm → 原地左转 90° → 刹车200ms → 下一条边, 走完4条边停车
                 * 左转 = 逆时针(从车顶看) = yaw 增加 (JY61P 倒扣: 模块顶面朝下,
                 *   从车顶看左转 = 从模块顶面看顺时针 = 维特标准 yaw+)
                 * T 命令 (g_one_shot_turn=1) 优先级最高: 任意模式下执行单次转弯后停车 */
                extern uint8_t g_square_state;   /* 0=直行 1=转弯 2=完成 3=刹车 */
                extern uint8_t g_square_edge;
                extern float   g_square_yaw_base;
                extern uint8_t g_one_shot_turn;
                extern float   g_one_shot_angle;
                extern uint32_t g_brake_start;
                #define SQ_EDGE_LEN    1500.0f   /* 边长 mm */
                #define SQ_TURN_THRESH 5.0f      /* 转弯到位阈值 (度) */
                #define SQ_BASE_SPD    80        /* 直行基础速度 (慢速验证) */
                #define SQ_TURN_SPD    60        /* 原地转弯速度 (慢, 防过冲) */
                #define SQ_YAW_KP       15.0f    /* 直行 yaw 修正 P (驱动板单位/度) */
                #define SQ_YAW_CLIP     40        /* 直行修正最大差速 (限幅) */
                #define SQ_BRAKE_MS     200       /* 转弯到位后刹车时间 (ms) */

                /* IMU 离线保护: yaw 恒 0 会导致修正乱打方向, 直接停车 */
                if (!g_imu_present) {
                    g_motor_l_speed = 0;
                    g_motor_r_speed = 0;
                    if (g_running) {
                        g_running = 0;
                        Motor_Stop();
                        CMD_SendText("[MSPM0] SQUARE: IMU offline, stop\n");
                    }
                } else {
                    float yaw_now = IMU_Get_Yaw_Cached();
                    g_yaw_now = yaw_now;

                    if (g_one_shot_turn) {
                        /* ── T 命令单次转弯 (任意角度) ──
                         * target = g_one_shot_angle (正=左转 yaw+, 负=右转 yaw-)
                         * delta = yaw_now - g_turn_start_yaw (相对变化幅度)
                         * err = target - delta, 正=还需左转, 负=还需右转 */
                        float target = g_one_shot_angle;
                        float delta = yaw_now - g_turn_start_yaw;
                        if (delta > 180.0f)  delta -= 360.0f;
                        if (delta < -180.0f) delta += 360.0f;
                        float err = target - delta;
                        g_yaw_err = err;
                        if (err > SQ_TURN_THRESH) {
                            g_motor_l_speed = -SQ_TURN_SPD;   /* 左转: 左轮后+右轮前 */
                            g_motor_r_speed =  SQ_TURN_SPD;
                        } else if (err < -SQ_TURN_THRESH) {
                            g_motor_l_speed =  SQ_TURN_SPD;   /* 右转: 左轮前+右轮后 */
                            g_motor_r_speed = -SQ_TURN_SPD;
                        } else {
                            /* 到位, 停车 (单次转弯不进直行) */
                            g_one_shot_turn = 0;
                            g_running = 0;
                            Motor_Stop();
                            CMD_SendText("[MSPM0] TURN done\n");
                        }
                    } else if (g_square_state == 2) {
                        /* 已完成 4 条边, 停车 */
                        g_motor_l_speed = 0;
                        g_motor_r_speed = 0;
                        g_running = 0;
                        Motor_Stop();
                        CMD_SendText("[MSPM0] SQUARE DONE! auto-stop\n");
                    } else if (g_square_state == 3) {
                        /* 刹车: 停车 SQ_BRAKE_MS 等惯性消除, 再进下一条边
                         * 转弯到位后立即切直行会有转弯惯性, 第一段直行开局就偏 */
                        g_motor_l_speed = 0;
                        g_motor_r_speed = 0;
                        if ((uint32_t)(g_sys_tick - g_brake_start) >= SQ_BRAKE_MS) {
                            g_square_yaw_base = IMU_Get_Yaw_Cached();  /* 锁新朝向 */
                            Odom_Reset_Edge();
                            g_square_edge++;
                            if (g_square_edge >= 4) {
                                g_square_state = 2;   /* 4 条边走完 */
                                CMD_SendText("[MSPM0] SQUARE: all edges done\n");
                            } else {
                                g_square_state = 0;   /* 进下一条边直行 */
                                CMD_SendText("[MSPM0] SQUARE: next edge\n");
                            }
                        }
                    } else if (g_square_state == 0) {
                        /* 直行阶段: 走直线, 用 yaw 误差做差速保持直行
                         * err = base - yaw_now, 正=yaw偏小(车头偏右), 需左转修正
                         * (倒扣: 车右转 yaw减, err=base-yaw>0 → 左转修正 yaw+ → 回正) */
                        float err = g_square_yaw_base - yaw_now;
                        if (err > 180.0f)  err -= 360.0f;
                        if (err < -180.0f) err += 360.0f;
                        g_yaw_err = err;
                        float correction = err * SQ_YAW_KP;
                        if (correction > SQ_YAW_CLIP)  correction = SQ_YAW_CLIP;
                        if (correction < -SQ_YAW_CLIP) correction = -SQ_YAW_CLIP;
                        g_motor_l_speed = (int16_t)(SQ_BASE_SPD - correction);
                        g_motor_r_speed = (int16_t)(SQ_BASE_SPD + correction);
                        /* 判断是否走完一条边 */
                        if (Odom_Get_Edge_Dist() >= SQ_EDGE_LEN) {
                            g_square_state = 1;
                            g_turn_start_yaw = yaw_now;   /* 记转弯起点, 用相对幅度判定 */
                            CMD_SendText("[MSPM0] SQUARE: edge done, turning\n");
                        }
                    } else {
                        /* state == 1 转弯阶段: 原地左转 90°
                         * delta = yaw_now - turn_start_yaw, 目标 delta=90° (左转 yaw+)
                         * err = 90 - delta, 正=还需左转, 负=过冲需右转修正 */
                        float delta = yaw_now - g_turn_start_yaw;
                        if (delta > 180.0f)  delta -= 360.0f;
                        if (delta < -180.0f) delta += 360.0f;
                        float err = 90.0f - delta;
                        g_yaw_err = err;
                        if (err > SQ_TURN_THRESH) {
                            g_motor_l_speed = -SQ_TURN_SPD;   /* 左转 */
                            g_motor_r_speed =  SQ_TURN_SPD;
                        } else if (err < -SQ_TURN_THRESH) {
                            g_motor_l_speed =  SQ_TURN_SPD;   /* 过冲, 右转修正 */
                            g_motor_r_speed = -SQ_TURN_SPD;
                        } else {
                            /* 到位, 进刹车 (不直接切直行, 防转弯惯性带偏下一条边) */
                            g_square_state = 3;
                            g_brake_start = g_sys_tick;
                            CMD_SendText("[MSPM0] SQUARE: turn done, braking\n");
                        }
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
