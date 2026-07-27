/**
 * @file    empty.c
 * @brief   巡线小车主程序 — 四路驱动板方案
 * 
 * 架构:
 *   MSPM0 只负责:
 *     1. 灰度巡线 → Track_Loop() 计算左右速度
 *     2. 发送 $spd 指令给四路驱动板 (自带PID)
 *     3. 接收 ESP32 转发的调参命令
 * 
 * 初始化流程:
 *   1. SysConfig + 3秒延时
 *   2. 配置驱动板参数 (电机类型/减速比/磁环线/轮径)
 *   3. 进入主循环: 巡线 + 发速度 + 收命令
 * 
 * 命令 (通过 WiFi→ESP32→UART):
 *   m0      巡线模式
 *   m1      空转模式
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
#include "BSP/template/uart_debug.h"
#include "BSP/imu.h"
#include "BSP/odometry.h"
#include "BSP/oled.h"
#include "BSP/button.h"
#include "delay.h"

/* ─── 驱板初始化指令序列 ─── */
static void Driver_Board_Init(void)
{
    /* I2C 方案: 驱动板参数通过 Motor_Init() 设置, 这里不需要额外操作 */
    /* 如需改 PID, 通过串口发 $MPID:kp,ki,kd# */
    (void)0;
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

int main(void)
{
    SYSCFG_DL_init();
    /* 延时 8 秒等 ESP32 WiFi 连上 PC, 否则启动信息发出去时 PC 还没连, 收不到 */
    delay_ms(8000);

    /* 初始化模块 */
    CMD_SendText("[MSPM0] init: track\n");
    Track_Init();
    CMD_SendText("[MSPM0] init: cmd\n");
    CMD_Init();

    /* 配置驱动板 */
    CMD_SendText("[MSPM0] init: motor\n");
    Motor_Init();
    Driver_Board_Init();

    /* ── IMU 初始化 (串口 UART_DEBUG, PA10/PA11, 9600bps) ──
     * IMU 和 OLED 物理隔离, 不再互斥, 可同时工作 */
    CMD_SendText("[MSPM0] init: imu (UART_DEBUG 9600bps)\n");
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

    /* 巡线在主循环中轮询执行
     * imu_tick 用作 IMU 降频计数 (每5次=50ms 解析一次) */
    uint8_t imu_tick = 0;

    while (1) {
        /* ── UART_DEBUG RX 搬运: 每周期都调, 把 JY61P 串口字节搬到环形缓冲区
         *    必须在 IMU_Poll 之前调, 否则 IMU_Poll 取不到最新数据
         *    g_uart_debug_echo=1 时 PollRx 内部会自动把字节回显到 PC */
        UART_Debug_PollRx();

        /* ── IMU 解析: 降频到每 50ms 一次, 仅当 IMU 接了且启用才解析 ──
         *    IMU_Poll 内部从环形缓冲区取字节, 状态机解析 0x55 0x53 帧 */
        if (g_use_imu && (++imu_tick >= 5)) {
            imu_tick = 0;
            IMU_Poll();
        }

        /* ── 按钮轮询: 纯 GPIO 读取, 开销极小, 每 10ms 一次 ── */
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
                /* 切换 IMU 解析开关: 取反 g_use_imu, switch_mode 内会发通知给电脑
                 * (IMU 改串口后不再与 OLED 互斥, 此按钮只切 IMU 解析) */
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

        /* ── 运行/停止控制 ── */
        if (!g_running) {
            g_motor_l_speed = 0;
            g_motor_r_speed = 0;
        } else if (g_mode == 0) {
            Track_Loop();
        } else if (g_mode == 3) {
            /* ── 不倒翁模式 (IMU yaw 自稳): 手转车自动回正到切 m3 时的朝向 ──
             * 切 m3 时在 cmd.c 的 case 'm' 中锁目标, 这里只做 PID 修正 */
            float yaw_now = IMU_Get_Yaw_Cached();
            float err = g_yaw_target - yaw_now;
            /* 角度回绕: 误差超过 ±180° 时取最短路径 */
            if (err > 180.0f)  err -= 360.0f;
            if (err < -180.0f) err += 360.0f;
            g_yaw_now = yaw_now;
            g_yaw_err = err;
            /* 简单 P 控制: P=30 驱动板单位/度, ±400 限幅 */
            float base = 0.0f;
            float correction = err * 30.0f;
            if (correction > 400.0f)  correction = 400.0f;
            if (correction < -400.0f) correction = -400.0f;
            g_motor_l_speed = (int16_t)(base - correction);
            g_motor_r_speed = (int16_t)(base + correction);
        } else {
            g_motor_l_speed = (int16_t)g_target_rpm;
            g_motor_r_speed = (int16_t)g_target_rpm;
        }

        /* ── 发送速度指令给驱动板 ── */
        Motor_Send_Speed(0, -g_motor_r_speed, 0, -g_motor_l_speed);

        /* ── 接收调参命令 ── */
        CMD_Poll();

        /* ── 第二轮 IMU RX 搬运: 避免 4 字节 FIFO 在 10ms 内溢出 ──
         * 9600bps 每 10ms 到 ~10 字节, MSPM0 UART FIFO 仅 4 字节,
         * 单次 PollRx 每周期会丢 ~6 字节。连续两次 = 每 5ms 等效, 不溢出。 */
        UART_Debug_PollRx();

        /* ── 控制刷新率 ~10ms ── */
        delay_ms(10);
    }
}
