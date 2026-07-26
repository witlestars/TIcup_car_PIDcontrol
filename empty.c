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

/* ─── IMU↔OLED 互斥切换 (共用 PA17/PA15 软件 I2C) ───
 * 按钮和串口 'E' 命令都走这个函数, 保证切换逻辑一致
 * 切换时先停用另一方 (g_xxx_present=0, 后续I2C操作自动跳过),
 * 再调用启用方的 Init 重新探测总线, 最后发文本通知电脑
 * use_imu: 1=启用IMU(停用OLED), 0=启用OLED(停用IMU) */
void switch_mode(uint8_t use_imu)
{
    if (use_imu) {
        /* 切到 IMU: 先关 OLED 显示让屏幕黑屏
         * 必须在 g_use_imu 改之前调 (oled_cmd 内部检查 g_use_imu) */
        OLED_PowerOff();   /* 发 0xAE, 屏幕立即黑屏 */
        /* 设置互斥标志 (imu.c/oled.c 内的 I2C 入口都检查这个标志) */
        g_use_imu = 1;
        /* 停 OLED: 清标志, 后续 oled_write 直接 return */
        g_oled_present = 0;
        /* 启 IMU: IMU_Init 内会重新探测 JY61P */
        CMD_SendText("[MSPM0] switch -> IMU (OLED off)\n");
        if (IMU_Init() == 0) {
            CMD_SendText("[MSPM0] IMU JY61P OK\n");
        } else {
            CMD_SendText("[MSPM0] IMU JY61P FAIL (skipped)\n");
        }
    } else {
        /* 切到 OLED: 设置互斥标志, IMU 侧 I2C 入口自动跳过 */
        g_use_imu = 0;
        /* 停 IMU: 清标志, 后续 IMU_Read_RPY 直接 return 缓存值 */
        g_imu_present = 0;
        /* 启 OLED: OLED_Init 内会重新探测 SSD1306 并发 0xAF 点亮 */
        CMD_SendText("[MSPM0] switch -> OLED (IMU off)\n");
        OLED_Init();
        if (g_oled_present) {
            OLED_Clear();
            OLED_PrintfAt(0, 0, "==TI CUP==");
            OLED_PrintfAt(1, 0, "OLED mode");
            CMD_SendText("[MSPM0] OLED OK\n");
        } else {
            CMD_SendText("[MSPM0] OLED FAIL (skipped)\n");
        }
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

    /* ── IMU 初始化 (默认 g_use_imu=1, IMU 占用 PA17/PA15 总线) ──
     * OLED/IMU 共用同一组软件 I2C, 上电默认走 IMU 模式 (巡线需要 IMU)
     * 后续可用 BTN_MODE 按钮 或 串口 'E' 命令切换到 OLED 模式 */
    CMD_SendText("[MSPM0] init: imu (default mode)\n");
    if (IMU_Init() == 0) {
        CMD_SendText("[MSPM0] IMU JY61P OK\n");
    } else {
        CMD_SendText("[MSPM0] IMU JY61P FAIL (skipped)\n");
    }

    CMD_SendText("[MSPM0] init: odom\n");
    Odom_Init();

    /* ── OLED 初始化 (g_use_imu=1 时 OLED_Init 直接跳过, 不抢 I2C) ──
     * 想用 OLED 时按 MODE 按钮或发 'E0' 切换, switch_mode 会调 OLED_Init */
    CMD_SendText("[MSPM0] init: oled (skipped, IMU mode)\n");
    OLED_Init();   /* g_use_imu=1, 内部直接 return, g_oled_present 保持 0 */

    /* ── 按钮初始化: 纯 GPIO, 无 I2C ── */
    CMD_SendText("[MSPM0] init: button\n");
    Button_Init();
    CMD_SendText("[MSPM0] init done\n");

    /* 巡线在主循环中轮询执行
     * imu_tick 用作 IMU 降频计数 (每5次=50ms 读一次 IMU) */
    uint8_t imu_tick = 0;

    while (1) {
        /* ── IMU 轮询: 降频到每 50ms 一次, 仅当 IMU 接了才读 ── */
        if (g_imu_present && (++imu_tick >= 5)) {
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
                /* 切换 IMU↔OLED: 取反 g_use_imu, switch_mode 内会发通知给电脑 */
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
        } else {
            g_motor_l_speed = (int16_t)g_target_rpm;
            g_motor_r_speed = (int16_t)g_target_rpm;
        }

        /* ── 发送速度指令给驱动板 ── */
        Motor_Send_Speed(0, -g_motor_r_speed, 0, -g_motor_l_speed);

        /* ── 接收调参命令 ── */
        CMD_Poll();

        /* ── 控制刷新率 ~10ms ── */
        delay_ms(10);
    }
}
