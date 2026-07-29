/**
 * @file    cmd.c
 * @brief   UART 命令解析 (精简版, H题)
 *          从 ESP32 接收并修改全局参数, 远程调参/查状态/切题
 */

#include "cmd.h"
#include "ti_msp_dl_config.h"
#include "track.h"
#include "motor.h"
#include "imu.h"
#include "soft_uart.h"
#include "odometry.h"
#include "oled.h"
#include "k230.h"
#include "task.h"
#include "vision_protocol.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#define CMD_BUF_SIZE 32

static char  cmd_buf[CMD_BUF_SIZE];
static uint8_t cmd_idx = 0;

/* ─── 文本回传: 通过软件串口发文本给 ESP32 → 电脑 ─── */
void CMD_SendText(const char *text)
{
    SoftUART_SendString(text);
}

void CMD_Init(void)
{
    cmd_idx = 0;
    /* 软件串口缓冲在 SoftUART_Init 已清, 无需清FIFO */
}

/**
 * @brief 简易 atof
 */
static float my_atof(const char *s)
{
    float val = 0, frac = 0;
    uint8_t dot = 0;
    int sign = 1;
    if (*s == '-') { sign = -1; s++; }
    while (*s) {
        if (*s == '.') { dot = 1; s++; continue; }
        if (*s < '0' || *s > '9') break;
        if (dot) { frac = frac * 10 + (*s - '0'); dot *= 10; }
        else val = val * 10 + (*s - '0');
        s++;
    }
    if (dot > 1) val += frac / dot;
    return sign * val;
}

/* 视觉协议实例 (定义在 main.c) */
extern Vision_ProtocolTypeDef g_vision;

/**
 * @brief 执行命令
 */
static void CMD_Exec(void)
{
    static char ack[256];   /* static 放 BSS, 避免栈溢出 */

    if (cmd_idx == 0) return;
    cmd_buf[cmd_idx] = '\0';

    char c = cmd_buf[0];
    float v = my_atof(cmd_buf + 1);

    switch (c) {
    case 'b':
        g_track_cfg.base_speed = v;
        snprintf(ack, sizeof(ack), "[MSPM0] base_speed=%.1f\n", g_track_cfg.base_speed);
        CMD_SendText(ack);
        break;
    case 'p':
        g_track_cfg.turn_p = v;
        snprintf(ack, sizeof(ack), "[MSPM0] turn_p=%.2f\n", g_track_cfg.turn_p);
        CMD_SendText(ack);
        break;
    case 'd':
        g_track_cfg.turn_d = v;
        snprintf(ack, sizeof(ack), "[MSPM0] turn_d=%.2f\n", g_track_cfg.turn_d);
        CMD_SendText(ack);
        break;

    case 's':
        g_running = 0;
        Motor_Stop();
        CMD_SendText("[MSPM0] STOPPED\n");
        break;
    case 'g':
        g_running = 1;
        Task_OnStart();
        CMD_SendText("[MSPM0] RUNNING\n");
        break;

    case 'N':   /* 切换题目号 1~5: N=下一题, N3=第3题 */
        if (cmd_idx == 1) {
            g_task_id = (g_task_id >= 5) ? 1 : (g_task_id + 1);
        } else {
            if (v < 1) v = 1;
            if (v > 5) v = 5;
            g_task_id = (uint8_t)v;
        }
        Task_ApplyTaskId();
        K230_SendTask(g_task_id);
        if (g_oled_present) {
            OLED_ClearArea(0, 0, 16);
            OLED_PrintfAt(0, 0, "Task: %d", g_task_id);
            OLED_PrintfAt(1, 0, "Task: %d %s", g_task_id, g_running ? "RUN " : "STOP");
        }
        snprintf(ack, sizeof(ack), "[MSPM0] task=%d\n", g_task_id);
        CMD_SendText(ack);
        break;

    case '?':   /* 查询状态 */
        snprintf(ack, sizeof(ack),
            "[MSPM0] ? task=%d run=%d base=%.1f p=%.2f d=%.2f\n"
            "       yaw=%.2f dist=%.1fmm corner=%d\n"
            "       imu=%d oled=%d vision=%d x=%d y=%d\n",
            g_task_id, g_running, g_track_cfg.base_speed, g_track_cfg.turn_p, g_track_cfg.turn_d,
            g_imu_data.Yaw, Odom_Get_Total_Dist(), g_corner_count,
            g_imu_data.update_flag, g_oled_present, g_vision.target_found ? 1 : 0,
            g_vision.target_x, g_vision.target_y);
        CMD_SendText(ack);
        break;

    case 'B':   /* 启动状态 */
        snprintf(ack, sizeof(ack),
            "[MSPM0] === Boot ===\n"
            "       track: OK (arc PD)\n"
            "       motor: OK\n"
            "       imu:   %s (update=%d)\n"
            "       oled:  %s (present=%d)\n"
            "       vision: OK\n"
            "       balance: OK\n",
            g_imu_data.update_flag ? "OK" : "FAIL", g_imu_data.update_flag,
            g_oled_present ? "OK" : "FAIL", g_oled_present);
        CMD_SendText(ack);
        break;

    default:
        break;
    }
}

/**
 * @brief 主循环轮询: 从软件串口环形缓冲读字节, 攒够一行执行
 *        非阻塞, 每次最多读 16 字节
 */
void CMD_Poll(void)
{
    uint8_t budget = 16;
    while (budget--) {
        int16_t b = SoftUART_ReadByte();
        if (b < 0) break;   /* 缓冲空 */
        char c = (char)b;
        if (c == '\n' || c == '\r') {
            CMD_Exec();
            cmd_idx = 0;
        } else if (cmd_idx < CMD_BUF_SIZE - 1) {
            cmd_buf[cmd_idx++] = c;
        } else {
            cmd_idx = 0;   /* 溢出丢弃 */
        }
    }
}
