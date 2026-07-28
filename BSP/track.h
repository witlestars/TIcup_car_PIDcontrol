/**
 * @file    track.h
 * @brief   巡线控制 — 灰度离心PD → 目标速度 → 驱动板
 * 
 * 架构: 单级 (离心PD → 速度目标)
 *   离心 centroid → 差速 turn → g_motor_l/r_speed
 *   main loop 统一发 $spd 指令给四路驱动板
 * 
 * 速度单位: 驱动板 -1000~1000 (非RPM)
 * 
 * 丢线处理:
 *   断线时保持最后有效方向 TRACK_LOST_HOLD 个周期(250ms)
 */

#ifndef __TRACK_H
#define __TRACK_H
#include <stdint.h>

#define TRACK_BASE_SPEED     200     /* 基础速度 (驱动板单位) */
#define TRACK_TURN_GAIN_P    18.0f   /* 离心 P: 转向强度 */
#define TRACK_TURN_GAIN_D     1.5f   /* 离心 D: 压低防反打 */
#define TRACK_LOST_HOLD        25    /* 丢线保持周期 (25×10ms=250ms) */
#define TRACK_PIVOT_TIMEOUT   0      /* 原地转弯超时 (0=不限时, 找到中间传感器才停) */

/* 运行时调参变量 (由 cmd.c 通过 UART 命令修改) */
typedef struct {
    float base_speed;    /* 基础速度 (驱动板单位) */
    float turn_p;        /* 离心 P */
    float turn_d;        /* 离心 D */
    float pivot_speed;   /* 原地转弯速度 (驱动板单位, 默认200) */
    float corner_fwd_ms; /* 直角弯前冲时间 (ms, 默认500, 命令f调节) */
} track_cfg_t;
extern track_cfg_t g_track_cfg;

extern float g_base_rpm;      /* 兼容旧命令, 实际映射到 g_track_cfg.base_speed */
extern uint8_t g_track_locked; /* 1=超时锁定停车, 需 'g' 命令复位 */
extern float g_speed_kp;       /* 不再使用, 保留兼容 */
extern float g_speed_ki;
extern float g_speed_kd;

/* 调试变量 (每帧更新, 供 cmd.c 发送) */
typedef struct {
    float   centroid;    /* 当前重心 -3.5~+3.5 */
    uint8_t active_cnt;  /* 激活传感器数量 0-8 */
    uint8_t active_bits; /* 激活传感器位图 (bit0=CH0 ... bit7=CH7) */
    uint8_t state;       /* 状态机 0=NORMAL 1=CORNER_FWD 2=CORNER_TURN 3=LOCKED */
    int16_t corner_dir;  /* 转弯方向 +1=右 -1=左 */
    float   turn;        /* PD输出 */
} track_dbg_t;
extern track_dbg_t g_dbg;

/* ─── IMU 辅助转弯 (避免转过头) ─── */
extern uint8_t g_imu_assist;       /* 1=启用IMU闭环转弯, 0=纯灰度 (默认1) */
extern float   g_yaw_target;       /* 当前转弯目标yaw (度) */
extern float   g_yaw_err;          /* 当前yaw误差 (度, 带符号) */
extern float   g_yaw_now;          /* 当前yaw (度, 缓存) */
extern uint8_t g_corner_done_by;   /* 转弯退出原因: 0=未退出 1=灰度找到线 2=yaw到位 3=超时 */

/* ─── 圈数控制 (过4个直角弯=1圈) ─── */
extern uint8_t g_target_laps;      /* 目标圈数 (按钮调, 默认1) */
extern uint8_t g_current_lap;      /* 当前已完成圈数 */
extern uint8_t g_corner_count;     /* 当前圈已过直角弯数 (0-4) */
extern uint8_t g_laps_done;        /* 1=跑完目标圈数, 自动停车 (需按START复位) */

void Track_Init(void);
void Track_Reset(void);              /* 复位巡线状态 + 圈数 */
void Track_Loop(void);               /* TIMA0 ISR 每10ms调用 */

#endif
