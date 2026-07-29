/**
 * @file    track.h
 * @brief   灰度巡线控制 — 离心PD 弧线跟线 (适配槽口型路线)
 *
 * 架构: 纯灰度离心PD, 直道+半圆弯统一弧线跟线不停车
 *   离心 centroid → 差速 turn → g_motor.l/r
 *   半圆弯时 centroid 持续偏一边, PD 自然产生持续差速, 形成弧线
 *
 * 速度单位: 驱动板 -1000~1000 (非RPM)
 *
 * 丢线处理:
 *   断线时保持最后有效方向 TRACK_LOST_HOLD 个周期(250ms)
 */

#ifndef __TRACK_H
#define __TRACK_H
#include <stdint.h>

#define TRACK_LOST_HOLD        25    /* 丢线保持周期 (25×10ms=250ms) */

/* 运行时调参变量 */
typedef struct {
    float base_speed;    /* 基础速度 (驱动板单位) */
    float turn_p;        /* 离心 P */
    float turn_d;        /* 离心 D */
    float pivot_speed;   /* 保留 (槽口型不停车不用) */
    float corner_fwd_ms; /* 保留 (槽口型不停车不用) */
} track_cfg_t;
extern track_cfg_t g_track_cfg;

extern uint8_t g_track_locked; /* 保留接口 (槽口型不触发) */

/* 调试变量 */
typedef struct {
    float   centroid;    /* 当前重心 -3.5~+3.5 */
    uint8_t active_cnt;  /* 激活传感器数量 0-8 */
    uint8_t active_bits; /* 激活传感器位图 (bit0=CH0 ... bit7=CH7) */
    uint8_t state;       /* 0=直道 1=弯道中 */
    int16_t corner_dir;  /* 保留 */
    float   turn;        /* PD输出 */
} track_dbg_t;
extern track_dbg_t g_dbg;

/* ─── IMU 辅助 (保留接口, 槽口型默认关闭) ─── */
extern uint8_t g_imu_assist;
extern float   g_yaw_target;
extern float   g_yaw_err;
extern float   g_yaw_now;
extern uint8_t g_corner_done_by;

/* ─── 过弯计数 (task.c 判断一圈用, 半圆弯+1) ─── */
extern uint8_t g_corner_count;

void Track_Init(void);
void Track_Reset(void);              /* 复位巡线状态 + 过弯计数 */
void Track_Loop(void);               /* 主循环10ms调用: 灰度PD弧线跟线 */

#endif
