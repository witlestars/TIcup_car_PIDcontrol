/**
 * @file    track.c
 * @brief   灰度巡线控制 — 离心PD 弧线跟线 (适配槽口型路线)
 *
 * 路线: 槽口型 (两条直道 + 两端半圆弯)
 * 算法: 纯灰度离心PD, 直道和半圆弯统一处理, 弧线跟线不停车
 *
 * 原理:
 *   灰度传感器读重心 centroid (-3.5~+3.5, 0=中央)
 *   centroid > 0 → 线偏右 → 右转找线 (左轮快, 右轮慢)
 *   centroid < 0 → 线偏左 → 左转找线 (左轮慢, 右轮快)
 *   PD = P*centroid + D*(centroid - last_centroid)
 *   半圆弯时 centroid 持续偏一边, PD 自然产生持续差速, 形成弧线
 *
 * 速度单位: 驱动板 -1000~1000 (非RPM)
 *
 * 丢线处理:
 *   断线时保持最后有效方向 TRACK_LOST_HOLD 个周期(250ms)
 */

#include "track.h"
#include "grey.h"
#include "motor.h"
#include "odometry.h"

/* ─── 运行时调参变量 ─── */
track_cfg_t g_track_cfg = {
    300,      /* base_speed 基础速度 (驱动板单位, 200≈慢速巡线) */      //  230         280
    18.5f,    /* turn_p 离心 P: 转向强度 (加大压震荡) */                //  18.5
    50.0f,     /* turn_d 离心 D: 压低防反打 (加大压震荡) */              //  44.02
};

/* ─── 超时锁定标志 (保留接口, 槽口型不触发) ─── */
uint8_t g_track_locked = 0;

/* ─── 调试变量 ─── */
track_dbg_t g_dbg = {0};

/* ─── IMU 辅助参数 (保留接口, 槽口型纯灰度PD不强制用) ─── */
uint8_t g_imu_assist      = 0;     /* 槽口型默认关闭, 纯灰度PD足够 */
float   g_yaw_target      = 0.0f;
float   g_yaw_err         = 0.0f;
float   g_yaw_now         = 0.0f;
uint8_t g_corner_done_by  = 0;

/* ─── 过弯计数 (task.c 判断一圈用) ─── */
uint8_t g_corner_count  = 0;

/* ─── 速度限幅 ─── */
#define SPEED_MAX   500
#define SPEED_MIN  -500

/* ─── 内部状态 ─── */
static float last_centroid = 0.0f;
static float last_valid    = 0.0f;
static uint8_t lost_cnt    = 0;

/* ─── 半圆弯检测 (用于过弯计数, 不停车)
 * 槽口型半圆弯特点: 灰度持续偏一边 (centroid 绝对值持续 > 阈值)
 * 检测进入半圆弯: centroid 同号持续 N 帧
 * 检测离开半圆弯: centroid 回到接近 0 ─── */
#define CURVE_ENTER_TH    2.5f    /* centroid 绝对值超过此值认为进入弯道 */
#define CURVE_LEAVE_TH    0.5f    /* centroid 绝对值低于此值认为离开弯道 */
#define CURVE_HOLD_TICKS  10      /* 持续 10 帧 (100ms) 才确认进弯 */
static uint8_t curve_in_curve = 0;  /* 1=当前在弯道中 */
static uint8_t curve_hold_cnt = 0;  /* 持续偏一边的帧数 */
static int8_t  curve_last_sign = 0; /* 上一次 centroid 符号 (+1/-1) */

/**
 * @brief 巡线初始化
 */
void Track_Init(void)
{
    last_centroid = 0.0f;
    last_valid    = 0.0f;
    lost_cnt      = 0;
    curve_in_curve = 0;
    curve_hold_cnt = 0;
    curve_last_sign = 0;

    Odom_Init(); // 初始化里程计 (编码器+IMU)
    Motor_Init(); // 初始化电机 (I2C 配置驱动板参数)
}

/**
 * @brief 复位巡线状态 + 过弯计数 (按 START 启动时调用)
 */
void Track_Reset(void)
{
    last_centroid = 0.0f;
    last_valid    = 0.0f;
    lost_cnt      = 0;
    g_corner_count = 0;
    g_track_locked = 0;
    curve_in_curve = 0;
    curve_hold_cnt = 0;
    curve_last_sign = 0;
}

/**
 * @brief 巡线主控 (主循环每10ms调用)
 *        纯灰度离心PD, 直道+半圆弯统一弧线跟线, 不停车
 *        内含半圆弯检测更新 g_corner_count (供 task.c 判断一圈)
 */
void Track_Loop(void)
{
    Grey_State_t grey_state;
    float centroid, centroid_d, turn;

    /* ─── 1. 读灰度 ─── */
    Grey_Read(&grey_state);

    /* 统计活跃通道数 (调试用) */
    uint8_t i, active_cnt = 0, active_bits = 0;
    for (i = 0; i < GREY_CHANNEL_COUNT; i++) {
        if (grey_state.active[i]) {
            active_cnt++;
            active_bits |= (1 << i);
        }
    }
    g_dbg.active_cnt  = active_cnt;
    g_dbg.active_bits = active_bits;

    /* ─── 2. 取重心 ─── */
    if (grey_state.valid) {
        centroid = grey_state.centroid;
        last_valid = centroid;
        lost_cnt = 0;
    } else {
        /* 丢线: 用最后有效方向, 给最大离心值激进转向找回线 */
        if (lost_cnt < TRACK_LOST_HOLD) {
            // if (last_valid > 0.01f)
            //     centroid = 3.5f;
            // else if (last_valid < -0.01f)
            //     centroid = -3.5f;
            // else
            //     centroid = 0.0f;
            // lost_cnt++;
        } else {
            centroid = 0.0f;
        }
    }

    /* ─── 3. 半圆弯检测 (更新 g_corner_count, 不停车) ─── */
    {
        float abs_c = (centroid >= 0) ? centroid : -centroid;
        int8_t sign = (centroid > 0.01f) ? 1 : (centroid < -0.01f ? -1 : 0);

        if (!curve_in_curve) {
            /* 不在弯道: 检测是否进弯 */
            if (abs_c > CURVE_ENTER_TH && sign != 0) {
                if (sign == curve_last_sign) {
                    curve_hold_cnt++;
                } else {
                    curve_hold_cnt = 1;
                    curve_last_sign = sign;
                }
                if (curve_hold_cnt >= CURVE_HOLD_TICKS) {
                    curve_in_curve = 1;
                    g_corner_count++;   /* 进弯计数+1 */
                    g_dbg.state = 1;    /* 1=弯道中 */
                }
            } else {
                curve_hold_cnt = 0;
                g_dbg.state = 0;        /* 0=直道 */
            }
        } else {
            /* 在弯道: 检测是否出弯 */
            if (abs_c < CURVE_LEAVE_TH) {
                curve_in_curve = 0;
                curve_hold_cnt = 0;
                /* 过弯后清里程计当前边距离 (供 task.c 测距用) */
                Odom_Reset_Edge();
                g_dbg.state = 0;        /* 回到直道 */
            }
        }
    }

    /* ─── 4. 离心PD ─── */
    centroid_d = centroid - last_centroid;
    last_centroid = centroid;

    turn = centroid * g_track_cfg.turn_p + centroid_d * g_track_cfg.turn_d;

    g_dbg.centroid = centroid;
    g_dbg.turn     = turn;

    /* ─── 5. 速度目标 + 限幅 ─── */
    {
        float base = (lost_cnt > 0) ? 100.0f : g_track_cfg.base_speed;
        float spd_l = base + turn;    /* 线偏右 → 左轮快 → 右转找线 */
        float spd_r = base - turn;

        if (spd_l > SPEED_MAX) spd_l = SPEED_MAX;
        if (spd_l < SPEED_MIN) spd_l = SPEED_MIN;
        if (spd_r > SPEED_MAX) spd_r = SPEED_MAX;
        if (spd_r < SPEED_MIN) spd_r = SPEED_MIN;

        g_motor.l = (int16_t)spd_l;
        g_motor.r = (int16_t)spd_r;
    }

    /* ─── 6. 下发速度到驱动板 (M2=右轮, M4=左轮, M1/M3=0) ─── */
    Motor_Send_Speed(0, -g_motor.r, 0, -g_motor.l);
}
