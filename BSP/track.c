/**
 * @file    track.c
 * @brief   巡线控制 — 灰度离心PD + 直角弯三段状态机
 *
 * 状态机:
 *   NORMAL     灰度离心PD巡线
 *              ↓ 检测到直角 (active_cnt>=5 + 有明显方向)
 *   CORNER_FWD 前冲一段可调时间 (车此时无线, 靠惯性过弯角)
 *              ↓ 前冲计时到
 *   CORNER_TURN 停车原地转 (左右轮反向), 找到线回中央或超时
 *              ↓ 找到线 (1-2路激活且|centroid|<1.0)
 *   NORMAL
 *              ↓ 超时 (TRACK_PIVOT_TIMEOUT)
 *   锁定停车 (等 'g' 复位)
 *
 * 速度单位: 驱动板 -1000~1000 (非RPM)
 *
 * 方向约定 (重点!):
 *   centroid > 0  → 线偏右 → 车需要右转找线
 *   centroid < 0  → 线偏左 → 车需要左转找线
 *   右转: 左轮正, 右轮负
 *   左转: 左轮负, 右轮正
 */

#include "track.h"
#include "grey.h"
#include "motor.h"
#include "imu.h"
#include "odometry.h"

/* ─── 运行时调参变量 — 可通过 UART 命令修改 ─── */
track_cfg_t g_track_cfg = {
    200,      /* base_speed 基础速度 (驱动板单位, 200≈慢速巡线) */
    20.0f,    /* turn_p 离心 P: 转向强度 (加大压震荡) */
    4.0f,     /* turn_d 离心 D: 压低防反打 (加大压震荡) */
    100.0f,   /* pivot_speed 原地转弯速度 (慢转防过头) */
    500.0f,   /* corner_fwd_ms 直角弯前冲时间 (ms, 固定500) */
};

/* ─── 超时锁定标志 (1=原地转弯超时停车, 需 'g' 复位) ─── */
uint8_t g_track_locked = 0;

/* ─── 调试变量 (每帧更新, 供 cmd.c 发送) ─── */
track_dbg_t g_dbg = {0};

/* ─── IMU 辅助转弯参数 (可调) ─── */
uint8_t g_imu_assist      = 1;     /* 1=启用IMU闭环转弯, 0=纯灰度 */
float   g_yaw_target      = 0.0f;  /* 转弯目标yaw (度) */
float   g_yaw_err         = 0.0f;  /* yaw误差 (度) */
float   g_yaw_now         = 0.0f;  /* 当前yaw缓存 */
uint8_t g_corner_done_by  = 0;     /* 转弯退出原因 */

#define YAW_CORNER_ANGLE  90.0f    /* 直角弯目标角度 (度) */
#define YAW_SLOW_BAND     15.0f    /* 进入此带减速 (度) */
#define YAW_ARRIVE_TH     3.0f     /* 到达阈值 (度, 小于此判为到位) */

/* ─── 圈数控制 ─── */
uint8_t g_target_laps   = 1;       /* 目标圈数 (按钮可调) */
uint8_t g_current_lap   = 0;       /* 已完成圈数 */
uint8_t g_corner_count  = 0;       /* 当前圈已过弯数 */
uint8_t g_laps_done     = 0;       /* 1=跑完, 自动停车 */

/* ─── 旧变量保留 (cmd.c 兼容) ─── */
float g_base_rpm  = 200;
float g_speed_kp  = 0;
float g_speed_ki  = 0;
float g_speed_kd  = 0;

/* ─── 速度限幅 ─── */
#define SPEED_MAX   500
#define SPEED_MIN  -500

/* ─── 状态机 ───
 * 时间用"周期计数"度量, 主循环每 10ms 调用一次 Track_Loop()
 * 1 tick = 10ms, 所以 g_track_cfg.corner_fwd_ms 和 TRACK_PIVOT_TIMEOUT 都换算成 tick */
#define TICK_MS  10

enum {
    ST_NORMAL = 0,
    ST_CORNER_FWD,
    ST_CORNER_TURN,
};
static uint8_t  track_state   = ST_NORMAL;
static int8_t   corner_dir    = 0;       /* +1=右转, -1=左转 */
static uint32_t corner_tick   = 0;       /* 状态进入后的周期计数 */

/**
 * @brief 巡线初始化
 */
void Track_Init(void)
{
    track_state = ST_NORMAL;
    corner_dir  = 0;
    corner_tick = 0;
}

/**
 * @brief 复位巡线状态 + 圈数 (发 g 命令时调用, 避免 LAPS DONE 残留)
 */
void Track_Reset(void)
{
    track_state = ST_NORMAL;
    corner_dir  = 0;
    corner_tick = 0;
    g_current_lap   = 0;
    g_corner_count  = 0;
    g_laps_done     = 0;
    g_track_locked  = 0;
}

/**
 * @brief 触发直角弯处理: 进入前冲状态
 * @param dir  +1=右转, -1=左转
 */
static void enter_corner(int8_t dir)
{
    track_state = ST_CORNER_FWD;
    corner_dir  = dir;
    corner_tick = 0;
    g_dbg.corner_dir = dir;
    g_corner_done_by = 0;

    /* 记录转弯目标yaw (在前冲阶段就预存, 前冲时yaw基本不变)
     * 右转: yaw += 90 (逆时针为正在JY61P, 但车右转是顺时针→yaw减小)
     * 左转: yaw -= 90
     * 注意: JY61P yaw 顺时针为负, 逆时针为正
     *       车的"右转"(corner_dir=+1) 是顺时针 → yaw 减小
     *       车的"左转"(corner_dir=-1) 是逆时针 → yaw 增大 */
    float yaw_now = IMU_Get_Yaw_Cached();
    if (dir > 0) {        /* 右转 (顺时针) */
        g_yaw_target = yaw_now - YAW_CORNER_ANGLE;
    } else {              /* 左转 (逆时针) */
        g_yaw_target = yaw_now + YAW_CORNER_ANGLE;
    }
    /* 归一化到 -180~180 */
    while (g_yaw_target >  180.0f) g_yaw_target -= 360.0f;
    while (g_yaw_target < -180.0f) g_yaw_target += 360.0f;

    /* 前冲: 两轮同向, 用基础速度 */
    int16_t spd = (int16_t)g_track_cfg.base_speed;
    g_motor.l = spd;
    g_motor.r = spd;
}

/**
 * @brief 巡线主控 (主循环每10ms调用)
 */
void Track_Loop(void)
{
    Grey_State_t grey_state;
    static float last_centroid = 0.0f;
    static float last_valid    = 0.0f;
    static uint8_t lost_cnt    = 0;

    float centroid, centroid_d, turn;

    /* ─── 0. 锁定状态: 超时停车后等 'g' 复位 ─── */
    if (g_track_locked) {
        g_motor.l = 0;
        g_motor.r = 0;
        g_dbg.state = 3;   /* 3=LOCKED */
        return;
    }

    /* ─── 1. 读灰度 ─── */
    Grey_Read(&grey_state);

    /* 统计活跃通道数 */
    uint8_t i, active_cnt = 0;
    uint8_t active_bits = 0;
    for (i = 0; i < GREY_CHANNEL_COUNT; i++) {
        if (grey_state.active[i]) {
            active_cnt++;
            active_bits |= (1 << i);
        }
    }
    g_dbg.active_cnt  = active_cnt;
    g_dbg.active_bits = active_bits;

    /* ════════════════════════════════════════════════
     * 状态机分发
     * ════════════════════════════════════════════════ */

    /* ─── 状态2: CORNER_TURN — 停车原地转, yaw闭环+灰度双保险 ─── */
    if (track_state == ST_CORNER_TURN) {
        corner_tick++;
        g_dbg.state = 2;

        /* 读当前yaw, 计算误差 (处理±180跨越) */
        g_yaw_now = IMU_Get_Yaw_Cached();
        g_yaw_err = g_yaw_target - g_yaw_now;
        while (g_yaw_err >  180.0f) g_yaw_err -= 360.0f;
        while (g_yaw_err < -180.0f) g_yaw_err += 360.0f;

        /* 退出条件1: 灰度找到中间传感器 (主保险, 找到线就停) */
        if (grey_state.active[3] || grey_state.active[4]) {
            track_state = ST_NORMAL;
            corner_dir  = 0;
            g_corner_done_by = 1;   /* 灰度找到线 */
            last_centroid = grey_state.centroid;
            last_valid    = grey_state.centroid;
            lost_cnt      = 0;
            /* 里程计: 过了一个直角弯, 边索引+1, 当前边距离清零 */
            Odom_Reset_Edge();
            /* 圈数: 过弯计数+1, 满4弯=1圈 */
            g_corner_count++;
            if (g_corner_count >= 4) {
                g_corner_count = 0;
                g_current_lap++;
                if (g_current_lap >= g_target_laps) {
                    g_laps_done = 1;   /* 跑完, 通知主循环停车 */
                }
            }
            /* 落到下面 NORMAL 逻辑 (本帧就用当前 centroid) */
        }
        /* 退出条件2: IMU yaw到位 (副保险, 灰度没找到但角度到了就停, 防转过头) */
        else if (g_imu_assist && (g_yaw_err >= -YAW_ARRIVE_TH && g_yaw_err <= YAW_ARRIVE_TH)) {
            track_state = ST_NORMAL;
            corner_dir  = 0;
            g_corner_done_by = 2;   /* yaw到位 */
            last_centroid = 0.0f;   /* 没有线, 给0直行 */
            last_valid    = 0.0f;
            lost_cnt      = 0;
            Odom_Reset_Edge();
            g_corner_count++;
            if (g_corner_count >= 4) {
                g_corner_count = 0;
                g_current_lap++;
                if (g_current_lap >= g_target_laps) {
                    g_laps_done = 1;
                }
            }
            /* 落到下面 NORMAL 逻辑 */
        }
        /* 退出条件3: 超时锁定停车 (TRACK_PIVOT_TIMEOUT=0 表示不限时) */
        else if (TRACK_PIVOT_TIMEOUT && corner_tick >= TRACK_PIVOT_TIMEOUT) {
            g_motor.l = 0;
            g_motor.r = 0;
            track_state     = ST_NORMAL;
            corner_dir      = 0;
            g_track_locked  = 1;
            g_corner_done_by = 3;   /* 超时 */
            return;
        }
        /* 继续原地转, yaw闭环控制速度 */
        else {
            int16_t spd = (int16_t)g_track_cfg.pivot_speed;
            /* IMU辅助: 接近目标时减速, 避免过冲 */
            if (g_imu_assist) {
                float abs_err = (g_yaw_err >= 0) ? g_yaw_err : -g_yaw_err;
                if (abs_err < YAW_SLOW_BAND) {
                    /* 在15°带内, 速度按比例衰减, 最低保留40% */
                    float scale = 0.4f + 0.6f * (abs_err / YAW_SLOW_BAND);
                    spd = (int16_t)(spd * scale);
                    if (spd < 40) spd = 40;   /* 最小转弯速度 */
                }
            }
            if (corner_dir > 0) {        /* 右转: 左轮正, 右轮负 */
                g_motor.l =  spd;
                g_motor.r = -spd;
            } else {                     /* 左转: 左轮负, 右轮正 */
                g_motor.l = -spd;
                g_motor.r =  spd;
            }
            return;
        }
    }

    /* ─── 状态1: CORNER_FWD — 前冲固定0.5s, 到了切到 CORNER_TURN ─── */
    if (track_state == ST_CORNER_FWD) {
        corner_tick++;
        g_dbg.state = 1;
        /* 前冲 tick 数 = g_track_cfg.corner_fwd_ms / TICK_MS (固定500ms) */
        uint32_t fwd_ticks = (uint32_t)(g_track_cfg.corner_fwd_ms / TICK_MS);
        if (fwd_ticks < 1) fwd_ticks = 1;
        if (corner_tick >= fwd_ticks) {
            /* 前冲结束, 进入原地转弯 */
            track_state  = ST_CORNER_TURN;
            corner_tick  = 0;
            /* 停一下再转 (避免惯性冲突) */
            g_motor.l = 0;
            g_motor.r = 0;
            return;
        }
        /* 前冲中: 保持直行速度 */
        int16_t spd = (int16_t)g_track_cfg.base_speed;
        g_motor.l = spd;
        g_motor.r = spd;
        return;
    }

    /* ─── 状态0: NORMAL — 灰度离心PD巡线 + 直角弯检测 ─── */
    g_dbg.state = 0;
    if (grey_state.valid) {
        centroid = grey_state.centroid;

        /* 直角弯检测: 同一边≥3路激活 (CH0-2左边 或 CH5-7右边)
         * CH0-2 都激活 → 线在左 → 左转 (dir=-1)
         * CH5-7 都激活 → 线在右 → 右转 (dir=+1) */
        uint8_t left_cnt = grey_state.active[0] + grey_state.active[1]
                         + grey_state.active[2];
        uint8_t right_cnt = grey_state.active[5] + grey_state.active[6]
                          + grey_state.active[7];
        if (left_cnt >= 3) {
            enter_corner(-1);   /* 左转 */
            return;
        }
        if (right_cnt >= 3) {
            enter_corner(+1);   /* 右转 */
            return;
        }

        /* 更新上一帧有效方向 (丢线时用) */
        last_valid = centroid;
        lost_cnt = 0;
    } else {
        /* 丢线: 用符号判断方向, 给最大离心值激进转向 */
        if (lost_cnt < TRACK_LOST_HOLD) {
            if (last_valid > 0.01f)
                centroid = 3.5f;
            else if (last_valid < -0.01f)
                centroid = -3.5f;
            else
                centroid = 0.0f;
            lost_cnt++;
        } else {
            centroid = 0.0f;
        }
    }

    /* ─── 微分 + 离心→差速 ─── */
    centroid_d = centroid - last_centroid;
    last_centroid = centroid;

    turn = centroid * g_track_cfg.turn_p + centroid_d * g_track_cfg.turn_d;

    /* 更新调试变量 */
    g_dbg.centroid = centroid;
    g_dbg.turn     = turn;

    /* 小偏差死区补偿 */
    if (centroid >  0.3f && turn < 3.0f) turn = 3.0f;
    if (centroid < -0.3f && turn > -3.0f) turn = -3.0f;

    /* 速度目标 + 限幅 */
    {
        float base = (lost_cnt > 0) ? 100.0f : g_track_cfg.base_speed;
        float spd_l = base + turn;    /* 线偏右 → 左轮快 → 右转找线 */
        float spd_r = base - turn;

        if ( spd_l > SPEED_MAX) spd_l = SPEED_MAX;
        if ( spd_l < SPEED_MIN) spd_l = SPEED_MIN;
        if ( spd_r > SPEED_MAX) spd_r = SPEED_MAX;
        if ( spd_r < SPEED_MIN) spd_r = SPEED_MIN;

        g_motor.l = (int16_t)spd_l;
        g_motor.r = (int16_t)spd_r;
    }
}
