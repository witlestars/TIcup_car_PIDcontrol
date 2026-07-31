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
#include "imu.h"

/* ─── 运行时调参变量 ───
 * 调参方法论 (参考网上成熟经验):
 *   1. 纯P起步, 找直道不振最大P
 *   2. D = P × 0.4~0.5 抑制震荡
 *   3. 弯道拐不过 → 加P或降速, 不要加D (D过大弯道抖)
 * 8路灰度权重±2.0, centroid范围±2.0
 * 驱动板速度-1000~1000, base=230对应约782mm/s
 *
 * 实测: 此系统需要大D阻尼 (D/P≈2.0), 小D会发散
 *       因8路灰度centroid离散跳变, 需大D抑制冲击
 *
 * 分段PD: 直道大D阻尼, 弯道大P转向 (curve_in_curve区分)
 */
track_cfg_t g_track_cfg = {
    230,      /* base_speed 220→230 用户指定 */
    17.0f,    /* turn_p  (19.5→17 用户降P, D33/D43抖动差不多说明非D主导) */
    27.0f,    /* turn_d  (24→27 二分: D24大幅/D28小幅, 取27) */
    19.0f,    /* curve_p (不分段时不用) */
    20.0f,    /* curve_d (不分段时不用) */
};

/* ─── 超时锁定标志 (保留接口, 槽口型不触发) ─── */
uint8_t g_track_locked = 0;

/* ─── 巡线模式: 0=单PD, 1=分段PD (task.c 设置) ─── */
uint8_t g_track_mode = 0;

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

/* ─── 槽口型赛道尺寸 (mm) ───
 * 直道长 1500mm, 半圆弯半径 500mm, 弯道周长 π×500≈1570mm
 * 进弯预判: 走够1400mm就切弯道PD (提前100mm, 转向更早介入) */
#define STRAIGHT_LEN_MM  1200.0f
#define CURVE_LEN_MM     1570.0f
/* 一圈周长: 实测6400mm
 * 降速点: total_dist >= 6400mm 时降速 */
#define LAP_PERIMETER_MM    6400.0f
#define FINISH_ALIGN_DIST_MM 6000.0f
#define SLOW_DOWN_DIST_MM   LAP_PERIMETER_MM
#define SLOW_DOWN_END_MM    6750.0f

/* ─── 内部状态 ─── */
static float last_centroid = 0.0f;
static float last_centroid_d = 0.0f;  /* D项低通滤波历史 */
static float last_yaw = 0.0f;         /* IMU yaw历史, 用于弯道检测 */
static float s_start_yaw = 0.0f;      /* 起点yaw基准 (IMU yaw是相对值, 用相对量判断) */
static float s_curve_yaw_accum = 0.0f; /* 弯道中yaw累计变化量 (达到180°判出弯) */
static float last_valid    = 0.0f;
static uint8_t lost_cnt    = 0;

/* ─── 半圆弯检测 (用于过弯计数, 不停车)
 * 槽口型半圆弯特点: 灰度持续偏一边 (centroid 绝对值持续 > 阈值)
 * 检测进入半圆弯: centroid 同号持续 N 帧
 * 检测离开半圆弯: centroid 回到接近 0 ───
 * 注意: 8路灰度权重±2.0, centroid最大±2.0, 阈值不能>2.0 */
#define CURVE_ENTER_TH    0.9f    /* centroid 绝对值超过0.9认为进弯 (1.3→0.9 提前介入) */
#define CURVE_LEAVE_TH    0.5f    /* centroid 绝对值低于此值认为离开弯道 */
#define CURVE_HOLD_TICKS  4       /* 持续 4 帧 (40ms) 才确认进弯 (8→4 加快切换) */
static uint8_t curve_in_curve = 0;  /* 1=当前在弯道中 */
static uint8_t curve_hold_cnt = 0;  /* 持续偏一边的帧数 */
static int8_t  curve_last_sign = 0; /* 上一次 centroid 符号 (+1/-1) */

/**
 * @brief 巡线初始化
 */
void Track_Init(void)
{
    last_centroid = 0.0f;
    last_centroid_d = 0.0f;
    last_yaw = 0.0f;
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
    last_centroid_d = 0.0f;
    last_yaw = g_imu_data.Yaw;  /* 复位时记录当前yaw作基准 */
    s_start_yaw = g_imu_data.Yaw;  /* 记录起点yaw, 弯道检测用相对量 (IMU yaw是相对值) */
    s_curve_yaw_accum = 0.0f;   /* 弯道yaw累计清零 */
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
        centroid = grey_state.centroid;  /* 直接读取, 不滤波 (滤波增加相位滞后致发散) */
        last_valid = centroid;
        lost_cnt = 0;
    } else {
        /* 丢线处理 (分段: 直道温和避抖, 弯道快速回正但力道小)
         * 弯道丢线: 立即放大(速度快) 但 ±1.5(力道小, ±2.0太猛)
         * 直道丢线: 前2帧温和保持, 3帧+放大 */
        if (lost_cnt < TRACK_LOST_HOLD) {
            if (curve_in_curve) {
                /* 弯道丢线: 立即放大到±2.25, 加大1/4力度 (1.8力道小) */
                if (last_valid > 0.1f)       centroid = 2.25f;
                else if (last_valid < -0.1f) centroid = -2.25f;
                else                          centroid = last_valid;
            } else {
                /* 直道丢线: 前2帧温和保持, 3帧+放大到±2.25 */
                if (lost_cnt < 2) {
                    centroid = last_valid;
                } else {
                    if (last_valid > 0.1f)       centroid = 2.25f;
                    else if (last_valid < -0.1f) centroid = -2.25f;
                    else                          centroid = last_valid;
                }
            }
            lost_cnt++;
        } else {
            centroid = 0.0f;
        }
    }

    /* ─── 3. 弯道检测 (始终启用, 用于过弯计数g_corner_count) ───
     * 方案: 里程驱动进弯, yaw累计转角驱动出弯, 物理确定可靠
     * 进弯: edge_dist >= STRAIGHT_LEN_MM → curve_in_curve=1, g_corner_count++ (进弯计数)
     * 出弯: 弯道中累计 |yaw变化| >= 185° → curve_in_curve=0, Odom_Reset_Edge
     * 进弯计数: 一圈=2个弯=进弯2次=count=2 (3/4圈进弯2时count=2, 靠2帧过滤+最终直道降速防误停)
     * 单PD模式(g_track_mode==0): 仅计数, PD/速度不切换
     * 分段PD模式(g_track_mode==1): 计数 + 切换PD参数 + 弯道降速 */
    {
        float yaw_now = g_imu_data.Yaw;
        float edge_dist = Odom_Get_Edge_Dist();

        if (!curve_in_curve) {
            /* 直道: 走够STRAIGHT_LEN_MM → 切弯道 */
            if (edge_dist >= STRAIGHT_LEN_MM) {
                curve_in_curve = 1;
                s_curve_yaw_accum = 0.0f;  /* 开始累计yaw变化 */
                last_yaw = yaw_now;         /* 记录进弯时刻yaw作基准 */
                g_corner_count++;   /* 进弯计数+1 (一圈=进弯2次=count2) */
                g_dbg.state = 1;    /* 1=弯道中 */
            } else {
                g_dbg.state = 0;    /* 0=直道 */
            }
        } else {
            /* 弯道: 累计yaw变化量, 达到185°判出弯 */
            float yaw_d = yaw_now - last_yaw;
            if (yaw_d > 180.0f)  yaw_d -= 360.0f;   /* 处理±180越界 */
            if (yaw_d < -180.0f) yaw_d += 360.0f;
            s_curve_yaw_accum += (yaw_d >= 0) ? yaw_d : -yaw_d;  /* 累计绝对值 */
            last_yaw = yaw_now;

            if (s_curve_yaw_accum >= 185.0f) {
                /* 已转够185°, 出弯 */
                curve_in_curve = 0;
                Odom_Reset_Edge();  /* 重置边距离, 下段直道从0计 */
                g_dbg.state = 0;    /* 回到直道 */
            }
        }
    }

    /* ─── 4. 离心PD (单PD统一参数 / 分段PD按curve_in_curve切换) ─── */
    centroid_d = centroid - last_centroid;
    {
        /* 单PD模式(g_track_mode==0): 始终用turn_p/turn_d
         * 分段PD模式(g_track_mode==1): 弯道用curve_p/curve_d, 直道用turn_p/turn_d */
        float p_eff = (g_track_mode == 1 && curve_in_curve) ? g_track_cfg.curve_p : g_track_cfg.turn_p;
        float d_eff = (g_track_mode == 1 && curve_in_curve) ? g_track_cfg.curve_d : g_track_cfg.turn_d;

        /* 终点前增强回正: 6000mm后直到停车保持+30% */
        if (g_corner_count >= 2 && Odom_Get_Total_Dist() >= FINISH_ALIGN_DIST_MM) {
            p_eff *= 1.3f;
        }

        turn = centroid * p_eff + centroid_d * d_eff;

        if (turn > 150.0f)  turn = 150.0f;
        if (turn < -150.0f) turn = -150.0f;
    }

    last_centroid = centroid;

    g_dbg.centroid = centroid;
    g_dbg.turn     = turn;

    /* ─── 5. 速度目标 + 限幅 ─── */
    {
        /* 默认全速; 分段PD弯道降到250; 最终接近终止线降速50%回正
         * 最终接近判定: count>=2(已进弯2) 且 总里程>=6400mm
         * 一圈实测周长6400mm */
        float base = g_track_cfg.base_speed;

        if (g_track_mode == 1 && curve_in_curve) {
            base = 250.0f;  /* 分段PD弯道降速 */
        }

        /* 6400~6750mm 从50%线性降到40%, 减小停车惯性和速度突变 */
        if (g_corner_count >= 2 && Odom_Get_Total_Dist() >= SLOW_DOWN_DIST_MM) {
            float slow_ratio = (Odom_Get_Total_Dist() - SLOW_DOWN_DIST_MM) /
                               (SLOW_DOWN_END_MM - SLOW_DOWN_DIST_MM);
            if (slow_ratio > 1.0f) slow_ratio = 1.0f;
            base = g_track_cfg.base_speed * (0.5f - 0.1f * slow_ratio);
        }

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
