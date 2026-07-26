/**
 * @file    odometry.h
 * @brief   航位推算 — 编码器 + IMU 融合定位
 *
 * 原理:
 *   每10ms读取左右轮编码器脉冲 (Motor_Read_Encoder_L/R),
 *   转换为位移, 用IMU的yaw作为朝向, 积分得到 (x, y, theta)
 *
 * 物理参数 (motor.h):
 *   轮径 = 65mm, 减速比 = 40, 磁环线数 = 11
 *   1 脉冲位移 = π×65 / (40×11) = 0.464mm  (单边轮)
 *   注意: Motor_Read_Encoder 返回的是10ms内脉冲数
 *
 * 坐标系:
 *   theta=0°   → +x 方向 (东)
 *   theta=90°  → +y 方向 (北)
 *   theta=180° → -x 方向 (西)
 *   顺时针为负, 逆时针为正 (与JY61P一致)
 *
 * 正方形赛道定位 (1500mm×1500mm):
 *   起点定为 (0,0), 起点朝向定为 theta0
 *   第0条边: theta0 方向, 走完1500mm后过直角弯
 *   第1条边: theta0 + 90° 方向
 *   ... 每过1个直角弯 edge_idx++
 *   当前边已走距离 = edge_dist (过弯时清零)
 */

#ifndef __ODOMETRY_H
#define __ODOMETRY_H
#include <stdint.h>

/* 物理常量 */
#define WHEEL_DIAMETER_MM   65.0f
#define GEAR_RATIO          40
#define PULSE_PER_ROUND     11
/* 1脉冲位移 = π × 直径 / (减速比 × 线数) */
#define MM_PER_PULSE        (3.14159f * WHEEL_DIAMETER_MM / (GEAR_RATIO * PULSE_PER_ROUND))

/* ────────────── API ────────────── */

/** 初始化: 清零所有位置状态 */
void Odom_Init(void);

/**
 * @brief 每10ms调用: 读编码器+IMU, 更新位置
 *        必须在 IMU_Poll() 之后调用 (用缓存yaw)
 * @return 累计总里程 (mm)
 */
float Odom_Update(void);

/** 重置位置到原点, 朝向设为当前yaw (用于起点校准) */
void Odom_Reset(void);

/** 重置当前边的已走距离 (直角弯后调用) */
void Odom_Reset_Edge(void);

/* ─── 查询接口 ─── */
float Odom_Get_X(void);            /* mm, 东为正 */
float Odom_Get_Y(void);            /* mm, 北为正 */
float Odom_Get_Theta(void);        /* 度, -180~180 */
float Odom_Get_Total_Dist(void);   /* mm, 累计总里程 */
float Odom_Get_Edge_Dist(void);    /* mm, 当前边已走距离 */
uint8_t Odom_Get_Edge_Index(void); /* 当前是第几条边 (0~3, 过弯递增) */

#endif
