/**
 * @file    odometry.c
 * @brief   编码器+IMU 航位推算实现
 */

#include "odometry.h"
#include "imu.h"
#include "motor.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979f
#endif

/* ─── 内部状态 ─── */
static float s_x = 0.0f;              /* mm, 东为正 */
static float s_y = 0.0f;              /* mm, 北为正 */
static float s_theta = 0.0f;          /* 度, -180~180 */
static float s_total_dist = 0.0f;     /* mm, 累计里程 */
static float s_edge_dist = 0.0f;      /* mm, 当前边已走距离 */
static uint8_t s_edge_idx = 0;        /* 当前是第几条边 */

void Odom_Init(void)
{
    s_x = 0;
    s_y = 0;
    s_theta = 0;
    s_total_dist = 0;
    s_edge_dist = 0;
    s_edge_idx = 0;
}

void Odom_Reset(void)
{
    s_x = 0;
    s_y = 0;
    s_theta = g_imu_data.Yaw;   /* 朝向 = 当前yaw */
    s_total_dist = 0;
    s_edge_dist = 0;
    s_edge_idx = 0;
}

void Odom_Reset_Edge(void)
{
    s_edge_dist = 0;
    s_edge_idx++;
}

/**
 * @brief 每10ms调用: 读编码器+IMU, 更新位置
 */
float Odom_Update(void)
{
    /* 1. 读编码器 (10ms内脉冲数) */
    int16_t enc_l = Motor_Read_Encoder_L();
    int16_t enc_r = Motor_Read_Encoder_R();

    /* 2. 转换为位移 (mm)
     * 注意: 编码器读数有正负, 正向前进为正
     *       电机线序: M4=左轮, M2=右轮, 用负号修正转向 (见main.c)
     *       所以编码器返回值的符号需要根据实际情况判断
     *       这里假设前进时 enc_l > 0, enc_r > 0 (如不对, 加负号) */
    float d_l = (float)enc_l * MM_PER_PULSE;
    float d_r = (float)enc_r * MM_PER_PULSE;

    /* 3. 中心位移 */
    float d_c = (d_l + d_r) * 0.5f;

    /* 4. 朝向: 直接用IMU的yaw (比编码器差速准) */
    s_theta = g_imu_data.Yaw;

    /* 5. 位置积分 (theta转弧度) */
    float theta_rad = s_theta * (float)M_PI / 180.0f;
    s_x += d_c * cosf(theta_rad);
    s_y += d_c * sinf(theta_rad);

    /* 6. 累计里程 */
    if (d_c < 0) d_c = -d_c;   /* 取绝对值 */
    s_total_dist += d_c;
    s_edge_dist  += d_c;

    return s_total_dist;
}

/* ─── 查询接口 ─── */
float Odom_Get_X(void)            { return s_x; }
float Odom_Get_Y(void)            { return s_y; }
float Odom_Get_Theta(void)        { return s_theta; }
float Odom_Get_Total_Dist(void)   { return s_total_dist; }
float Odom_Get_Edge_Dist(void)    { return s_edge_dist; }
uint8_t Odom_Get_Edge_Index(void) { return s_edge_idx; }
