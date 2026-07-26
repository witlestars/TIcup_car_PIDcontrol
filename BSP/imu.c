/**
 * @file    imu.c
 * @brief   维特智能 JY61P I2C 驱动实现
 */

#include "imu.h"
#include "IOI2C.h"
#include "delay.h"

/* ─── 内部状态 ─── */
static float s_yaw_cached   = 0.0f;   /* 最近一次读取的 yaw (度) */
static float s_pitch_cached = 0.0f;
static float s_roll_cached  = 0.0f;
static uint8_t s_imu_ok = 0;          /* 1=通信正常 */
uint8_t g_imu_present = 0;            /* 1=检测到JY61P, 0=未接(所有I2C操作跳过) */
uint8_t g_use_imu = 1;                /* 1=IMU占用I2C总线(OLED禁用), 0=OLED占用I2C总线(IMU禁用)
                                       * IMU和OLED共用 PA17/PA15 软件I2C, 必须互斥访问 */

/**
 * @brief 初始化: 读取一次角度验证通信
 * @return 0=成功, 非0=失败
 * 注意: 如果 g_use_imu=0 (OLED占用总线), 直接跳过不发起I2C
 */
uint8_t IMU_Init(void)
{
    if (!g_use_imu) {
        /* OLED 占用总线, IMU 不初始化 */
        g_imu_present = 0;
        s_imu_ok = 0;
        return 0xFF;
    }
    /* 探测 JY61P 是否在线: 尝试读寄存器, ACK=0 说明设备存在 */
    uint8_t buf[6];
    uint8_t probe = i2cRead(JY61P_I2C_ADDR, JY61P_REG_RPY, 6, buf);
    if (probe != 0) {
        g_imu_present = 0;   /* 未接, 后续所有IMU操作跳过, 避免I2C超时拖慢主循环 */
        s_imu_ok = 0;
        return probe;
    }
    g_imu_present = 1;

    float r, p, y;
    uint8_t ret = IMU_Read_RPY(&r, &p, &y);
    if (ret == 0) {
        s_imu_ok = 1;
        s_yaw_cached = y;
        s_pitch_cached = p;
        s_roll_cached = r;
    } else {
        s_imu_ok = 0;
    }
    return ret;
}

/**
 * @brief 读取 Roll/Pitch/Yaw (度)
 * @return 0=成功, 非0=I2C失败
 */
uint8_t IMU_Read_RPY(float *roll, float *pitch, float *yaw)
{
    uint8_t buf[6];
    int16_t raw_r, raw_p, raw_y;

    if (!g_imu_present || !g_use_imu) {
        /* IMU 未接 或 OLED 占用总线, 返回缓存值, 不触发I2C */
        if (roll)  *roll  = s_roll_cached;
        if (pitch) *pitch = s_pitch_cached;
        if (yaw)   *yaw   = s_yaw_cached;
        return 1;
    }

    /* 从 0x3D 连续读 6 字节 */
    uint8_t ret = i2cRead(JY61P_I2C_ADDR, JY61P_REG_RPY, 6, buf);
    if (ret != 0) {
        /* 失败: 返回上次缓存值 */
        if (roll)  *roll  = s_roll_cached;
        if (pitch) *pitch = s_pitch_cached;
        if (yaw)   *yaw   = s_yaw_cached;
        return ret;
    }

    /* 维特智能字节序: 高字节在前 */
    raw_r = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
    raw_p = (int16_t)(((uint16_t)buf[2] << 8) | buf[3]);
    raw_y = (int16_t)(((uint16_t)buf[4] << 8) | buf[5]);

    /* 转换: raw / 32768 × 180 = 度 */
    s_roll_cached  = (float)raw_r / 32768.0f * 180.0f;
    s_pitch_cached = (float)raw_p / 32768.0f * 180.0f;
    s_yaw_cached   = (float)raw_y / 32768.0f * 180.0f;

    if (roll)  *roll  = s_roll_cached;
    if (pitch) *pitch = s_pitch_cached;
    if (yaw)   *yaw   = s_yaw_cached;

    s_imu_ok = 1;
    return 0;
}

/**
 * @brief 读取 yaw (度, -180~180)
 *         失败返回缓存值, 保证不会因I2C错误返回异常数据
 */
float IMU_Read_Yaw(void)
{
    float y;
    if (IMU_Read_RPY(NULL, NULL, &y) == 0) {
        return y;
    }
    return s_yaw_cached;
}

/**
 * @brief Z轴归零: 当前朝向设为0°
 *        用于起点校准 / 每圈重置
 * @return 0=成功
 */
uint8_t IMU_Calibrate_Z(void)
{
    if (!g_imu_present || !g_use_imu) return 1;   /* 未接 或 OLED占用总线, 跳过 */

    uint8_t unlock[2] = {0x88, 0xB5};
    uint8_t calib[2]  = {0x04, 0x00};
    uint8_t ret;

    /* 1. 解锁寄存器 */
    ret = i2cWrite(JY61P_I2C_ADDR, 0x69, 2, unlock);
    if (ret != 0) return ret;
    delay_ms(200);

    /* 2. Z轴归零 */
    ret = i2cWrite(JY61P_I2C_ADDR, 0x01, 2, calib);
    if (ret != 0) return ret;
    delay_ms(200);

    /* 归零后清缓存 */
    s_yaw_cached = 0.0f;
    return 0;
}

/**
 * @brief 每10ms轮询: 读取并缓存 yaw
 *        供 Track_Loop 和 Odom_Update 使用, 避免重复I2C读取
 */
void IMU_Poll(void)
{
    IMU_Read_RPY(NULL, NULL, NULL);
}

/**
 * @brief 获取缓存的 yaw (度), 不触发 I2C 读取
 */
float IMU_Get_Yaw_Cached(void)
{
    return s_yaw_cached;
}
