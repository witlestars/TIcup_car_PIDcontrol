/**
 * @file    imu.h
 * @brief   维特智能 JY61P 六轴姿态传感器驱动 (I2C)
 *
 * 硬件接线:
 *   JY61P SCL → MSPM0 PA15 (与驱动板共用软件I2C总线)
 *   JY61P SDA → MSPM0 PA17
 *   JY61P VCC → 3.3V (兼容5V)
 *   JY61P GND → GND (必须与MSPM0共地)
 *
 * I2C地址: 0x50 (7位, 维特智能模块地址, 非MPU6050的0x68)
 *          与驱动板 0x26 不冲突, 共享同一条软件I2C
 *
 * 寄存器 (从0x3D开始连续6字节角度):
 *   0x3D-0x3E: Roll  (int16, raw/32768*180 = 度)
 *   0x3F-0x40: Pitch
 *   0x41-0x42: Yaw
 *
 * Z轴归零命令 (维特智能协议):
 *   1. 解锁: i2cWrite(0x50, 0x69, [0x88, 0xB5])
 *   2. 归零: i2cWrite(0x50, 0x01, [0x04, 0x00])
 *
 * yaw范围: -180° ~ +180°, 顺时针为正
 * 漂移: JY61P (MPU6050+磁力计) 约 1°/min, 跑30s圈漂移~0.5°可接受
 */

#ifndef __IMU_H
#define __IMU_H
#include <stdint.h>

/* JY61P I2C 7位地址 */
#define JY61P_I2C_ADDR   0x50

/* 角度寄存器起始地址 */
#define JY61P_REG_RPY     0x3D

/* IMU 在线标志 (1=检测到, 0=未接, 所有I2C操作自动跳过) */
extern uint8_t g_imu_present;

/* I2C 总线互斥开关 (IMU 和 OLED 共用 PA17/PA15 软件 I2C, 必须互斥):
 *   g_use_imu=1 → IMU 占用总线, OLED 所有 I2C 操作跳过
 *   g_use_imu=0 → OLED 占用总线, IMU 所有 I2C 操作跳过
 * 默认 1 (上电优先 IMU, 因为巡线需要 IMU 辅助转弯) */
extern uint8_t g_use_imu;

/* ────────────── API ────────────── */

/** 初始化: 测试I2C通信, 返回0=成功, 非0=失败 */
uint8_t IMU_Init(void);

/** 读取 Roll/Pitch/Yaw (度), 返回0=成功 */
uint8_t IMU_Read_RPY(float *roll, float *pitch, float *yaw);

/** 只读 Yaw (度, -180~180), 失败返回上次值 */
float IMU_Read_Yaw(void);

/** Z轴归零 (当前朝向设为0°), 返回0=成功 */
uint8_t IMU_Calibrate_Z(void);

/** 每10ms轮询: 缓存yaw, 供Track_Loop和Odom_Update用 */
void IMU_Poll(void);

/** 获取缓存的yaw (度, -180~180), 不触发I2C读取 */
float IMU_Get_Yaw_Cached(void);

#endif
