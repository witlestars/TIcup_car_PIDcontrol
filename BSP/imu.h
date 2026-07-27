/**
 * @file    imu.h
 * @brief   维特智能 JY61P 串口驱动 (UART_DEBUG 通道)
 *
 * 硬件接线 (改用串口, I2C 方案已弃用):
 *   JY61P TX  → MSPM0 PA11 (UART_DEBUG RX)
 *   JY61P RX  → MSPM0 PA10 (UART_DEBUG TX)  ← 发归零命令用
 *   JY61P VCC → 3.3V (兼容5V)
 *   JY61P GND → GND (必须与MSPM0共地)
 *
 * 串口参数:
 *   波特率 9600 (JY61P 出厂默认), 8N1, 无流控
 *   JY61P 上电后自动以 10Hz 持续输出姿态数据帧
 *
 * 数据帧格式 (11 字节):
 *   [0x55][TYPE][D0 D1 D2 D3 D4 D5 D6 D7][SUM]
 *   TYPE=0x53 角度帧: D0D1=Roll, D2D3=Pitch, D4D5=Yaw (int16, 小端, /32768*180=度)
 *   TYPE=0x51 加速度, 0x52 角速度, 0x54 磁场, 0x56 端口状态...
 *   SUM = (0x55 + TYPE + ΣD0..D7) & 0xFF
 *
 * Z 轴归零命令 (5 字节):
 *   解锁: 0xFF 0xAA 0x69 0x88 0xB5
 *   归零: 0xFF 0xAA 0x76 0x00 0x00
 *   保存: 0xFF 0xAA 0x00 0x00 0x00
 *
 * yaw范围: -180° ~ +180°, 顺时针为正
 * 漂移: JY61P (MPU6050+磁力计) 约 1°/min, 跑30s圈漂移~0.5°可接受
 *
 * 历史: 原 I2C 实现 (PA17/PA15) 因模块 I2C 不工作已弃用
 */

#ifndef __IMU_H
#define __IMU_H
#include <stdint.h>

/* IMU 在线标志 (1=收到有效帧, 0=未接或无数据) */
extern uint8_t g_imu_present;

/* IMU 启用开关 (兼容旧 switch_mode 接口):
 *   g_use_imu=1 → IMU 解析启用 (默认)
 *   g_use_imu=0 → IMU 解析暂停 (yaw 缓存冻结)
 * 改串口后不再与 OLED 互斥, 此标志仅控制 IMU 解析, 不影响 OLED */
extern uint8_t g_use_imu;

/* ────────────── 缓存数据 (解析后自动更新) ──────────────
 * 0x53 角度帧: Roll/Pitch/Yaw (度)
 * 0x52 角速度帧: GyroX/Y/Z (°/s, 量程 ±2000)
 * 0x51 加速度帧: AccX/Y/Z (g, 量程 ±16g)
 * JY61P 默认 10Hz 输出, 三种帧交替出现 */
extern float g_imu_roll, g_imu_pitch, g_imu_yaw;
extern float g_imu_gyrox, g_imu_gyroy, g_imu_gyroz;
extern float g_imu_accx, g_imu_accy, g_imu_accz;

/* ────────────── API ────────────── */

/** 初始化: 等 200ms 看 UART_DEBUG 是否收到 JY61P 数据帧, 返回0=成功 */
uint8_t IMU_Init(void);

/** 读取 Roll/Pitch/Yaw (度), 直接返回缓存值, 不触发 I2C, 返回0=有数据 */
uint8_t IMU_Read_RPY(float *roll, float *pitch, float *yaw);

/** 只读 Yaw (度, -180~180), 返回缓存值 */
float IMU_Read_Yaw(void);

/** Z轴归零 (通过串口发归零命令给 JY61P), 返回0=已发送 */
uint8_t IMU_Calibrate_Z(void);

/** 主循环轮询: 从环形缓冲取字节解析 0x51/0x52/0x53 帧
 *  每轮调用 (非 50ms 节拍), 降低 yaw 延迟避免车乱跑 */
void IMU_Poll(void);

/** 获取缓存的 yaw (度, -180~180) */
float IMU_Get_Yaw_Cached(void);

#endif
