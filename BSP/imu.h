/**
 * @file    imu.h
 * @brief   维特智能 JY61P 串口驱动 (UART_IMU, 9600bps, PA10/PA11)
 *          ISR + 环形缓冲 + 帧解析, 合并原 imu_uart.c
 */

#ifndef __IMU_H
#define __IMU_H
#include <stdint.h>

extern uint8_t g_imu_present;   /* 1=收到过有效帧 */
extern uint8_t g_use_imu;       /* 1=解析启用(默认) 0=暂停 */

/* 0x53 角度(度) / 0x52 角速度(°/s) / 0x51 加速度(g) 缓存 */
extern float g_imu_roll, g_imu_pitch, g_imu_yaw;
extern float g_imu_gyrox, g_imu_gyroy, g_imu_gyroz;
extern float g_imu_accx, g_imu_accy, g_imu_accz;

void     IMU_EnableRxIRQ(void);                          /* 启用 RX 中断 (init 后调一次) */
void     IMU_EchoTick(void);                              /* 主循环调: echo 诊断 (非阻塞) */
void     IMU_SendBytes(const uint8_t *data, uint8_t len);/* 发命令给 JY61P */
uint8_t  IMU_Init(void);                                 /* 等1000ms看是否收到帧, 0=成功 */
uint8_t  IMU_Read_RPY(float *r, float *p, float *y);     /* 读角度缓存, 0=有数据 */
float    IMU_Read_Yaw(void);                             /* 读 yaw 缓存 */
uint8_t  IMU_Calibrate_Z(void);                          /* Z轴归零 (600ms delay, 勿在ISR调) */
void     IMU_Poll(void);                                 /* 主循环每轮调: 解析帧 */
float    IMU_Get_Yaw_Cached(void);                       /* 读 yaw 缓存 */

#endif
