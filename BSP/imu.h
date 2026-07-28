/**
 * @file    imu.h
 * @brief   维特智能 JY61P 串口驱动 (UART_IMU, 9600bps, PA10/PA11)
 *          ISR + 环形缓冲 + 帧解析, 合并原 imu_uart.c
 */

#ifndef __IMU_H
#define __IMU_H
#include <stdint.h>

/* IMU 缓存数据 (解析后自动更新) */
typedef struct {
    float roll, pitch, yaw;       /* 0x53 角度 (度) */
    float gyrox, gyroy, gyroz;   /* 0x52 角速度 (°/s) */
    float accx, accy, accz;       /* 0x51 加速度 (g) */
    uint8_t present;              /* 1=收到过有效帧 */
    uint8_t use_imu;              /* 1=解析启用(默认) 0=暂停 */
} imu_state_t;
extern imu_state_t g_imu;

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
