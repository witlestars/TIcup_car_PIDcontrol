/**
 * @file    imu_uart.h
 * @brief   JY61P IMU 串口驱动 (UART_DEBUG 通道, UART0, PA10/PA11, 9600bps)
 *
 * 硬件接线:
 *   JY61P TX  → MSPM0 PA11 (UART_DEBUG RX)
 *   JY61P RX  → MSPM0 PA10 (UART_DEBUG TX)  ← 发归零命令用
 *   JY61P VCC → 3.3V (兼容5V)
 *   JY61P GND → GND (必须与MSPM0共地)
 *
 * 串口参数:
 *   波特率 9600 (JY61P 出厂默认), 8N1, 无流控
 *   JY61P 上电后自动以 10Hz 持续输出姿态数据帧
 *
 * 接口:
 *   - IMU_UART_EnableRxIRQ(): 启用 UART0 RX 中断 (在 SYSCFG_DL_init 之后调一次)
 *   - IMU_UART_PollRx()      : 兼容接口, 中断版下留空 (主循环不再需要轮询)
 *   - IMU_UART_GetByte()     : 从环形缓冲区取 1 字节 (返回1=有数据, 0=空)
 *   - IMU_UART_SendBytes()   : 发 N 字节给 JY61P (用于归零/解锁命令)
 *   - IMU_UART_EchoTick()    : 主循环调: echo 诊断输出 (非阻塞)
 *
 * 数据通路 (中断版):
 *   JY61P → UART0 RX FIFO → UART0_IRQHandler (ISR) → 环形缓冲 → IMU_Poll 取字节
 */

#ifndef __IMU_UART_H
#define __IMU_UART_H
#include <stdint.h>

/* ── JY61P IMU 串口接口 ── */
void IMU_UART_EnableRxIRQ(void);   /* 启用 RX 中断 (init 后调一次) */
void IMU_UART_EchoTick(void);      /* 主循环调: echo 诊断输出 (非阻塞) */
void IMU_UART_PollRx(void);        /* 兼容接口, 中断版下空操作 */
uint8_t IMU_UART_GetByte(uint8_t *byte_out);
void IMU_UART_SendBytes(const uint8_t *data, uint8_t len);

#endif
