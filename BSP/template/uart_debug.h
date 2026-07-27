#ifndef __UART_DEBUG_H
#define __UART_DEBUG_H

#include <stdint.h>

/**
 * @file uart_debug.h
 * @brief UART_DEBUG (UART1, PA10=TX, PA11=RX, 9600bps) — JY61P IMU 串口通道
 *
 * 历史用途: 原本用于 VOFA+ JustFloat 调试输出 (115200)。
 * 现在用途: 接维特智能 JY61P IMU 串口 (9600bps 默认)。
 *   - JY61P TX → MSPM0 PA11 (UART_DEBUG RX)
 *   - JY61P RX → MSPM0 PA10 (UART_DEBUG TX)  ← 发归零命令用
 *   - JY61P VCC → 3.3V, GND → 共地
 *
 * 接口:
 *   - UART_Debug_EnableRxIRQ(): 启用 UART0 RX 中断 (在 SYSCFG_DL_init 之后调一次)
 *   - UART_Debug_PollRx()      : 兼容接口, 中断版下留空 (主循环不再需要轮询)
 *   - UART_Debug_GetByte()     : 从环形缓冲区取 1 字节 (返回1=有数据, 0=空)
 *   - UART_Debug_SendBytes()   : 发 N 字节给 JY61P (用于归零/解锁命令)
 *   - UART_Debug_Send()        : 旧 VOFA+ JustFloat 接口 (保留兼容, 不再使用)
 *
 * 数据通路 (中断版):
 *   JY61P → UART0 RX FIFO → UART0_IRQHandler (ISR) → 环形缓冲 → IMU_Poll 取字节
 */

void UART_Debug_Send(float *data, uint8_t count);

/* ── JY61P IMU 串口接口 ── */
void UART_Debug_EnableRxIRQ(void);   /* 启用 RX 中断 (init 后调一次) */
void UART_Debug_PollRx(void);        /* 兼容接口, 中断版下空操作 */
uint8_t UART_Debug_GetByte(uint8_t *byte_out);
void UART_Debug_SendBytes(const uint8_t *data, uint8_t len);

#endif
