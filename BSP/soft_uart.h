/**
 * @file    soft_uart.h
 * @brief   软件串口 (9600bps 8N1, GPIO+TIMG0 模拟)
 *          硬件UART全被占(IMU/K230/Motor), 蓝牙调参用软件串口
 *
 * 架构:
 *   TX: bit-bang 忙等待 (PB8, 发送时短关中断)
 *   RX: TIMG0 3x过采样 (PB10, 定时器ISR内状态机解码)
 *
 * 引脚 (syscfg GPIO_SOFTUART):
 *   SOFT_TX = PB8  (MSPM0 → ESP32)
 *   SOFT_RX = PB10 (ESP32 → MSPM0)
 *
 * 定时器 (syscfg SOFTUART_TIMER):
 *   TIMG0, 34.72μs 周期 (9600×3=28800Hz)
 */

#ifndef __SOFT_UART_H
#define __SOFT_UART_H

#include <stdint.h>
#include <stdbool.h>

/* 初始化软件串口 (配置GPIO, 启动TIMG0采样) */
void SoftUART_Init(void);

/* 发送一字节 (bit-bang, 阻塞约1ms) */
void SoftUART_SendByte(uint8_t data);

/* 发送字符串 */
void SoftUART_SendString(const char *str);

/* 读一字节 (非阻塞, 无数据返回-1) */
int16_t SoftUART_ReadByte(void);

/* 缓冲区是否有数据可读 */
bool SoftUART_Available(void);

/* TIMG0 定时器ISR调用 (3x过采样RX状态机, ~2μs) */
void SoftUART_TimerISR(void);

#endif /* __SOFT_UART_H */
