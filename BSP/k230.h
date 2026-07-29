/**
 * @file    k230.h
 * @brief   庐山派 K230 视觉模块 — 题号发送 (轻量版)
 *
 * 说明:
 *   队友的 vision_protocol.c 已接管 UART_K230 的 RX (接收视觉坐标)
 *   本模块只负责 TX: 发题号通知给 K230 ("T<task_id>\n")
 *   UART_K230 全双工, 收发互不冲突
 *
 * 接线 (UART_K230 = UART1 外设, PA8/PA9):
 *   MSPM0 PA8 (TX) ──── K230 RX
 *   MSPM0 PA9 (RX) ──── K230 TX
 *   GND ──────────────── GND
 */

#ifndef __K230_H
#define __K230_H
#include <stdint.h>

/* 发题号给 K230: "T<task_id>\n" (task_id 1~5, 超范围自动钳位) */
void K230_SendTask(uint8_t task_id);

/* 发文本给 K230 (通用, 带超时防 TX FIFO 满卡死) */
void K230_SendText(const char *text);

#endif
