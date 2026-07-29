/**
 * @file    k230.h
 * @brief   庐山派 K230 视觉模块对接 (UART 双向 + GPIO 备用)
 *
 * 硬件: 庐山派 K230 CanMV 开发板
 *   UART: 复用 UART_LORA (UART2, PB15=MSPM0_TX→K230_RX, PB16=MSPM0_RX←K230_TX)
 *         9600bps 8N1, ISR+环形缓冲
 *   GPIO: PA14 (K230→MSPM0, 拉高=检测到钢珠, 备用/诊断)
 *
 * 通信协议 (MSPM0 ↔ K230):
 *   MSPM0 → K230: "T<task_id>\n"  题号通知 (1~5)
 *   K230 → MSPM0: "B<ball_pos>\n" 球位置mm (预留, 队友CV算法完成后对接)
 *
 * 接线:
 *   MSPM0 PB15 (TX) ──── K230 RX
 *   MSPM0 PB16 (RX) ──── K230 TX
 *   GND ──────────────── GND (共地)
 *   (3.3V 直连, 无需电平转换)
 *
 * 历史: 原方案用 GPIO 单信号 (PA14), 因 H 题需球坐标双向通信,
 *       改用 UART (复用原 LoRa 引脚, LoRa 在 H 题不需要)
 */

#ifndef __K230_H
#define __K230_H
#include <stdint.h>

/* K230 状态 (诊断 + 控制) */
typedef struct {
    uint8_t  bead_detected;  /* 1=GPIO检测到钢珠上升沿 (备用, PA14) */
    uint32_t detect_cnt;     /* GPIO 累计检测上升沿次数 (诊断) */
    uint8_t  sig_level;      /* 当前 PA14 电平 (0/1, 诊断用) */
    uint8_t  present;        /* 1=UART 收到过字节 (K230 在线) */
    uint16_t rx_cnt;         /* UART 累计接收字节数 (诊断) */
    uint16_t tx_cnt;         /* UART 累计发送字节数 (诊断) */
} k230_state_t;
extern k230_state_t g_k230;

/* 初始化: 清状态 + 清 UART RX FIFO + 使能 RX 中断
 * GPIO 由 SysConfig 配置, UART 复用 UART_LORA_INST */
void K230_Init(void);

/* 使能 UART RX 中断 (复用 UART_LORA_INST_INT_IRQN) */
void K230_EnableRxIRQ(void);

/* 主循环 10ms 调: GPIO 轮询 PA14 (备用) + UART 收字节预留解析 */
void K230_Poll(void);

/* 发题号给 K230: "T<task_id>\n" (task_id 1~5) */
void K230_SendTask(uint8_t task_id);

/* 发文本给 K230 (通用) */
void K230_SendText(const char *text);

/* UART 缓冲区待读字节数 */
uint16_t K230_Available(void);

/* 从 UART 缓冲读一个字节, 0=成功 1=缓冲空 */
uint8_t K230_GetByte(uint8_t *out);

/* 查询 GPIO 是否检测到钢珠 (备用), 0=无 1=有 */
uint8_t K230_IsBeadDetected(void);

/* 清除 GPIO 检测标志 */
void K230_Clear(void);

/* 读 PA14 原始电平 (调试用) */
uint8_t K230_Get_Signal(void);

#endif
