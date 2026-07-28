/**
 * @file    lora.h
 * @brief   大夏龙雀 DX-LR22-433T22D LoRa 模块串口驱动 (UART_LORA, 9600bps)
 *          透明传输模式: ISR + 环形缓冲双向收发
 *
 * 硬件: DX-LR22-433T22D (433MHz LoRa)
 *   通信: UART 9600bps 8N1, 透明传输 (发什么收什么)
 *   配置: 出厂默认即透明模式, 无需 AT 命令初始化
 *   参考: STM32 例程 (RXNE 中断逐字节接收 + IDLE 帧结束)
 *
 * SysConfig 需添加 UART_LORA 实例:
 *   - UART 外设任选 (UART2/UART3 推荐, 避开 UART0=IMU, UART1=BT)
 *   - 波特率 9600, 8N1, RX 中断使能
 *   - 引脚任意空闲 GPIO (PA8/PA9/PA10/PA11 已占用)
 *   - 添加后 SysConfig 自动生成 UART_LORA_INST / UART_LORA_INST_INT_IRQN /
 *     UART_LORA_INST_IRQHandler 等宏, 本驱动自动适配
 *
 * 架构参考: imu.c (ISR 搬字节到环形缓冲, 应用层消费)
 */

#ifndef __LORA_H
#define __LORA_H
#include <stdint.h>

/* LoRa 状态 (诊断 + 控制)
 * volatile: ISR 写 present/rx_cnt, 主循环读, 防编译器缓存 */
typedef volatile struct {
    uint8_t  present;   /* 1=收到过字节 */
    uint8_t  echo;      /* 1=回显收到字节到 UART_BLUETOOTH (调试用, 'U'类机制) */
    uint16_t rx_cnt;    /* 累计接收字节数 (诊断) */
    uint16_t tx_cnt;    /* 累计发送字节数 (诊断) */
} lora_state_t;
extern lora_state_t g_lora;

/* 启用 RX 中断 (init 后调一次, 模仿 IMU_EnableRxIRQ) */
void LORA_EnableRxIRQ(void);

/* 初始化: 清缓冲, 重置状态, 清 RX FIFO 残留 */
void LORA_Init(void);

/* 发送字节流 (非阻塞带超时, TX FIFO 满跳过, 模仿 IMU_SendBytes) */
void LORA_Send(const uint8_t *data, uint16_t len);

/* 发送字符串 (以 \0 结尾) */
void LORA_SendText(const char *text);

/* 缓冲区待读字节数 */
uint16_t LORA_Available(void);

/* 读一个字节, 返回 0=成功 1=缓冲空 */
uint8_t LORA_GetByte(uint8_t *out);

/* 读多个字节到 buf, 返回实际读到的数量 */
uint16_t LORA_Read(uint8_t *buf, uint16_t max_len);

/* 主循环调: echo 诊断 (g_lora.echo=1 时把收到字节 hex 回显到 UART_BLUETOOTH) */
void LORA_Poll(void);

#endif
