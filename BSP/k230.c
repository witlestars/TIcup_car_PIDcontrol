/**
 * @file    k230.c
 * @brief   庐山派 K230 视觉模块对接 (UART 双向 + GPIO 备用)
 *          复用 UART_LORA (UART2, PB15/PB16), LoRa 在 H 题不需要
 *          UART: ISR+环形缓冲, MSPM0→K230 发题号, K230→MSPM0 预留球坐标
 *          GPIO: PA14 轮询检测上升沿 (备用诊断)
 */

#include "k230.h"
#include "ti_msp_dl_config.h"
#include <string.h>
#include <stdio.h>

/* ─── 状态 ─── */
k230_state_t g_k230 = {0};

/* ─── UART RX 环形缓冲 (ISR 生产, 应用层消费) ─── */
#define K230_RX_BUF_SIZE 128
static volatile uint8_t  s_rx_buf[K230_RX_BUF_SIZE];
static volatile uint16_t s_rx_head, s_rx_tail;

/* ─── GPIO 备用: 上一次电平 (上升沿检测) ─── */
static uint8_t s_last_level = 0;

/* ═══════════════ UART 层: ISR + 环形缓冲 (复用 UART_LORA_INST) ═══════════════ */

void K230_EnableRxIRQ(void)
{
    NVIC_ClearPendingIRQ(UART_LORA_INST_INT_IRQN);
    DL_UART_Main_enableInterrupt(UART_LORA_INST,
        DL_UART_MAIN_INTERRUPT_RX | DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR);
    NVIC_EnableIRQ(UART_LORA_INST_INT_IRQN);
}

/* UART_LORA RX 中断: 搬字节到环形缓冲 (接管自 lora.c, LoRa 已禁用) */
void UART_LORA_INST_IRQHandler(void)
{
    uint32_t irq = DL_UART_Main_getPendingInterrupt(UART_LORA_INST);
    switch (irq) {
    case DL_UART_MAIN_IIDX_RX:
        while (!DL_UART_Main_isRXFIFOEmpty(UART_LORA_INST)) {
            uint8_t b = (uint8_t)DL_UART_Main_receiveData(UART_LORA_INST);
            uint16_t next = (uint16_t)((s_rx_head + 1) % K230_RX_BUF_SIZE);
            if (next != s_rx_tail) {
                s_rx_buf[s_rx_head] = b;
                s_rx_head = next;
            }
            g_k230.rx_cnt++;
            g_k230.present = 1;
        }
        break;
    case DL_UART_MAIN_IIDX_OVERRUN_ERROR:
        /* OVERRUN: 丢弃 FIFO 残留防卡死 */
        while (!DL_UART_Main_isRXFIFOEmpty(UART_LORA_INST)) {
            (void)DL_UART_Main_receiveData(UART_LORA_INST);
        }
        break;
    default:
        break;
    }
}

/* ═══════════════ 对外 API ═══════════════ */

void K230_Init(void)
{
    s_rx_head = s_rx_tail = 0;
    s_last_level = 0;
    g_k230.bead_detected = 0;
    g_k230.detect_cnt    = 0;
    g_k230.sig_level     = 0;
    g_k230.present       = 0;
    g_k230.rx_cnt        = 0;
    g_k230.tx_cnt        = 0;
    /* 清空 UART RX FIFO 残留 */
    while (!DL_UART_Main_isRXFIFOEmpty(UART_LORA_INST)) {
        (void)DL_UART_Main_receiveData(UART_LORA_INST);
    }
}

/* 发 N 字节 (非阻塞带超时, TX FIFO 满跳过) */
static void k230_send(const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        uint16_t timeout = 60000;
        while (DL_UART_Main_isTXFIFOFull(UART_LORA_INST) && timeout--);
        if (timeout == 0) break;
        DL_UART_Main_transmitDataBlocking(UART_LORA_INST, data[i]);
        g_k230.tx_cnt++;
    }
}

void K230_SendText(const char *text)
{
    k230_send((const uint8_t *)text, (uint16_t)strlen(text));
}

/* 发题号: "T<id>\n" (id 1~5) */
void K230_SendTask(uint8_t task_id)
{
    if (task_id < 1) task_id = 1;
    if (task_id > 5) task_id = 5;
    char buf[8];
    snprintf(buf, sizeof(buf), "T%d\n", task_id);
    k230_send((const uint8_t *)buf, (uint16_t)strlen(buf));
}

/* 缓冲区待读字节数 */
uint16_t K230_Available(void)
{
    return (uint16_t)((s_rx_head + K230_RX_BUF_SIZE - s_rx_tail) % K230_RX_BUF_SIZE);
}

/* 读一个字节, 0=成功 1=缓冲空 */
uint8_t K230_GetByte(uint8_t *out)
{
    if (s_rx_head == s_rx_tail) return 1;
    *out = s_rx_buf[s_rx_tail];
    s_rx_tail = (uint16_t)((s_rx_tail + 1) % K230_RX_BUF_SIZE);
    return 0;
}

/* ═══════════════ GPIO 备用: PA14 轮询 ═══════════════ */

/* 主循环 10ms 调:
 * 1. GPIO 轮询 PA14 (备用诊断, 检测上升沿)
 * 2. UART 收字节预留 (球坐标解析队友完成后在此扩展) */
void K230_Poll(void)
{
    /* --- GPIO 备用: PA14 上升沿检测 --- */
    uint8_t level = (DL_GPIO_readPins(GPIO_K230_PORT,
                                      GPIO_K230_K230_SIG_PIN) != 0) ? 1 : 0;
    g_k230.sig_level = level;
    if (level == 1 && s_last_level == 0) {
        g_k230.bead_detected = 1;
        g_k230.detect_cnt++;
    }
    s_last_level = level;

    /* --- UART 收字节预留: 队友CV完成后在此解析 "B<pos>\n" --- */
    /* TODO: 球坐标解析 (基本要求3/4/5/6 需要)
     * while (K230_GetByte(&b) == 0) { ... 解析球位置 ... } */
}

uint8_t K230_IsBeadDetected(void)
{
    return g_k230.bead_detected;
}

void K230_Clear(void)
{
    g_k230.bead_detected = 0;
}

uint8_t K230_Get_Signal(void)
{
    return (uint8_t)((DL_GPIO_readPins(GPIO_K230_PORT,
                                       GPIO_K230_K230_SIG_PIN) != 0) ? 1 : 0);
}
