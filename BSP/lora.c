/**
 * @file    lora.c
 * @brief   大夏龙雀 DX-LR22-433T22D LoRa 串口驱动 (UART_LORA, 9600bps)
 *          透明传输: ISR + 环形缓冲 + 双向收发
 *          模仿 imu.c 模式, 无协议解析 (应用层自行处理)
 *
 * 与 STM32 例程对应关系:
 *   例程 RXNE 中断逐字节接收  → 本驱动 ISR 搬字节到环形缓冲
 *   例程 IDLE 中断标记帧结束 → 本驱动不依赖 IDLE, 应用层按需分帧
 *   例程 echo 测试           → 本驱动 g_lora.echo=1 时 hex 回显到 BT
 */

#include "lora.h"
#include "ti_msp_dl_config.h"
#include <string.h>

/* ─── 状态 (诊断 + 控制) ─── */
lora_state_t g_lora = {0};

/* ─── RX 环形缓冲 (ISR 生产, 应用层消费) ─── */
#define LORA_RX_BUF_SIZE 256
static volatile uint8_t  s_rx_buf[LORA_RX_BUF_SIZE];
static volatile uint16_t s_rx_head, s_rx_tail;

/* ═══════════════ UART 层: ISR + 环形缓冲 ═══════════════ */

void LORA_EnableRxIRQ(void)
{
    NVIC_ClearPendingIRQ(UART_LORA_INST_INT_IRQN);
    DL_UART_Main_enableInterrupt(UART_LORA_INST,
        DL_UART_MAIN_INTERRUPT_RX | DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR);
    NVIC_EnableIRQ(UART_LORA_INST_INT_IRQN);
}

/* UART_LORA RX 中断: 搬字节到环形缓冲, ISR 内不做解析/echo (防卡死) */
void UART_LORA_INST_IRQHandler(void)
{
    uint32_t irq = DL_UART_Main_getPendingInterrupt(UART_LORA_INST);
    switch (irq) {
    case DL_UART_MAIN_IIDX_RX:
        while (!DL_UART_Main_isRXFIFOEmpty(UART_LORA_INST)) {
            uint8_t b = (uint8_t)DL_UART_Main_receiveData(UART_LORA_INST);
            uint16_t next = (uint16_t)((s_rx_head + 1) % LORA_RX_BUF_SIZE);
            if (next != s_rx_tail) {
                s_rx_buf[s_rx_head] = b;
                s_rx_head = next;
            }
            g_lora.rx_cnt++;
            g_lora.present = 1;
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

void LORA_Init(void)
{
    s_rx_head = s_rx_tail = 0;
    g_lora.present = 0;
    g_lora.echo    = 0;
    g_lora.rx_cnt  = 0;
    g_lora.tx_cnt  = 0;
    /* 清空 RX FIFO 残留 */
    while (!DL_UART_Main_isRXFIFOEmpty(UART_LORA_INST)) {
        (void)DL_UART_Main_receiveData(UART_LORA_INST);
    }
}

/* 发 N 字节 (非阻塞带超时, TX FIFO 满跳过, 模仿 IMU_SendBytes) */
void LORA_Send(const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        uint16_t timeout = 60000;
        while (DL_UART_Main_isTXFIFOFull(UART_LORA_INST) && timeout--);
        if (timeout == 0) break;   /* TX FIFO 持续满, 放弃避免卡死 */
        DL_UART_Main_transmitDataBlocking(UART_LORA_INST, data[i]);
        g_lora.tx_cnt++;
    }
}

void LORA_SendText(const char *text)
{
    LORA_Send((const uint8_t *)text, (uint16_t)strlen(text));
}

/* 缓冲区待读字节数 */
uint16_t LORA_Available(void)
{
    return (uint16_t)((s_rx_head + LORA_RX_BUF_SIZE - s_rx_tail) % LORA_RX_BUF_SIZE);
}

/* 读一个字节, 0=成功 1=缓冲空 */
uint8_t LORA_GetByte(uint8_t *out)
{
    if (s_rx_head == s_rx_tail) return 1;
    *out = s_rx_buf[s_rx_tail];
    s_rx_tail = (uint16_t)((s_rx_tail + 1) % LORA_RX_BUF_SIZE);
    return 0;
}

/* 读多个字节到 buf, 返回实际读到的数量 */
uint16_t LORA_Read(uint8_t *buf, uint16_t max_len)
{
    uint16_t n = 0;
    while (n < max_len && LORA_GetByte(&buf[n]) == 0) n++;
    return n;
}

/* 主循环调: echo 诊断 (g_lora.echo=1 时把收到字节 hex 回显到 UART_BLUETOOTH)
 * 模仿 IMU_EchoTick, 非阻塞, 每轮预算 16 字节 */
void LORA_Poll(void)
{
    if (!g_lora.echo) return;
    const char h[] = "0123456789ABCDEF";
    uint8_t budget = 16;
    while (budget--) {
        uint8_t b;
        if (LORA_GetByte(&b)) break;
        char hex[3];
        hex[0] = h[(b >> 4) & 0x0F];
        hex[1] = h[b & 0x0F];
        hex[2] = ' ';
        for (uint8_t i = 0; i < 3; i++) {
            if (!DL_UART_Main_isTXFIFOFull(UART_BLUETOOTH_INST)) {
                DL_UART_Main_transmitDataBlocking(UART_BLUETOOTH_INST, (uint8_t)hex[i]);
            }
        }
    }
    if (!DL_UART_Main_isTXFIFOFull(UART_BLUETOOTH_INST)) {
        DL_UART_Main_transmitDataBlocking(UART_BLUETOOTH_INST, '\n');
    }
}
