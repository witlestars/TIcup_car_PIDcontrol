/**
 * @file    soft_uart.c
 * @brief   软件串口实现 (9600bps 8N1)
 *
 * TX: bit-bang 忙等待, 发送时关中断 ~1ms
 * RX: TIMG0 3x过采样状态机, ISR ~2μs
 *
 * RX 3x过采样原理:
 *   定时器每 34.7μs (1/3 bit) 中断一次
 *   起始位下降沿 → 每3次采样=1bit, 多数表决
 *   bit序列: 起始位(低) + D0..D7(LSB) + 停止位(高)
 */

#include "soft_uart.h"
#include "ti_msp_dl_config.h"
#include "delay.h"

/* 9600bps: 每bit 104.17μs */
#define BIT_US              104

/* RX 环形缓冲 (volatile: ISR写, 主循环读) */
#define RX_BUF_SIZE         64
#define RX_BUF_MASK         (RX_BUF_SIZE - 1)
static volatile uint8_t  rx_buf[RX_BUF_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;

/* RX 3x过采样状态机变量 (仅ISR访问, 无需volatile) */
static uint8_t rx_phase       = 0;   /* 0=IDLE等起始位 1=接收中 */
static uint8_t rx_sample_cnt  = 0;   /* 当前bit内采样计数 0-2 */
static uint8_t rx_bit_count   = 0;   /* 已完成bit数 1=起始 2-9=数据 10=停止 */
static uint8_t rx_data        = 0;
static uint8_t rx_low_votes   = 0;   /* 当前bit低电平票数 */


void SoftUART_Init(void)
{
    /* GPIO + TIMG0 由 SysConfig (SYSCFG_DL_init) 配好, 这里只清状态 */
    rx_head = 0;
    rx_tail = 0;
    rx_phase = 0;
    rx_sample_cnt = 0;
    rx_bit_count = 0;
    rx_data = 0;
    rx_low_votes = 0;

    /* TX 空闲为高 */
    DL_GPIO_setPins(GPIO_SOFTUART_PORT, GPIO_SOFTUART_SOFT_TX_PIN);

    /* 启动 TIMG0 定时器 (开始3x过采样) */
    NVIC_ClearPendingIRQ(SOFTUART_TIMER_INST_INT_IRQN);
    NVIC_EnableIRQ(SOFTUART_TIMER_INST_INT_IRQN);
    DL_Timer_startCounter(SOFTUART_TIMER_INST);
}


void SoftUART_SendByte(uint8_t data)
{
    uint8_t i;

    /* 关中断保证位时序不被打断 (耗时约1ms) */
    __disable_irq();

    /* 起始位 (拉低) */
    DL_GPIO_clearPins(GPIO_SOFTUART_PORT, GPIO_SOFTUART_SOFT_TX_PIN);
    delay_us(BIT_US);

    /* 8数据位, LSB first */
    for (i = 0; i < 8; i++) {
        if (data & (1u << i))
            DL_GPIO_setPins(GPIO_SOFTUART_PORT, GPIO_SOFTUART_SOFT_TX_PIN);
        else
            DL_GPIO_clearPins(GPIO_SOFTUART_PORT, GPIO_SOFTUART_SOFT_TX_PIN);
        delay_us(BIT_US);
    }

    /* 停止位 (拉高) */
    DL_GPIO_setPins(GPIO_SOFTUART_PORT, GPIO_SOFTUART_SOFT_TX_PIN);
    delay_us(BIT_US);

    __enable_irq();
}


void SoftUART_SendString(const char *str)
{
    while (*str) {
        SoftUART_SendByte((uint8_t)*str);
        str++;
    }
}


int16_t SoftUART_ReadByte(void)
{
    int16_t ret;

    if (rx_head == rx_tail)
        return -1;   /* 缓冲空 */

    ret = rx_buf[rx_tail];
    rx_tail = (rx_tail + 1) & RX_BUF_MASK;
    return ret;
}


bool SoftUART_Available(void)
{
    return (rx_head != rx_tail);
}


/* TIMG0 定时器中断: 3x过采样RX状态机
 * 每 34.7μs 调用一次, ISR内只读引脚+状态机, ~2μs */
void SoftUART_TimerISR(void)
{
    /* 读RX引脚电平 (非0=高, 0=低) */
    bool level = (DL_GPIO_readPins(GPIO_SOFTUART_PORT,
                                   GPIO_SOFTUART_SOFT_RX_PIN) != 0);

    if (rx_phase == 0) {
        /* IDLE: 检测起始位下降沿 */
        if (!level) {
            rx_phase = 1;
            rx_sample_cnt = 1;
            rx_low_votes = 1;
            rx_bit_count = 0;
            rx_data = 0;
        }
        return;
    }

    /* 接收中: 累积采样 */
    rx_sample_cnt++;
    if (!level) rx_low_votes++;

    if (rx_sample_cnt < 3)
        return;   /* 当前bit未采满3次 */

    /* 3次采样完成, 多数表决 (低票≥2=低电平) */
    rx_sample_cnt = 0;
    bool bit_val = (rx_low_votes < 2);   /* true=高, false=低 */
    rx_low_votes = 0;
    rx_bit_count++;

    if (rx_bit_count == 1) {
        /* 起始位: 必须低, 否则帧错误回IDLE */
        if (bit_val) rx_phase = 0;
    } else if (rx_bit_count <= 9) {
        /* 数据位 D0-D7 (LSB first) */
        if (bit_val)
            rx_data |= (1u << (rx_bit_count - 2));
    } else {
        /* 停止位: 高=正确帧, 入缓冲; 低=帧错误丢弃 */
        if (bit_val) {
            uint16_t next = (rx_head + 1) & RX_BUF_MASK;
            if (next != rx_tail) {   /* 缓冲未满才写 */
                rx_buf[rx_head] = rx_data;
                rx_head = next;
            }
        }
        rx_phase = 0;   /* 回IDLE等下一帧 */
    }
}
