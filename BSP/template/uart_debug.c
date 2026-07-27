#include "uart_debug.h"
#include "ti_msp_dl_config.h"

/* 回显开关注入 (由 cmd.c 的 'U' 命令切换, 来自 empty.c 主循环) */
extern uint8_t g_uart_debug_echo;

/* ──────────────────────────────────────────────────────────
 * UART_DEBUG (UART1, PA10=TX, PA11=RX, 9600bps) 串口实现
 *
 * 当前用途: 接维特智能 JY61P IMU 模块 (默认 9600bps)
 *   - JY61P 持续以 0x55 + 类型字节 开头的帧输出姿态数据
 *   - 角度帧 (0x55 0x53): 11 字节, 含 Roll/Pitch/Yaw (int16, 小端, /32768*180)
 *   - 归零命令: 0xFF 0xAA 0x76 0x00 0x00 (5 字节)
 *
 * 数据量估算: 10Hz × 11字节 = 110 字节/秒, 主循环 100Hz, 每周期 ~1 字节
 * 环形缓冲区 256 字节 >> 110 字节/秒, 不会溢出
 * ────────────────────────────────────────────────────────── */

/* RX 环形缓冲区 (256 字节, 8 位索引自动回绕) */
#define DBG_BUF_SIZE 256
static volatile uint8_t  s_rx_buf[DBG_BUF_SIZE];
static volatile uint8_t  s_rx_head;   /* 写入位置 (PollRx 推进) */
static volatile uint8_t  s_rx_tail;   /* 读取位置 (GetByte 推进) */

/* 旧 VOFA+ JustFloat 接口 (保留兼容, 当前不再使用, grey.c 已禁用) */
void UART_Debug_Send(float *data, uint8_t count)
{
    uint8_t i;
    uint8_t *p;
    for (i = 0; i < count; i++) {
        p = (uint8_t *)(&data[i]);
        DL_UART_Main_transmitDataBlocking(UART_DEBUG_INST, p[0]);
        DL_UART_Main_transmitDataBlocking(UART_DEBUG_INST, p[1]);
        DL_UART_Main_transmitDataBlocking(UART_DEBUG_INST, p[2]);
        DL_UART_Main_transmitDataBlocking(UART_DEBUG_INST, p[3]);
    }
    /* JustFloat 帧尾 */
    DL_UART_Main_transmitDataBlocking(UART_DEBUG_INST, 0x00);
    DL_UART_Main_transmitDataBlocking(UART_DEBUG_INST, 0x00);
    DL_UART_Main_transmitDataBlocking(UART_DEBUG_INST, 0x80);
    DL_UART_Main_transmitDataBlocking(UART_DEBUG_INST, 0x7f);
}

/**
 * @brief 主循环调用: 把 UART_DEBUG RX FIFO 里的字节搬到环形缓冲区
 *        同时清 overrun/timeout 错误, 防止 RX 卡死
 * @note  必须在 CMD_Poll 之前调 (供 IMU 解析器尽早拿到数据)
 */
void UART_Debug_PollRx(void)
{
    /* 清 overrun / framing / rx_timeout 等错误标志 */
    DL_UART_clearInterruptStatus(UART_DEBUG_INST,
        DL_UART_INTERRUPT_OVERRUN_ERROR |
        DL_UART_INTERRUPT_RX_TIMEOUT_ERROR);

    /* 把 RX FIFO 里所有字节搬到环形缓冲区 */
    while (!DL_UART_Main_isRXFIFOEmpty(UART_DEBUG_INST)) {
        uint8_t b = (uint8_t)DL_UART_Main_receiveData(UART_DEBUG_INST);
        uint8_t next = (uint8_t)(s_rx_head + 1);
        if (next != s_rx_tail) {   /* 缓冲区未满 */
            s_rx_buf[s_rx_head] = b;
            s_rx_head = next;
        }
        /* 原始字节回显: 每个字节以 hex 发到 PC (诊断 JY61P 串口) */
        if (g_uart_debug_echo) {
            const char h[] = "0123456789ABCDEF";
            char hex[4];
            hex[0] = h[(b >> 4) & 0x0F];
            hex[1] = h[b & 0x0F];
            hex[2] = ' ';
            hex[3] = '\0';
            const char *p = hex;
            while (*p) {
                uint16_t timeout = 60000;
                while (DL_UART_Main_isTXFIFOFull(UART_BLUETOOTH_INST) && timeout--);
                if (timeout == 0) break;
                DL_UART_Main_transmitDataBlocking(UART_BLUETOOTH_INST, (uint8_t)*p);
                p++;
            }
        }
        /* 满则丢弃 (理论上不会发生, 256 字节 / 110字节每秒 远大于消费速度) */
    }
    /* 回显开启时: 加换行分隔, 方便阅读 */
    if (g_uart_debug_echo) {
        uint16_t timeout = 60000;
        while (DL_UART_Main_isTXFIFOFull(UART_BLUETOOTH_INST) && timeout--);
        DL_UART_Main_transmitDataBlocking(UART_BLUETOOTH_INST, '\n');
    }
}

/**
 * @brief 从环形缓冲区取 1 字节
 * @return 1=取到, 0=缓冲区空
 */
uint8_t UART_Debug_GetByte(uint8_t *byte_out)
{
    if (s_rx_head == s_rx_tail) return 0;   /* 空 */
    *byte_out = s_rx_buf[s_rx_tail];
    s_rx_tail = (uint8_t)(s_rx_tail + 1);
    return 1;
}

/**
 * @brief 发 N 字节给 JY61P (用于归零/解锁/保存命令)
 *        命令帧格式: 0xFF 0xAA REG VAL_LO VAL_HI
 */
void UART_Debug_SendBytes(const uint8_t *data, uint8_t len)
{
    uint8_t i;
    for (i = 0; i < len; i++) {
        uint16_t timeout = 60000;
        while (DL_UART_Main_isTXFIFOFull(UART_DEBUG_INST) && timeout--);
        if (timeout == 0) break;
        DL_UART_Main_transmitDataBlocking(UART_DEBUG_INST, data[i]);
    }
}
