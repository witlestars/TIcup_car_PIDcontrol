#include "uart_debug.h"
#include "ti_msp_dl_config.h"

/* 回显开关注入 (由 cmd.c 的 'U' 命令切换, 来自 empty.c 主循环) */
extern uint8_t g_uart_debug_echo;

/* ──────────────────────────────────────────────────────────
 * UART_DEBUG (UART0, PA10=TX, PA11=RX, 9600bps) 串口实现
 *
 * 当前用途: 接维特智能 JY61P IMU 模块 (默认 9600bps)
 *   - JY61P 持续以 0x55 + 类型字节 开头的帧输出姿态数据
 *   - 角度帧 (0x55 0x53): 11 字节, 含 Roll/Pitch/Yaw (int16, 小端, /32768*180)
 *   - 归零命令: 0xFF 0xAA 0x76 0x00 0x00 (5 字节)
 *
 * 数据通路 (中断版):
 *   JY61P → UART0 RX FIFO → RX 中断 → ISR 搬到环形缓冲 → IMU_Poll 取字节解析
 *   中断驱动不依赖主循环轮询, 不会因主循环阻塞丢字节
 * ────────────────────────────────────────────────────────── */

/* RX 环形缓冲区 (256 字节, 8 位索引自动回绕) */
#define DBG_BUF_SIZE 256
static volatile uint8_t  s_rx_buf[DBG_BUF_SIZE];
static volatile uint8_t  s_rx_head;   /* 写入位置 (ISR 推进) */
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
 * @brief 启用 UART_DEBUG RX 中断
 *        在 syscfg 没配 UART RX 中断的情况下, 这里手动启用
 *        (driverlib 允许运行时配置, 不依赖 SysConfig 重新生成)
 *        必须在 UART0 外设初始化 (SYSCFG_DL_init) 之后调用
 */
void UART_Debug_EnableRxIRQ(void)
{
    /* 清 NVIC pending, 防止残留触发 */
    NVIC_ClearPendingIRQ(UART_DEBUG_INST_INT_IRQN);

    /* 启用 RX 中断 + Overrun 错误中断
     * 用 DL_UART_Main_* 系列 API (MSPM0G3507 的 UART 是 Main 类型)
     * DL_UART_MAIN_INTERRUPT_RX: RX FIFO 达到阈值时触发 (默认 1/8 满 = 4 字节)
     *   注: 9600bps 每字节 ~1ms, JY61P 10Hz 每帧 11 字节, 4 字节阈值足够及时
     * DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR: FIFO 溢出时触发, ISR 里清掉防卡死 */
    DL_UART_Main_enableInterrupt(UART_DEBUG_INST,
        DL_UART_MAIN_INTERRUPT_RX | DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR);

    /* 在 NVIC 里启用 UART0 中断 */
    NVIC_EnableIRQ(UART_DEBUG_INST_INT_IRQN);
}

/**
 * @brief UART0 RX 中断服务程序
 *        用 DL_UART_Main_getPendingInterrupt 自动清 pending 标志 (关键!)
 *        处理两种中断: RX (读 FIFO 字节) + OVERRUN (清错误, 读走 FIFO 防卡死)
 *        echo 开启时: 非阻塞发 hex 到蓝牙串口 (TX FIFO 满就跳过该字节, 不等待)
 *
 * @note 之前用 DL_UART_clearInterruptStatus 没真正清 RX pending,
 *        导致 ISR 反复进但 FIFO 空, while 不执行, 新字节不处理, yaw 卡在 0
 */
void UART0_IRQHandler(void)
{
    /* getPendingInterrupt 返回当前 pending 的中断类型, 同时自动清除该标志
     * (这是 MSPM0 driverlib 的标准用法, 比手动 clearInterruptStatus 可靠) */
    uint32_t irq = DL_UART_Main_getPendingInterrupt(UART_DEBUG_INST);

    switch (irq) {
    case DL_UART_MAIN_IIDX_RX:
        /* RX 中断: FIFO 有数据, 全部搬到环形缓冲 */
        while (!DL_UART_Main_isRXFIFOEmpty(UART_DEBUG_INST)) {
            uint8_t b = (uint8_t)DL_UART_Main_receiveData(UART_DEBUG_INST);
            uint8_t next = (uint8_t)(s_rx_head + 1);
            if (next != s_rx_tail) {   /* 缓冲区未满 */
                s_rx_buf[s_rx_head] = b;
                s_rx_head = next;
            }
            /* 原始字节回显: hex 发到蓝牙串口 (非阻塞, TX 满就跳过, 不影响 ISR 时序) */
            if (g_uart_debug_echo) {
                const char h[] = "0123456789ABCDEF";
                char hex[3];
                uint8_t i;
                hex[0] = h[(b >> 4) & 0x0F];
                hex[1] = h[b & 0x0F];
                hex[2] = ' ';
                for (i = 0; i < 3; i++) {
                    if (!DL_UART_Main_isTXFIFOFull(UART_BLUETOOTH_INST)) {
                        DL_UART_Main_transmitDataBlocking(UART_BLUETOOTH_INST, (uint8_t)hex[i]);
                    }
                }
            }
        }
        break;

    case DL_UART_MAIN_IIDX_OVERRUN_ERROR:
        /* Overrun: FIFO 溢出 (ISR 太慢没及时读), 读走所有字节丢弃, 防止卡死 */
        while (!DL_UART_Main_isRXFIFOEmpty(UART_DEBUG_INST)) {
            (void)DL_UART_Main_receiveData(UART_DEBUG_INST);
        }
        break;

    default:
        /* 其他中断 (TX 等), 忽略 */
        break;
    }
}

/**
 * @brief 兼容接口: 主循环轮询 RX FIFO
 *        改用中断后此函数不再需要, 保留仅为兼容旧调用点
 *        (函数体留空, 不做任何事, 中断已自动处理所有字节接收)
 */
void UART_Debug_PollRx(void)
{
    /* 中断版: 不需要主循环轮询, 留空兼容 */
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
