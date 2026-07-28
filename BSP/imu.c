/**
 * @file    imu.c
 * @brief   维特智能 JY61P 串口驱动 (UART_IMU, 9600bps, PA10/PA11)
 *          合并原 imu_uart.c, 统一管理 ISR + 环形缓冲 + 帧解析
 *
 * 帧格式 (11字节): [0x55][TYPE][D0..D7][SUM]
 *   TYPE=0x51 加速度(/32768*16=g) 0x52 角速度(/32768*2000=°/s) 0x53 角度(/32768*180=度)
 *   SUM=(0x55+TYPE+ΣD0..D7)&0xFF 严格校验
 *
 * 教训/历程见 DEVELOPMENT_NOTES.txt
 */

#include "imu.h"
#include "delay.h"
#include "ti_msp_dl_config.h"

extern uint8_t g_imu_uart_echo;

/* ─── 缓存数据 (解析后自动更新) ─── */
imu_state_t g_imu = { .present = 0, .use_imu = 1 };

/* ─── RX 环形缓冲 (ISR 生产, IMU_Poll 消费) ─── */
#define RX_BUF_SIZE 256
static volatile uint8_t s_rx_buf[RX_BUF_SIZE];
static volatile uint8_t s_rx_head, s_rx_tail;

/* ─── 帧解析状态机 ─── */
typedef enum { PS_FIND_55, PS_TYPE, PS_DATA, PS_SUM } parse_state_t;
static parse_state_t s_state = PS_FIND_55;
static uint8_t s_frame_type, s_data_idx, s_data_buf[8], s_sum;

/* ═══════════════ UART 层: ISR + 环形缓冲 + 收发字节 ═══════════════ */

void IMU_EnableRxIRQ(void)
{
    NVIC_ClearPendingIRQ(UART_IMU_INST_INT_IRQN);
    DL_UART_Main_enableInterrupt(UART_IMU_INST,
        DL_UART_MAIN_INTERRUPT_RX | DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR);
    NVIC_EnableIRQ(UART_IMU_INST_INT_IRQN);
}

/* UART0 RX 中断: 搬字节到环形缓冲, ISR 内不做 echo/解析 (防卡死) */
void UART0_IRQHandler(void)
{
    uint32_t irq = DL_UART_Main_getPendingInterrupt(UART_IMU_INST);
    switch (irq) {
    case DL_UART_MAIN_IIDX_RX:
        while (!DL_UART_Main_isRXFIFOEmpty(UART_IMU_INST)) {
            uint8_t b = (uint8_t)DL_UART_Main_receiveData(UART_IMU_INST);
            uint8_t next = (uint8_t)(s_rx_head + 1);
            if (next != s_rx_tail) {
                s_rx_buf[s_rx_head] = b;
                s_rx_head = next;
            }
        }
        break;
    case DL_UART_MAIN_IIDX_OVERRUN_ERROR:
        while (!DL_UART_Main_isRXFIFOEmpty(UART_IMU_INST)) {
            (void)DL_UART_Main_receiveData(UART_IMU_INST);
        }
        break;
    default:
        break;
    }
}

static uint8_t imu_get_byte(uint8_t *byte_out)
{
    if (s_rx_head == s_rx_tail) return 0;
    *byte_out = s_rx_buf[s_rx_tail];
    s_rx_tail = (uint8_t)(s_rx_tail + 1);
    return 1;
}

/* 发 N 字节给 JY61P (归零/解锁/保存命令: 0xFF 0xAA REG VAL_LO VAL_HI) */
void IMU_SendBytes(const uint8_t *data, uint8_t len)
{
    for (uint8_t i = 0; i < len; i++) {
        uint16_t timeout = 60000;
        while (DL_UART_Main_isTXFIFOFull(UART_IMU_INST) && timeout--);
        if (timeout == 0) break;
        DL_UART_Main_transmitDataBlocking(UART_IMU_INST, data[i]);
    }
}

/* 主循环调: echo 诊断输出 ('U' 命令开启, 非阻塞, TX 满跳过) */
void IMU_EchoTick(void)
{
    if (!g_imu_uart_echo) return;
    const char h[] = "0123456789ABCDEF";
    uint8_t budget = 8;
    while (budget--) {
        uint8_t b;
        if (!imu_get_byte(&b)) break;
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

/* ═══════════════ 帧解析层 ═══════════════ */

static void imu_parse_byte(uint8_t b)
{
    switch (s_state) {
    case PS_FIND_55:
        if (b == 0x55) { s_sum = 0x55; s_state = PS_TYPE; }
        break;
    case PS_TYPE:
        s_sum += b; s_frame_type = b; s_data_idx = 0; s_state = PS_DATA;
        break;
    case PS_DATA:
        s_data_buf[s_data_idx++] = b; s_sum += b;
        if (s_data_idx >= 8) s_state = PS_SUM;
        break;
    case PS_SUM:
        if (b == (uint8_t)(s_sum & 0xFF)) {
            int16_t r0 = (int16_t)(((uint16_t)s_data_buf[1] << 8) | s_data_buf[0]);
            int16_t r1 = (int16_t)(((uint16_t)s_data_buf[3] << 8) | s_data_buf[2]);
            int16_t r2 = (int16_t)(((uint16_t)s_data_buf[5] << 8) | s_data_buf[4]);
            if (s_frame_type == 0x53) {
                g_imu.roll  = (float)r0 / 32768.0f * 180.0f;
                g_imu.pitch = (float)r1 / 32768.0f * 180.0f;
                g_imu.yaw   = (float)r2 / 32768.0f * 180.0f;
                g_imu.present = 1;
            } else if (s_frame_type == 0x52) {
                g_imu.gyrox = (float)r0 / 32768.0f * 2000.0f;
                g_imu.gyroy = (float)r1 / 32768.0f * 2000.0f;
                g_imu.gyroz = (float)r2 / 32768.0f * 2000.0f;
            } else if (s_frame_type == 0x51) {
                g_imu.accx = (float)r0 / 32768.0f * 16.0f;
                g_imu.accy = (float)r1 / 32768.0f * 16.0f;
                g_imu.accz = (float)r2 / 32768.0f * 16.0f;
            }
        }
        s_state = PS_FIND_55;
        break;
    default:
        s_state = PS_FIND_55;
        break;
    }
}

/* ═══════════════ 对外 API ═══════════════ */

/* 等 1000ms 看是否收到 JY61P 帧, 返回0=成功 */
uint8_t IMU_Init(void)
{
    if (!g_imu.use_imu) { g_imu.present = 0; return 0xFF; }
    s_state = PS_FIND_55; s_data_idx = 0; g_imu.present = 0;
    while (!DL_UART_Main_isRXFIFOEmpty(UART_IMU_INST)) {
        (void)DL_UART_Main_receiveData(UART_IMU_INST);
    }
    for (uint16_t i = 0; i < 200; i++) {
        delay_ms(5);
        uint8_t b;
        while (imu_get_byte(&b)) imu_parse_byte(b);
        if (g_imu.present) return 0;
    }
    return 0x01;
}

uint8_t IMU_Read_RPY(float *roll, float *pitch, float *yaw)
{
    if (roll)  *roll  = g_imu.roll;
    if (pitch) *pitch = g_imu.pitch;
    if (yaw)   *yaw   = g_imu.yaw;
    return (g_imu.present && g_imu.use_imu) ? 0 : 1;
}

float IMU_Read_Yaw(void) { return g_imu.yaw; }

/* Z轴归零: 解锁→归零→保存 (内部 600ms delay, 不要在 ISR 内调) */
uint8_t IMU_Calibrate_Z(void)
{
    static const uint8_t unlock[] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
    static const uint8_t calib[] = {0xFF, 0xAA, 0x76, 0x00, 0x00};
    static const uint8_t save[]  = {0xFF, 0xAA, 0x00, 0x00, 0x00};
    IMU_SendBytes(unlock, 5); delay_ms(200);
    IMU_SendBytes(calib, 5); delay_ms(200);
    IMU_SendBytes(save,  5); delay_ms(200);
    g_imu.yaw = 0.0f;
    return 0;
}

/* 主循环每轮调用: 从环形缓冲取字节解析帧, 降低 yaw 延迟避免车乱跑 */
void IMU_Poll(void)
{
    if (!g_imu.use_imu) return;
    uint8_t b, budget = 32;
    while (budget-- && imu_get_byte(&b)) imu_parse_byte(b);
}

float IMU_Get_Yaw_Cached(void) { return g_imu.yaw; }
