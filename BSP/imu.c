/**
 * @file    imu.c
 * @brief   维特智能 JY61P 串口驱动 (UART_IMU, 9600bps, PA10/PA11)
 *          ISR 内直接解析帧 (无环形缓冲, 借鉴队友方案)
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

/* ─── 缓存数据 (解析后自动更新) ─── */
imu_state_t g_imu = { .present = 0, .use_imu = 1 };

/* ─── 帧解析状态机 (ISR 内运行, 静态变量) ─── */
typedef enum { PS_FIND_55, PS_TYPE, PS_DATA, PS_SUM } parse_state_t;
static parse_state_t s_state = PS_FIND_55;
static uint8_t s_frame_type, s_data_idx, s_data_buf[8], s_sum;

/* ═══════════════ UART 层: ISR + 中断使能 + 收发字节 ═══════════════ */

void IMU_EnableRxIRQ(void)
{
    NVIC_ClearPendingIRQ(UART_IMU_INST_INT_IRQN);
    DL_UART_Main_enableInterrupt(UART_IMU_INST,
        DL_UART_MAIN_INTERRUPT_RX | DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR);
    NVIC_EnableIRQ(UART_IMU_INST_INT_IRQN);
}

/* ISR 内单字节解析 (借鉴队友方案: ISR 直接喂状态机, 无环形缓冲) */
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

/* UART0 RX 中断: 读字节直接喂状态机, 处理 OVERRUN 防 FIFO 卡死 */
void UART0_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_IMU_INST)) {
    case DL_UART_MAIN_IIDX_RX:
        while (!DL_UART_Main_isRXFIFOEmpty(UART_IMU_INST)) {
            uint8_t b = (uint8_t)DL_UART_Main_receiveData(UART_IMU_INST);
            imu_parse_byte(b);
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

/* ═══════════════ 对外 API ═══════════════ */

/* 等 1000ms 看是否收到 JY61P 帧, 返回0=成功
 * 内部先启用 RX 中断 (NVIC + 外设级 RX/OVERRUN), 再 polling 等 present */
uint8_t IMU_Init(void)
{
    if (!g_imu.use_imu) { g_imu.present = 0; return 0xFF; }
    IMU_EnableRxIRQ();   /* 启用 UART_IMU RX 中断 (NVIC_EnableIRQ + RX|OVERRUN) */
    s_state = PS_FIND_55; s_data_idx = 0; g_imu.present = 0;
    while (!DL_UART_Main_isRXFIFOEmpty(UART_IMU_INST)) {
        (void)DL_UART_Main_receiveData(UART_IMU_INST);
    }
    for (uint16_t i = 0; i < 200; i++) {
        delay_ms(5);
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

float IMU_Get_Yaw_Cached(void) { return g_imu.yaw; }
