/**
 * @file    imu.c
 * @brief   维特智能 JY61P 串口驱动实现 (UART_DEBUG 通道)
 *
 * 数据流:
 *   JY61P ──9600bps──> MSPM0 PA11 (UART_DEBUG RX)
 *                      ↓
 *   UART_Debug_PollRx (主循环调用) → 环形缓冲区
 *                      ↓
 *   IMU_Poll (10ms一次) → 取字节 → 状态机解析 0x55 0x53 帧缓存 yaw
 *
 *   IMU_Read_Yaw / IMU_Get_Yaw_Cached → 直接返回缓存值 (不阻塞主循环)
 */

#include "imu.h"
#include "uart_debug.h"
#include "delay.h"
#include "ti_msp_dl_config.h"

/* ─── 内部状态 ─── */
static float s_yaw_cached   = 0.0f;   /* 最近一次解析的 yaw (度) */
static float s_pitch_cached = 0.0f;
static float s_roll_cached  = 0.0f;

uint8_t g_imu_present = 0;            /* 1=收到过有效 JY61P 帧, 0=未接 */
uint8_t g_use_imu = 1;                /* 1=IMU 解析启用 (默认), 0=暂停解析 */

/* ─── 帧解析状态机 ───
 * 帧: [0x55][TYPE][D0..D7][SUM], 共 11 字节
 * 只关心 TYPE=0x53 角度帧
 */
typedef enum {
    PS_FIND_55 = 0,   /* 等待帧头 0x55 */
    PS_TYPE,          /* 收 0x55 后, 读类型字节 */
    PS_DATA,          /* 收 8 字节数据 D0..D7 */
    PS_SUM            /* 收校验字节 */
} parse_state_t;

static parse_state_t s_state = PS_FIND_55;
static uint8_t  s_frame_type;          /* 当前帧类型 (0x53 角度 等) */
static uint8_t  s_data_idx;            /* PS_DATA 状态下已收字节数 */
static uint8_t  s_data_buf[8];         /* 8 字节数据缓冲 */
static uint8_t  s_sum;                 /* 累计校验和 */

/**
 * @brief 喂一个字节给状态机, 若完整解析出角度帧则更新缓存
 */
static void imu_parse_byte(uint8_t b)
{
    switch (s_state) {
    case PS_FIND_55:
        if (b == 0x55) {
            s_sum = 0x55;
            s_state = PS_TYPE;
        }
        break;

    case PS_TYPE:
        s_sum += b;
        s_frame_type = b;
        s_data_idx = 0;
        s_state = PS_DATA;
        break;

    case PS_DATA:
        s_data_buf[s_data_idx++] = b;
        s_sum += b;
        if (s_data_idx >= 8) {
            s_state = PS_SUM;
        }
        break;

    case PS_SUM:
        /* 校验和: SUM = (0x55 + type + Σdata[0..7]) & 0xFF
         * 必须严格校验, 否则数据区里的 0x55 会被误判为帧头,
         * 导致帧同步漂移 (0x51 加速度帧数据区常含 0x55,
         * 被误判后 0x53 角度帧的数据索引全错, yaw 解出来恒为 0) */
        if (b == (uint8_t)(s_sum & 0xFF)) {
            /* 校验通过, 是真帧 */
            if (s_frame_type == 0x53) {
                int16_t raw_r = (int16_t)(((uint16_t)s_data_buf[1] << 8) | s_data_buf[0]);
                int16_t raw_p = (int16_t)(((uint16_t)s_data_buf[3] << 8) | s_data_buf[2]);
                int16_t raw_y = (int16_t)(((uint16_t)s_data_buf[5] << 8) | s_data_buf[4]);
                s_roll_cached  = (float)raw_r / 32768.0f * 180.0f;
                s_pitch_cached = (float)raw_p / 32768.0f * 180.0f;
                s_yaw_cached   = (float)raw_y / 32768.0f * 180.0f;
                g_imu_present = 1;   /* 至少收到一帧, 标记在线 */
            }
        }
        /* 校验失败: 丢弃当前帧, 回找下一帧 0x55
         * (不更新缓存, 不置 g_imu_present, 等下一真帧) */
        s_state = PS_FIND_55;
        break;

    default:
        s_state = PS_FIND_55;
        break;
    }
}

/**
 * @brief 初始化: 等 200ms 看 UART_DEBUG 是否收到 JY61P 数据帧
 *        在等待期间持续 PollRx + 喂字节给状态机
 * @return 0=收到有效帧, 非0=超时未收到
 */
uint8_t IMU_Init(void)
{
    if (!g_use_imu) {
        /* 用户禁用 IMU, 不解析 */
        g_imu_present = 0;
        return 0xFF;
    }

    /* 重置状态机 */
    s_state = PS_FIND_55;
    s_data_idx = 0;
    g_imu_present = 0;

    /* 先把 UART_DEBUG RX FIFO 残留字节清空 */
    while (!DL_UART_Main_isRXFIFOEmpty(UART_DEBUG_INST)) {
        (void)DL_UART_Main_receiveData(UART_DEBUG_INST);
    }

    /* 等 1000ms 看是否收到有效帧 (JY61P 默认 10Hz=100ms/帧, 冷启动需 200-500ms)
     * MSPM0 UART FIFO 仅 4 字节, 9600bps 每 10ms 到 ~10 字节, 必须 ≤5ms 轮询 */
    for (uint16_t i = 0; i < 200; i++) {
        delay_ms(5);
        UART_Debug_PollRx();
        uint8_t b;
        while (UART_Debug_GetByte(&b)) {
            imu_parse_byte(b);
        }
        if (g_imu_present) {
            return 0;   /* 已收到帧 */
        }
    }
    return 0x01;   /* 超时未收到 */
}

/**
 * @brief 读取 Roll/Pitch/Yaw (度)
 *        直接返回缓存值 (不主动触发读取, 读取由 IMU_Poll 异步完成)
 * @return 0=有有效数据 (g_imu_present=1), 1=IMU未接或暂停
 */
uint8_t IMU_Read_RPY(float *roll, float *pitch, float *yaw)
{
    if (!g_imu_present || !g_use_imu) {
        if (roll)  *roll  = s_roll_cached;
        if (pitch) *pitch = s_pitch_cached;
        if (yaw)   *yaw   = s_yaw_cached;
        return 1;
    }
    if (roll)  *roll  = s_roll_cached;
    if (pitch) *pitch = s_pitch_cached;
    if (yaw)   *yaw   = s_yaw_cached;
    return 0;
}

/**
 * @brief 读取 yaw (度, -180~180), 返回缓存值
 */
float IMU_Read_Yaw(void)
{
    return s_yaw_cached;
}

/**
 * @brief Z轴归零: 通过串口发归零命令给 JY61P
 *        协议: 先解锁寄存器, 再发归零, 再保存
 * @return 0=已发送
 */
uint8_t IMU_Calibrate_Z(void)
{
    /* 解锁寄存器 */
    static const uint8_t unlock[] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
    UART_Debug_SendBytes(unlock, 5);
    delay_ms(200);

    /* Z 轴归零 */
    static const uint8_t calib[] = {0xFF, 0xAA, 0x76, 0x00, 0x00};
    UART_Debug_SendBytes(calib, 5);
    delay_ms(200);

    /* 保存到 Flash */
    static const uint8_t save[] = {0xFF, 0xAA, 0x00, 0x00, 0x00};
    UART_Debug_SendBytes(save, 5);
    delay_ms(200);

    /* 归零后清缓存 (JY61P 会在下一帧输出归零后的角度) */
    s_yaw_cached = 0.0f;
    return 0;
}

/**
 * @brief 主循环轮询: 把 UART_DEBUG 的字节喂给解析器
 *        @note 不在内部调 UART_Debug_PollRx, 由上层主循环统一调
 *              (因为 UART_Debug_PollRx 还要清 overrun, 集中调避免遗漏)
 */
void IMU_Poll(void)
{
    if (!g_use_imu) return;
    uint8_t b;
    /* 每次最多解析 32 字节, 防止突发数据卡住主循环 */
    uint8_t budget = 32;
    while (budget-- && UART_Debug_GetByte(&b)) {
        imu_parse_byte(b);
    }
}

/**
 * @brief 获取缓存的 yaw (度, -180~180)
 */
float IMU_Get_Yaw_Cached(void)
{
    return s_yaw_cached;
}
