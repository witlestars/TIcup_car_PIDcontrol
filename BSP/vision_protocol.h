/**
 * @file    vision_protocol.h
 * @brief   视觉通信协议 (K230 -> TI 主控)
 * 
 * 通信方式: 串口 (UART)
 * 波特率: 115200[cite: 11]
 * 数据格式: $O<position_01mm>,<velocity_mm_s>,<confidence>,<flags>,<age_ms>,<seq><F|L>*<XOR_HEX>[cite: 11]
 * 示例: $O473,126,218,5,35,38F*23[cite: 11]
 * 
 * 帧尾无附加回车或换行，校验和后立刻结束[cite: 11]。
 */

#ifndef __VISION_PROTOCOL_H
#define __VISION_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* 视觉数据缓冲区大小 */
#define VISION_RX_BUF_SIZE  64

/* 视觉状态位掩码定义[cite: 11] */
#define VISION_FLAG_VALID       0x01  /* 当前视觉位置有效 */
#define VISION_FLAG_YOLO        0x02  /* 结果经过YOLO检测确认或纠偏 */
#define VISION_FLAG_TRACK       0x04  /* 结果由NanoTracker跟踪得到 */
#define VISION_FLAG_HOLD        0x08  /* 当前帧漏检，仅保留上次结果 */
#define VISION_FLAG_NEAR_END    0x10  /* 钢球接近摆杆端部 */

typedef enum
{
    VISION_WAIT_HEAD = 0,
    VISION_RECEIVE_DATA,
    VISION_RECEIVE_CHECKSUM
} VisionRxState_e;

/* 视觉通信协议结构体 */
typedef struct {
    uint8_t rx_buf[VISION_RX_BUF_SIZE];
    uint8_t rx_checksum_buf[2]; /* 校验和缓冲区 */
    uint16_t rx_index;
    uint8_t checksum_count; /* 校验和计数 */

    /* 解析后的目标数据[cite: 11] */
    int16_t  position_01mm;  /* 相对中心O点位置，0.1mm单位 -562就是56.2mm */
    int16_t  velocity_mm_s;  /* 速度 mm/s */
    uint8_t  confidence;     /* 置信度 0~255 */
    uint8_t  flags;          /* 状态掩码 */
    uint16_t age_ms;         /* 数据年龄 ms */
    uint8_t  seq;            /* 帧序号 0~255 */
    
    uint8_t  target_found;   /* 是否可用: 1=正常闭环可用(F且VALID), 0=不可用(L或HOLD)[cite: 11] */

    VisionRxState_e rx_state; /* 接收状态机 */
    
    // 记录成功解析到这一帧时的系统滴答时间
    uint32_t last_update_tick;

} Vision_ProtocolTypeDef;

// 视觉数据全局变量
extern volatile Vision_ProtocolTypeDef g_vision_data;

extern volatile uint32_t g_sys_tick;

/**
 * @brief  初始化视觉通信
 */
void Vision_Init();

/**
 * @brief  视觉数据解析 (在串口接收中断中逐字节调用)
 */
uint8_t Vision_ParseByte(uint8_t data);

#endif /* __VISION_PROTOCOL_H */

