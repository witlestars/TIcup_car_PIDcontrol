/**
 * @file    vision_protocol.c
 * @brief   视觉通信协议解析实现
 */

#include "vision_protocol.h"
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "ti_msp_dl_config.h"
#include "imu.h"


#include "ZDT_X42S_Driver.h"
#include "balance.h"

volatile Vision_ProtocolTypeDef g_vision_data;

// 计算字节数组的 XOR 校验和[cite: 4, 6]
static uint8_t Vision_CalcXor(const uint8_t *data, uint16_t len) {
    uint8_t checksum = 0;
    for (uint16_t i = 0; i < len; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

// 将两个十六进制字符转换为字节（0~255），无效返回 0xFF[cite: 4]
static uint8_t Vision_HexToByte(uint8_t high, uint8_t low) {
    uint8_t val = 0;
    if (high >= '0' && high <= '9') val = (high - '0') << 4;
    else if (high >= 'A' && high <= 'F') val = (high - 'A' + 10) << 4;
    else if (high >= 'a' && high <= 'f') val = (high - 'a' + 10) << 4;
    else return 0xFF;

    if (low >= '0' && low <= '9') val |= (low - '0');
    else if (low >= 'A' && low <= 'F') val |= (low - 'A' + 10);
    else if (low >= 'a' && low <= 'f') val |= (low - 'a' + 10);
    else return 0xFF;

    return val;
}

// 解析 payload 内容
// 例如: "O473,126,218,5,35,38F"[cite: 6]
static void Vision_ProcessPayload( const uint8_t *payload,uint16_t len)
{
    if (len == 0 || payload[0] != 'O')
        return;

    char buf[VISION_RX_BUF_SIZE];

    if (len >= sizeof(buf))
        len = sizeof(buf) - 1;

    memcpy(buf, payload, len);
    buf[len] = '\0';

    char *p = buf + 1; // 跳过 'O'[cite: 6]

    // 1. 解析 position_01mm[cite: 6]
    g_vision_data.position_01mm = atoi(p);
    p = strchr(p, ','); if (p == NULL) return; p++;

    // 2. 解析 velocity_mm_s[cite: 6]
    g_vision_data.velocity_mm_s = atoi(p);
    p = strchr(p, ','); if (p == NULL) return; p++;

    // 3. 解析 confidence[cite: 6]
    g_vision_data.confidence = atoi(p);
    p = strchr(p, ','); if (p == NULL) return; p++;

    // 4. 解析 flags[cite: 6]
    g_vision_data.flags = atoi(p);
    p = strchr(p, ','); if (p == NULL) return; p++;

    // 5. 解析 age_ms[cite: 6]
    g_vision_data.age_ms = atoi(p);
    p = strchr(p, ','); if (p == NULL) return; p++;

    // 6. 解析 seq[cite: 6]
    g_vision_data.seq = atoi(p);

    // 7. 找 F/L 状态后缀[cite: 6]
    while (*p && *p != 'F' && *p != 'L') {
        p++;
    }

    // 综合判定目标是否可用于闭环: 后缀为'F' 且 VALID位置位(bit0为1)[cite: 6]
    if (*p == 'F' && (g_vision_data.flags & VISION_FLAG_VALID))
    {
        g_vision_data.target_found = 1;
    }
    else
    {
        // 包含 'L' 或仅为 HOLD 帧等情况，不可作正常测量值[cite: 6]
        g_vision_data.target_found = 0; 
    }
}

void Vision_Init()
{
    g_vision_data.rx_index = 0;
    g_vision_data.position_01mm = 0;
    g_vision_data.velocity_mm_s = 0;
    g_vision_data.confidence = 0;
    g_vision_data.flags = 0;
    g_vision_data.age_ms = 0;
    g_vision_data.seq = 0;
    g_vision_data.target_found = 0;
    
    g_vision_data.rx_state = VISION_WAIT_HEAD;
    g_vision_data.checksum_count = 0;
    
    memset(g_vision_data.rx_buf, 0, sizeof(g_vision_data.rx_buf));
    memset(g_vision_data.rx_checksum_buf, 0, sizeof(g_vision_data.rx_checksum_buf));
    
    NVIC_EnableIRQ(UART_K230_INST_INT_IRQN);
}

uint8_t Vision_ParseByte(uint8_t data)
{
    uint8_t parse_success = 0; // 默认解析未完成/失败

    switch (g_vision_data.rx_state)
    {
    case VISION_WAIT_HEAD: // 等待帧头 '$'[cite: 6]
        if (data == '$')
        {
            g_vision_data.rx_state = VISION_RECEIVE_DATA;
            g_vision_data.rx_index = 0;
            g_vision_data.checksum_count = 0;
        }
        break;

    case VISION_RECEIVE_DATA: // 接收数据载荷[cite: 6]
        if (data == '*') // 遇到 '*' 分隔符，进入校验和接收[cite: 6]
        {
            g_vision_data.rx_state = VISION_RECEIVE_CHECKSUM;
            g_vision_data.checksum_count = 0;
        }
        else if (g_vision_data.rx_index < VISION_RX_BUF_SIZE)
        {
            g_vision_data.rx_buf[g_vision_data.rx_index++] = data;
        }
        else
        { 
            // 数据过长，重置状态防溢出
            g_vision_data.rx_state = VISION_WAIT_HEAD;
        }
        break;

    case VISION_RECEIVE_CHECKSUM: // 接收 2位 HEX 校验和[cite: 6]
        if (g_vision_data.checksum_count < 2) // 防越界保护
        {
            g_vision_data.rx_checksum_buf[g_vision_data.checksum_count++] = data;
        }
        
        // 收到第2个校验字符后，直接原地解析并强制结束当前帧[cite: 4]
        if(g_vision_data.checksum_count == 2) 
        {
            // 1. 计算 payload 的 XOR 校验和[cite: 6]
            uint8_t calc_checksum = Vision_CalcXor(g_vision_data.rx_buf, g_vision_data.rx_index);
            
            // 2. 将收到的两个十六进制字符转换成字节[cite: 6]
            uint8_t recv_checksum = Vision_HexToByte(g_vision_data.rx_checksum_buf[0],
                                                     g_vision_data.rx_checksum_buf[1]);

            // 3. 比较验证[cite: 6]
            if (recv_checksum != 0xFF && calc_checksum == recv_checksum)
            {
                // 校验成功，解析 payload[cite: 6]
                Vision_ProcessPayload( g_vision_data.rx_buf, g_vision_data.rx_index);
                parse_success = 1;
                // 记录当前系统时间 (ms)
                g_vision_data.last_update_tick = g_sys_tick;
            }
            
            // 协议规定校验和后直接结束（无换行/其他结束符），必须复位状态机[cite: 4, 6]
            g_vision_data.rx_state = VISION_WAIT_HEAD;
            g_vision_data.rx_index = 0;
            g_vision_data.checksum_count = 0;
        }
        break;
        
    default:
        g_vision_data.rx_state = VISION_WAIT_HEAD;
        break;
    }
    
    return parse_success; // 返回解析状态[cite: 5]
}
