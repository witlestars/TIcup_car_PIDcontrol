// /**
//  * 视觉通信协议实现
//  * 协议: $OX,YF*CS  (只有一个*,校验和后直接结束)
//  */

// #include "vision_protocol.h"
// #include <stdlib.h>
// #include <stdint.h>
// #include <string.h>
// #include <ctype.h>

// // 计算字节数组的 XOR 校验和
// static uint8_t Vision_CalcXor(const uint8_t *data, uint16_t len) {
//     uint8_t checksum = 0;
//     for (uint16_t i = 0; i < len; i++) {
//         checksum ^= data[i];
//     }
//     return checksum;
// }

// // 将两个十六进制字符转换为字节（0~255），无效返回 0xFF
// static uint8_t Vision_HexToByte(uint8_t high, uint8_t low) {
//     uint8_t val = 0;
//     if (high >= '0' && high <= '9') val = (high - '0') << 4;
//     else if (high >= 'A' && high <= 'F') val = (high - 'A' + 10) << 4;
//     else if (high >= 'a' && high <= 'f') val = (high - 'a' + 10) << 4;
//     else return 0xFF;

//     if (low >= '0' && low <= '9') val |= (low - '0');
//     else if (low >= 'A' && low <= 'F') val |= (low - 'A' + 10);
//     else if (low >= 'a' && low <= 'f') val |= (low - 'a' + 10);
//     else return 0xFF;

//     return val;
// }

// // 解析 payload 内容，例如 "O320,240F" 或 "O0,0L"
// static void Vision_ProcessPayload(Vision_ProtocolTypeDef *vision,
//                                   const uint8_t *payload,
//                                   uint16_t len)
// {
//     if (len == 0 || payload[0] != 'O')
//         return;

//     char buf[32];

//     if (len >= sizeof(buf))
//         len = sizeof(buf) - 1;

//     memcpy(buf, payload, len);
//     buf[len] = '\0';

//     char *p = buf + 1;

//     // 解析 x
//     int x = atoi(p);

//     // 找 ','
//     p = strchr(p, ',');
//     if (p == NULL)
//         return;

//     p++;

//     // 解析 y
//     int y = atoi(p);

//     // 找 F/L
//     while (*p && *p != 'F' && *p != 'L')
//         p++;

//     if (*p == 'F')
//     {
//         vision->target_found = 1;
//         vision->target_x = x;
//         vision->target_y = y;
//     }
//     else if (*p == 'L')
//     {
//         vision->target_found = 0;
//     }
// }



// void Vision_Init(Vision_ProtocolTypeDef *vision)
// {
//     vision->rx_index = 0;
//     vision->target_x = 0;
//     vision->target_y = 0;
//     vision->target_found = 0;
//     vision->rx_state = VISION_WAIT_HEAD;
//     vision->checksum_count = 0;
//     memset(vision->rx_buf, 0, sizeof(vision->rx_buf));
//     memset(vision->rx_checksum_buf, 0, sizeof(vision->rx_checksum_buf));
// }


// uint8_t Vision_ParseByte(Vision_ProtocolTypeDef *vision, uint8_t data)
// {
//     switch (vision->rx_state)
//     {
//     case VISION_WAIT_HEAD:// 等待帧头
//         if (data == '$')
//         {
//             vision->rx_state = VISION_RECEIVE_DATA;
//             vision->rx_index = 0;
//         }
//         break;
//     case VISION_RECEIVE_DATA:// 接收数据
//         if (data == '*')
//         {
//             vision->rx_state = VISION_RECEIVE_CHECKSUM;
//             vision->checksum_count = 0;
//         }
//         else if (vision->rx_index < VISION_RX_BUF_SIZE)
//         {
//             vision->rx_buf[vision->rx_index++] = data;
//         }
//         else
//         { // 数据过长，重置状态
//             vision->rx_state = VISION_WAIT_HEAD;
//         }
//         break;

//     case VISION_RECEIVE_CHECKSUM:// 接收校验和
//         vision->rx_checksum_buf[vision->checksum_count++] = data;
        
//         // 立刻原地解析
//         if(vision->checksum_count == 2) 
//         {
//             // 1. 计算 payload 的 XOR 校验和
//             uint8_t calc_checksum = Vision_CalcXor(vision->rx_buf, vision->rx_index);
            
//             // 2. 将收到的两个十六进制字符转换成字节
//             uint8_t recv_checksum = Vision_HexToByte(vision->rx_checksum_buf[0],
//                                                      vision->rx_checksum_buf[1]);

//             // 3. 比较
//             if (recv_checksum != 0xFF && calc_checksum == recv_checksum)
//             {
//                 // 校验成功，解析 payload
//                 Vision_ProcessPayload(vision, vision->rx_buf, vision->rx_index);
//             }
            
//             // 原地复位状态机，下一秒来的任何字符都将作为新的一帧开始！
//             vision->rx_state = VISION_WAIT_HEAD;
//             vision->rx_index = 0;
//             vision->checksum_count = 0;
//         }
//         break;
    
//     }
//     return 0;
// }




