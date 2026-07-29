// /**
//  * 视觉通信协议
//  * 用于接收视觉发送的目标位置数据
//  * 通信方式: 串口 (UART)
//  * 波特率: 115200
//  * 数据格式: $OX,YF*CS*
//  * $ - 帧头
//  * O - 标识符
//  * X - X坐标 (0-799)
//  * , - 分隔符
//  * Y - Y坐标 (0-479)
//  * F - 状态标识 (F=找到目标, L=丢失)
//  * * - 分隔符
//  * CS - 校验和 (2位十六进制, XOR)
//  * * - 结束符
//  *
//  * 示例:
//  * $O120,50F*2A*  目标在(120,50), 已找到, 校验和0x2A
//  * $O0,0L*00*    目标丢失, 校验和0x00
//  */

// #ifndef __VISION_PROTOCOL_H
// #define __VISION_PROTOCOL_H

// #ifdef __cplusplus
// extern "C" {
// #endif

// #include <stdint.h>
// #include <stdbool.h>

// /* 视觉数据缓冲区大小 */
// #define VISION_RX_BUF_SIZE  32

// typedef enum
// {
//     VISION_WAIT_HEAD = 0,
//     VISION_RECEIVE_DATA,
//     VISION_RECEIVE_CHECKSUM,
//     VISION_RECEIVE_END,
// } VisionRxState_e;

// /* 视觉通信协议 */
// typedef struct {
//     uint8_t rx_buf[VISION_RX_BUF_SIZE];
//     uint8_t rx_checksum_buf[2]; /* 校验和缓冲区 */
//     uint16_t rx_index;
//     uint8_t checksum_count; /* 校验和计数 */

//     /* 解析后的目标数据 */
//     int16_t target_x;        /* 目标X坐标 (像素) */
//     int16_t target_y;        /* 目标Y坐标 (像素) */
//     uint8_t target_found;    /* 目标是否找到: 1=找到, 0=丢失 */

//     VisionRxState_e rx_state; /* 接收状态 */

// } Vision_ProtocolTypeDef;


// /**
//  * @brief  初始化视觉通信
//  * @param  vision: 视觉协议结构体
//  */
// void Vision_Init(Vision_ProtocolTypeDef *vision);

// /**
//  * @brief  视觉数据解析 (在串口接收中断中调用)
//  * @param  vision: 视觉协议结构体
//  * @param  data: 接收到的字节
//  * @retval 1=解析成功, 0=继续接收
//  */
// uint8_t Vision_ParseByte(Vision_ProtocolTypeDef *vision, uint8_t data);

// #endif /* __VISION_PROTOCOL_H */

