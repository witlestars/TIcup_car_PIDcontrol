#ifndef __UART_DEBUG_H
#define __UART_DEBUG_H

#include <stdint.h>

/**
 * @brief 通过 UART 发送 float 数组 (VOFA+ JustFloat 协议)
 * @param data   float 数组指针
 * @param count  数组长度
 * @note  在 VOFA+ 中配置: 协议=JustFloat, 通道数=count
 *        需要先在 SysConfig 中添加 UART 模块, 命名为 "UART_DEBUG"
 */
void UART_Debug_Send(float *data, uint8_t count);

#endif
