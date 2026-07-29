/**
 * @file    k230.c
 * @brief   K230 题号发送实现 (UART_K230 TX, 视觉 RX 由 vision_protocol.c 处理)
 */

#include "k230.h"
#include "ti_msp_dl_config.h"
#include <string.h>
#include <stdio.h>

/* 发 N 字节 (非阻塞带超时, TX FIFO 满跳过, 防卡死) */
void K230_SendText(const char *text)
{
    const char *p = text;
    while (*p) {
        uint16_t timeout = 60000;
        while (DL_UART_Main_isTXFIFOFull(UART_K230_INST) && timeout--) ;
        if (timeout == 0) break;   /* TX FIFO 持续满, 放弃避免卡死 */
        DL_UART_Main_transmitDataBlocking(UART_K230_INST, (uint8_t)*p);
        p++;
    }
}

void K230_SendTask(uint8_t task_id)
{
    if (task_id < 1) task_id = 1;
    if (task_id > 5) task_id = 5;
    char buf[8];
    snprintf(buf, sizeof(buf), "T%d\n", task_id);
    K230_SendText(buf);
}
