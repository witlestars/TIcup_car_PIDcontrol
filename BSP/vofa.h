#ifndef __VOFA_H
#define __VOFA_H

#include "ti_msp_dl_config.h"
// 请确保此处包含你定义 x_pid, y_pid 的头文件
#include "balance.h"

// 声明外部的 PID 变量 (根据你的实际命名修改)
extern volatile Balance_PID_t g_balance_pid;

extern volatile bool g_vofa_tx_busy;

void VOFA_SendWaveData(float target, float current, float output);
void VOFA_ParseCommand(uint8_t *rx_buf, uint16_t len);

// 用于在 UART RX 中断里逐字节收集数据的函数
void VOFA_RX_Byte_Callback(uint8_t rx_data);

// 在主循环中处理已经接收完整的调参命令
void VOFA_CommandTask(void);

// 【新增】专门用于发送调试字符串的函数
void VOFA_SendString(const char *str);

#endif
