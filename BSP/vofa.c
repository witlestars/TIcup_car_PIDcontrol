#include "vofa.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// VOFA JustFloat 协议的固定帧尾
const uint8_t vofa_tail[4] = {0x00, 0x00, 0x80, 0x7f};

// DMA 发送缓冲区必须是全局的，确保 DMA 传输期间内存有效
uint8_t vofa_tx_buffer[16]; 

// 用于接收中断拼包的缓冲区和索引
char vofa_rx_buffer[64];
uint16_t vofa_rx_index = 0;
static char vofa_cmd_buffer[64];
static volatile uint16_t vofa_cmd_len = 0;
static volatile bool vofa_cmd_ready = false;

volatile bool g_vofa_tx_busy = false;

/**
 * @brief  向 VOFA+ 发送 JustFloat 协议波形数据
 */
void VOFA_SendWaveData(float target, float current, float output)
{
    // DMA 忙时丢弃本帧，避免覆盖 DMA 正在读取的缓冲区。
    if (g_vofa_tx_busy)
    {
        return;
    }

    g_vofa_tx_busy = true;

    float tx_data[3];
    tx_data[0] = target;
    tx_data[1] = current;
    tx_data[2] = output;
    
    // 组装数据和帧尾
    memcpy(vofa_tx_buffer, tx_data, sizeof(tx_data));
    memcpy(vofa_tx_buffer + 12, vofa_tail, 4);

    // 1. 配置前必须先关闭 DMA 通道
    DL_DMA_disableChannel(DMA, DMA_CH3_CHAN_ID);
    
    // 2. 设置源地址 (你的内存数组)
    DL_DMA_setSrcAddr(DMA, DMA_CH3_CHAN_ID, (uint32_t)vofa_tx_buffer);
    
    // 3. 设置目标地址 (UART 外设的发送寄存器)
    DL_DMA_setDestAddr(DMA, DMA_CH3_CHAN_ID, (uint32_t)(&UART_VOFA_INST->TXDATA));
    
    // 4. 设置发送长度
    DL_DMA_setTransferSize(DMA, DMA_CH3_CHAN_ID, 16);
    
    // 5. 重新开启通道，开始搬运
    DL_DMA_enableChannel(DMA, DMA_CH3_CHAN_ID);
}

/**
 * @brief  单字节接收回调 (放在中断中调用)
 * @param  rx_data 串口收到的一个字节
 */
void VOFA_RX_Byte_Callback(uint8_t rx_data)
{
    // 防止缓冲区溢出
    if (vofa_rx_index < sizeof(vofa_rx_buffer) - 1)
    {
        vofa_rx_buffer[vofa_rx_index++] = rx_data;
        
        // VOFA 发送的命令通常以回车换行符 '\n' 结尾
        if (rx_data == '\n') 
        {
            if (!vofa_cmd_ready)
            {
                memcpy(vofa_cmd_buffer, vofa_rx_buffer, vofa_rx_index);
                vofa_cmd_buffer[vofa_rx_index] = '\0';
                vofa_cmd_len = vofa_rx_index;
                vofa_cmd_ready = true;
            }
            vofa_rx_index = 0; // 解析完成后清零索引，准备下一次接收
        }
    }
    else
    {
        vofa_rx_index = 0; // 溢出则丢弃并重置
    }
}

void VOFA_CommandTask(void)
{
    uint16_t len;

    if (!vofa_cmd_ready)
    {
        return;
    }

    len = vofa_cmd_len;
    VOFA_ParseCommand((uint8_t *)vofa_cmd_buffer, len);
    vofa_cmd_ready = false;
}

/**
 * @brief  解析 VOFA+ 发送的调参字符串 (例如: "XP=1.5\n")
 */
void VOFA_ParseCommand(uint8_t *rx_buf, uint16_t len)
{
    char *cmdStr = (char *)rx_buf;

    // 严谨校验：确保是正常的可见字符
    if (len == 0 || cmdStr[0] < 32 || cmdStr[0] > 126) 
    {
        return;
    }

    // ================= 解析 X 轴参数 =================
    // 强制使用 strncmp + atof，避开 sscanf 的浮点坑
    if (strncmp(cmdStr, "PP=", 3) == 0) 
    {
        g_pos_pid.Kp = atof(cmdStr + 3);
        // VOFA_SendString("-> Success: Kp updated!\n");
    } 
    else if (strncmp(cmdStr, "PI=", 3) == 0) 
    {
        g_pos_pid.Ki = atof(cmdStr + 3);
        // VOFA_SendString("-> Success: Ki updated!\n");
    } 
    else if (strncmp(cmdStr, "PD=", 3) == 0) 
    {
        g_pos_pid.Kd = atof(cmdStr + 3);
        // VOFA_SendString("-> Success: Kd updated!\n");
    }
    else if (strncmp(cmdStr, "PF=", 3) == 0)
    {
        g_pos_pid.Kff = atof(cmdStr + 3);
        // VOFA_SendString("-> Success: Kff updated!\n");
    }
    else 
    {
        // VOFA_SendString("-> Error: Header not matched!\n");
    }
}


// void DMA_IRQHandler(void)
// {
//     /* 检查是否是你配置给串口 TX 的 DMA 通道（例如 Channel 0）产生的中断 */
//     switch (DL_DMA_getPendingInterrupt(DMA)) {
//         case DL_DMA_EVENT_IIDX_DMACH0:  // 假设 DMA CH0 是给电机串口用的
            
//             /* 清除中断标志 */
//             DL_DMA_clearInterruptStatus(DMA, DL_DMA_INTERRUPT_CHANNEL0);
            
//             /* 解除忙碌状态，允许下一次 Balance_Task 下发指令 */
//             g_vofa_tx_busy = false; 
            
//             break;
            
//         // 如果有 K230 或陀螺仪的 DMA 接收通道，在此处继续添加 case
//         // case DL_DMA_EVENT_IIDX_DMACH1: ...
        
//         default:
//             break;
//     }
// }

/**
 * @brief  向 VOFA+ 发送字符串 (用于文本视图调试)
 * @param  str 要发送的字符串指针
 */
void VOFA_SendString(const char *str)
{
    // 如果 DMA 还在发波形，等待其发完，防止数据交叉乱码
    while(g_vofa_tx_busy); 

    // 逐字节阻塞发送字符串
    while (*str != '\0')
    {
        DL_UART_Main_transmitDataBlocking(UART_VOFA_INST, (uint8_t)(*str));
        str++;
    }
}
