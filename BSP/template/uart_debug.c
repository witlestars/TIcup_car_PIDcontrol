#include "uart_debug.h"
#include "ti_msp_dl_config.h"

/*
 * VOFA+ JustFloat 协议: 原始 float 字节流 (小端序, ARM 原生)
 * 在 VOFA+ 中: 左上角协议选 JustFloat, 手动设置通道数=发送的 float 数量
 *
 * SysConfig 配置步骤:
 *   1. 打开 empty.syscfg → 左侧点 "+" (ADD)
 *   2. 搜索 "UART" → 选 UART (不是 UART_Extend)
 *   3. 命名为 "UART_DEBUG"
 *   4. TX 引脚: 选 PA11 (UART1_TX, 空闲引脚)
 *   5. RX 引脚: 不勾选 (我们不需要接收)
 *   6. 波特率: 115200
 *   7. 保存 → SysConfig 自动生成 SYSCFG_DL_UART_DEBUG_init()
 *
 * HC-05 蓝牙准备:
 *   默认波特率 9600, 需要用 USB-TTL + AT 命令改成 115200:
 *   AT+UART=115200,0,0
 */

void UART_Debug_Send(float *data, uint8_t count)
{
    uint8_t i;
    uint8_t *p;
    for (i = 0; i < count; i++) {
        p = (uint8_t *)(&data[i]);
        DL_UART_Main_transmitDataBlocking(UART_DEBUG_INST, p[0]);
        DL_UART_Main_transmitDataBlocking(UART_DEBUG_INST, p[1]);
        DL_UART_Main_transmitDataBlocking(UART_DEBUG_INST, p[2]);
        DL_UART_Main_transmitDataBlocking(UART_DEBUG_INST, p[3]);
    }
    /* JustFloat 帧尾 */
    DL_UART_Main_transmitDataBlocking(UART_DEBUG_INST, 0x00);
    DL_UART_Main_transmitDataBlocking(UART_DEBUG_INST, 0x00);
    DL_UART_Main_transmitDataBlocking(UART_DEBUG_INST, 0x80);
    DL_UART_Main_transmitDataBlocking(UART_DEBUG_INST, 0x7f);
}
