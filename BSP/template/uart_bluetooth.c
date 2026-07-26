#include "uart_bluetooth.h"
#include "ti_msp_dl_config.h"

void UART_Bluetooth_Send(float *data, uint8_t count)
{
    uint8_t i;
    uint8_t *p;
    for (i = 0; i < count; i++) {
        p = (uint8_t *)(&data[i]);
        DL_UART_Main_transmitDataBlocking(UART_BLUETOOTH_INST, p[0]);
        DL_UART_Main_transmitDataBlocking(UART_BLUETOOTH_INST, p[1]);
        DL_UART_Main_transmitDataBlocking(UART_BLUETOOTH_INST, p[2]);
        DL_UART_Main_transmitDataBlocking(UART_BLUETOOTH_INST, p[3]);
    }
    /* JustFloat 帧尾 */
    DL_UART_Main_transmitDataBlocking(UART_BLUETOOTH_INST, 0x00);
    DL_UART_Main_transmitDataBlocking(UART_BLUETOOTH_INST, 0x00);
    DL_UART_Main_transmitDataBlocking(UART_BLUETOOTH_INST, 0x80);
    DL_UART_Main_transmitDataBlocking(UART_BLUETOOTH_INST, 0x7f);
}