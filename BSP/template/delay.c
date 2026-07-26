#include "delay.h"
#include "ti_msp_dl_config.h"

void delay_ms(int ms)
{
    for (int i=0;i<100;i++)
        delay_cycles(320000/1000*ms);
}

void delay_us(int us)
{
    delay_cycles(32 * us);  /* 32MHz: 32 cycles = 1μs */
}