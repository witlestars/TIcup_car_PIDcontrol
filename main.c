/**
 * @file    main.c
 * @brief   板球系统主程序
 */

#include "ti_msp_dl_config.h"
#include "balance.h"

/* SysTick 1ms 时基[cite: 2] */
volatile uint32_t g_sys_tick = 0;
void SysTick_Handler(void) { g_sys_tick++; }

int main(void)
{
    /* 1. 初始化系统外设[cite: 2] */
    SYSCFG_DL_init();
    
    /* 2. 使能全局中断[cite: 2] */
    __enable_irq();

    /* 3. 初始化平衡控制系统 (内部会自动完成步进电机的初始化和使能) */
    Balance_Init();

    uint32_t last_10ms = 0;
    
    ///////////////////* 主循环中只应出现task函数 *///////////////////

    while (1) {
        /* 10ms 控制节拍 */
        if ((uint32_t)(g_sys_tick - last_10ms) >= 10) {
            last_10ms = g_sys_tick;

            /* 执行循迹 PID 控制任务 */
            

            /* 执行平衡 PID 控制任务 */
            // Balance_Task(target_pos, current_pos,小车前向加速度前馈);
        }
    }
}

///////////////////* 需要中断运行的任务函数统一写在后面 *///////////////////

// IMU数据解析任务   数据更新在全局变量g_imu_data中
void UART_IMU_INST_IRQHandler(void)
{
    // 获取当前触发的是什么中断
    uint32_t pending_irq = DL_UART_Main_getPendingInterrupt(UART_IMU_INST);
    // 正常的接收中断
    if (pending_irq == DL_UART_IIDX_RX) 
    {
        // 只要 FIFO 里有数据，就一直读，防止残留数据导致堵塞
        while (DL_UART_Main_isRXFIFOEmpty(UART_IMU_INST) == false) 
        {
            uint8_t rx_data = DL_UART_Main_receiveData(UART_IMU_INST);
            IMU_UART_ParseByte(rx_data);
        }
    }
    // 溢出、帧错误、校验错误
    else if ((pending_irq == DL_UART_IIDX_OVERRUN_ERROR) ||
             (pending_irq == DL_UART_IIDX_BREAK_ERROR) ||
             (pending_irq == DL_UART_IIDX_FRAMING_ERROR) ||
             (pending_irq == DL_UART_IIDX_PARITY_ERROR))
    {
        // 发生错误时，必须清除标志位防止死锁！
        DL_UART_Main_clearInterruptStatus(UART_IMU_INST, 
            (DL_UART_INTERRUPT_OVERRUN_ERROR | 
             DL_UART_INTERRUPT_BREAK_ERROR | 
             DL_UART_INTERRUPT_FRAMING_ERROR | 
             DL_UART_INTERRUPT_PARITY_ERROR));
    }
}
