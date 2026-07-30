/**
 * @file    main.c
 * @brief   板球系统主程序
 */

#include "ti_msp_dl_config.h"
#include "task.h"
#include "balance.h"
#include "track.h"
#include "imu.h"
#include "oled.h"

/* SysTick 1ms 时基[cite: 2] */
volatile uint32_t g_sys_tick = 0;
void SysTick_Handler(void) { g_sys_tick++; }

int main(void)
{
    /* 1. 初始化系统外设[cite: 2] */
    SYSCFG_DL_init();

    /* 2. 使能全局中断[cite: 2] */
    __enable_irq();
    NVIC_EnableIRQ(GPIOA_INT_IRQn);
    NVIC_EnableIRQ(GPIOB_INT_IRQn);

    /* 3.模块初始化 */
    // IMU_Init();
    OLED_Init();

    /* 4. 初始化任务 (内部会自动完成底层里程计和电机的初始化和使能) */
    Track_Init();
    // Balance_Init();

    uint32_t last_10ms = 0;
    uint32_t last_100ms = 0;

    ///////////////////* 主循环中只应出现在task.c中定义的任务 *///////////////////
    /* 先看task.c！！！！！其中已留好任务切换的标志位 */
    while (1)
    {
        /* 10ms 控制节拍 */
        if ((uint32_t)(g_sys_tick - last_10ms) >= 10)
        {
            last_10ms = g_sys_tick;

            /* 执行循迹控制任务 */
            Chassis_Task();

            /* 执行平衡控制任务 */
            // Balance_Task(g_balance_task, g_running);
        }

        if ((uint32_t)(g_sys_tick - last_100ms) >= 100)
        {
            last_100ms = g_sys_tick;
            OLED_Task();
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

// 视觉数据解析任务   数据更新在全局变量g_vision中
void UART_K230_INST_IRQHandler(void)
{
    // 获取当前触发的是什么中断
    uint32_t pending_irq = DL_UART_Main_getPendingInterrupt(UART_K230_INST);
    // 正常的接收中断
    if (pending_irq == DL_UART_IIDX_RX)
    {
        // 只要 FIFO 里有数据，就一直读，防止残留数据导致堵塞
        while (DL_UART_Main_isRXFIFOEmpty(UART_K230_INST) == false)
        {
            // uint8_t rx_data = DL_UART_Main_receiveData(UART_K230_INST);
            // Vision_ParseByte(&g_vision, rx_data);
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

// 按键中断处理函数 (统一入口)
/*
    同过中断改变标志位，在主循环中传参来切换模式
*/
/* 先看task.c！！！！！其中已留好任务切换的标志位 */
#define DEBOUNCE_TIME_MS 200
// 这是 GPIO 统一的硬件中断入口函数
// 按键中断处理函数
void GROUP1_IRQHandler(void)
{
    // 每个按键建议用独立的防抖时间戳，防止互相干扰
    static uint32_t last_time_start = 0;
    static uint32_t last_time_chassis = 0;
    static uint32_t last_time_balance = 0;
    static uint32_t last_time_end = 0;

    // ====== 循环检查并清除 GPIOA 端口的所有按键标志位 ======
    uint32_t gpioA_status;
    while ((gpioA_status = DL_GPIO_getPendingInterrupt(GPIOA)) != 0)
    {
        if (gpioA_status == GPIO_BUTTON_BTN_3_IIDX)
        {
            if ((g_sys_tick - last_time_balance) > DEBOUNCE_TIME_MS)
            {
                g_balance_task++;
                if (g_balance_task >= 4)
                    g_balance_task = 0;
                last_time_balance = g_sys_tick;
            }
        }
        else if (gpioA_status == GPIO_BUTTON_BTN_1_IIDX)
        {
            if ((g_sys_tick - last_time_end) > DEBOUNCE_TIME_MS)
            {
                g_running = false;
                last_time_balance = g_sys_tick;
            }
        }
    }

    // ====== 循环检查并清除 GPIOB 端口的所有按键标志位 ======
    uint32_t gpioB_status;
    while ((gpioB_status = DL_GPIO_getPendingInterrupt(GPIOB)) != 0)
    {
        if (gpioB_status == GPIO_BUTTON_BTN_4_IIDX)
        {
            if ((g_sys_tick - last_time_start) > DEBOUNCE_TIME_MS)
            {
                g_running = true;
                last_time_start = g_sys_tick;
            }
        }
        else if (gpioB_status == GPIO_BUTTON_BTN_2_IIDX)
        {
            if ((g_sys_tick - last_time_chassis) > DEBOUNCE_TIME_MS)
            {
                g_chassis_task++;
                if (g_chassis_task >= 3)
                    g_chassis_task = 0;
                last_time_chassis = g_sys_tick;
            }
        }
    }
}
