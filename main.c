/**
 * @file    main.c
 * @brief   VOFA+ 通信测试主程序
 */

#include "ti_msp_dl_config.h"
#include "task.h"
#include "balance.h"
#include "track.h"
#include "imu.h"
#include "oled.h"
#include "vision_protocol.h"

#include <stdio.h>

/* 必须包含 VOFA 头文件才能调用发送函数 */
#include "vofa.h" 


/* SysTick 1ms 时基[cite: 9] */
volatile uint32_t g_sys_tick = 0;
void SysTick_Handler(void) { g_sys_tick++; }

int main(void)
{
    /* 1. 初始化系统外设[cite: 9] */
    SYSCFG_DL_init();

    /* 2. 使能全局中断[cite: 9] */
    __enable_irq();
    NVIC_EnableIRQ(GPIOA_INT_IRQn);
    NVIC_EnableIRQ(GPIOB_INT_IRQn);
    NVIC_EnableIRQ(DMA_INT_IRQn);
    NVIC_EnableIRQ(UART_VOFA_INST_INT_IRQN);

    /* 3. 模块初始化 (测试 VOFA，暂时屏蔽其余外设)[cite: 9] */
    // IMU_Init();
    // OLED_Init();

    /* 4. 初始化任务 (测试 VOFA，暂时屏蔽底层控制)[cite: 9] */
    // Track_Init();
    Balance_Init();
    Vision_Init();

    uint32_t last_10ms = 0;
    uint32_t last_100ms = 0;
    

    while (1)
    {
        // UART 中断只负责收包，命令解析放在主循环避免中断内阻塞。
        VOFA_CommandTask();
        Balance_AngleEstimateTask();

        /* 10ms 控制节拍[cite: 9] */
        if ((uint32_t)(g_sys_tick - last_10ms) >= 10)
        {
            last_10ms = g_sys_tick;
            Balance_Task();

        }

        /* 100ms 节拍[cite: 9] */
        if ((uint32_t)(g_sys_tick - last_100ms) >= 100)
        {
        
            
        }
    }
}

///////////////////* 需要中断运行的任务函数统一写在后面 *///////////////////

// IMU数据解析任务
void UART_IMU_INST_IRQHandler(void)
{
    uint32_t pending_irq = DL_UART_Main_getPendingInterrupt(UART_IMU_INST);
    if (pending_irq == DL_UART_IIDX_RX)
    {
        while (DL_UART_Main_isRXFIFOEmpty(UART_IMU_INST) == false)
        {
            // uint8_t rx_data = DL_UART_Main_receiveData(UART_IMU_INST);
            // IMU_UART_ParseByte(rx_data); // 暂时屏蔽
        }
    }
    else if ((pending_irq == DL_UART_IIDX_OVERRUN_ERROR) ||
             (pending_irq == DL_UART_IIDX_BREAK_ERROR) ||
             (pending_irq == DL_UART_IIDX_FRAMING_ERROR) ||
             (pending_irq == DL_UART_IIDX_PARITY_ERROR))
    {
        DL_UART_Main_clearInterruptStatus(UART_IMU_INST,
                                          (DL_UART_INTERRUPT_OVERRUN_ERROR |
                                           DL_UART_INTERRUPT_BREAK_ERROR |
                                           DL_UART_INTERRUPT_FRAMING_ERROR |
                                           DL_UART_INTERRUPT_PARITY_ERROR));
    }
}

// 视觉数据解析任务
void UART_K230_INST_IRQHandler(void)
{
    uint32_t pending_irq = DL_UART_Main_getPendingInterrupt(UART_K230_INST);
    if (pending_irq == DL_UART_IIDX_RX)
    {
        while (DL_UART_Main_isRXFIFOEmpty(UART_K230_INST) == false)
        {
            uint8_t rx_data = DL_UART_Main_receiveData(UART_K230_INST);
            Vision_ParseByte(rx_data); 
        }
    }
    else if ((pending_irq == DL_UART_IIDX_OVERRUN_ERROR) ||
             (pending_irq == DL_UART_IIDX_BREAK_ERROR) ||
             (pending_irq == DL_UART_IIDX_FRAMING_ERROR) ||
             (pending_irq == DL_UART_IIDX_PARITY_ERROR))
    {
        DL_UART_Main_clearInterruptStatus(UART_IMU_INST,
                                          (DL_UART_INTERRUPT_OVERRUN_ERROR |
                                           DL_UART_INTERRUPT_BREAK_ERROR |
                                           DL_UART_INTERRUPT_FRAMING_ERROR |
                                           DL_UART_INTERRUPT_PARITY_ERROR));
    }
}

void UART_VOFA_INST_IRQHandler(void)
{
    uint32_t pending_irq = DL_UART_Main_getPendingInterrupt(UART_VOFA_INST);
    if (pending_irq == DL_UART_IIDX_RX)
    {
        while (DL_UART_Main_isRXFIFOEmpty(UART_VOFA_INST) == false)
        {
            uint8_t rx_data = DL_UART_Main_receiveData(UART_VOFA_INST);
            VOFA_RX_Byte_Callback(rx_data);
        }
    }
    else if ((pending_irq == DL_UART_IIDX_OVERRUN_ERROR) ||
             (pending_irq == DL_UART_IIDX_BREAK_ERROR) ||
             (pending_irq == DL_UART_IIDX_FRAMING_ERROR) ||
             (pending_irq == DL_UART_IIDX_PARITY_ERROR))
    {
        DL_UART_Main_clearInterruptStatus(UART_VOFA_INST,
                                          (DL_UART_INTERRUPT_OVERRUN_ERROR |
                                           DL_UART_INTERRUPT_BREAK_ERROR |
                                           DL_UART_INTERRUPT_FRAMING_ERROR |
                                           DL_UART_INTERRUPT_PARITY_ERROR));
    }
}

#define DEBOUNCE_TIME_MS 200
// 按键中断处理函数[cite: 9]
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
                Balance_StartReturnToZero(BALANCE_RETURN_SPEED_DEFAULT);
                last_time_end = g_sys_tick;
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

// DMA中断函数
void DMA_IRQHandler(void)
{
    switch (DL_DMA_getPendingInterrupt(DMA)) {
        case DL_DMA_EVENT_IIDX_DMACH3:
            
            /* 清除中断标志 */
            DL_DMA_clearInterruptStatus(DMA, DL_DMA_INTERRUPT_CHANNEL3);
            
            /* 解除忙碌状态，允许下一次 Balance_Task 下发指令 */
            g_vofa_tx_busy = false; 
            
            break;
        case DL_DMA_EVENT_IIDX_DMACH4:
            
            /* 清除中断标志 */
            DL_DMA_clearInterruptStatus(DMA, DL_DMA_INTERRUPT_CHANNEL4);
            
            /* 解除忙碌状态，允许下一次 Balance_Task 下发指令 */
            g_motor_tx_busy = false; 
            
            break;

    
            
        // 如果有 K230 或陀螺仪的 DMA 接收通道，在此处继续添加 case
        // case DL_DMA_EVENT_IIDX_DMACH1: ...
        
        default:
            break;
    }
}
