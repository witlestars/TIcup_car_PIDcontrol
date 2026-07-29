/**
 * @file    main.c
 * @brief   板球系统主程序 (H题)
 *
 * 架构 (分层):
 *   main.c      : 初始化 + 主循环调度 + ISR 收口
 *   task.c      : 任务切换 (g_task_id → ChassisTask_e / BalanceTask_e)
 *   balance.c   : 板球平衡 PID + ZDT 步进电机
 *   track.c     : 灰度巡线 PID
 *   vision_protocol.c : K230 视觉坐标接收
 *   k230.c      : K230 题号发送 (TX)
 *   imu.c       : JY61P 姿态解析 (ISR 内直解析)
 *   soft_uart.c : 软件串口 (PB8/PB10, 9600, TIMG0采样, 供cmd调参)
 *   cmd.c       : 命令解析 (基于软件串口)
 *
 * 按键 (GROUP1 中断, 20ms 防抖):
 *   BTN_START  (PA7)  : 启停切换
 *   BTN_LAP_UP (PA18) : 切题 1→2→3→4→5→1 + 发K230 + OLED更新
 *   BTN_LAP_DN (PB1)  : 空置 (预留)
 */

#include "ti_msp_dl_config.h"
#include "task.h"
#include "balance.h"
#include "track.h"
#include "motor.h"
#include "imu.h"
#include "oled.h"
#include "odometry.h"
#include "vision_protocol.h"
#include "k230.h"
#include "soft_uart.h"
#include "cmd.h"
#include <stdio.h>

/* SysTick 1ms 时基 */
volatile uint32_t g_sys_tick = 0;
void SysTick_Handler(void) { g_sys_tick++; }

/* 视觉协议实例 (UART_K230 ISR 写入) */
Vision_ProtocolTypeDef g_vision;

/* 按钮防抖 */
#define DEBOUNCE_TIME_MS 20
static uint32_t last_time = 0;

/* 题目切换: 1→2→3→4→5→1, 同步 OLED + K230 + 任务映射 */
static void Task_Next(void)
{
    g_task_id = (g_task_id >= 5) ? 1 : (g_task_id + 1);
    Task_ApplyTaskId();
    K230_SendTask(g_task_id);
    if (g_oled_present) {
        OLED_ClearArea(0, 0, 16);
        OLED_PrintfAt(0, 0, "Task: %d", g_task_id);
        OLED_PrintfAt(1, 0, "Task: %d %s", g_task_id, g_running ? "RUN " : "STOP");
    }
}

/* 启停切换 */
static void Run_Toggle(void)
{
    if (g_running) {
        g_running = 0;
        Motor_Stop();
        if (g_oled_present) OLED_PrintfAt(1, 0, "Task: %d STOP", g_task_id);
    } else {
        g_running = 1;
        Task_OnStart();   /* 复位里程计/巡线/时间戳 */
        if (g_oled_present) OLED_PrintfAt(1, 0, "Task: %d RUN ", g_task_id);
    }
}

int main(void)
{
    /* 1. 初始化系统外设 (SysConfig 自动生成) */
    SYSCFG_DL_init();

    /* 2. 使能全局中断 */
    __enable_irq();

    /* 3. 模块初始化 */
    OLED_Init();                  /* OLED 显示 (I2C, 在线检测) */
    Motor_Init();                 /* 双轮驱动板 (I2C) */
    Track_Init();                 /* 巡线状态机复位 */
    Odom_Init();                  /* 里程计清零 */
    IMU_Init();                   /* IMU 使能 RX 中断 */
    Vision_Init(&g_vision);       /* 视觉协议状态机复位 */
    Balance_Init();               /* 平衡 PID + ZDT 步进电机 */
    SoftUART_Init();              /* 软件串口 (PB8/PB10, 9600, TIMG0采样) */
    CMD_Init();                   /* 命令解析 (基于软件串口) */
    /* 4. 上电默认 task_id=1, 应用任务映射 + 发题号给 K230 */
    Task_ApplyTaskId();
    K230_SendTask(g_task_id);

    /* 5. OLED 开屏显示 */
    if (g_oled_present) {
        OLED_Clear();
        OLED_PrintfAt(0, 0, "==TI CUP H==");
        OLED_PrintfAt(1, 0, "Task: %d STOP", g_task_id);
    }

    uint32_t last_10ms = 0;

    /* 主循环: 10ms 节拍, 只做调度 */
    while (1) {
        /* 命令解析: 每轮都轮询 (非阻塞, 内部限 16 字节) */
        CMD_Poll();

        if ((uint32_t)(g_sys_tick - last_10ms) >= 10) {
            last_10ms = g_sys_tick;

            if (g_running) {
                /* 运行中: 执行底盘 + 平衡任务 */
                Chassis_Task(g_chassis_task);
                Balance_Task(g_balance_task);
            } else {
                /* 停止: 确保电机停 */
                Motor_Stop();
            }
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
            uint8_t rx_data = DL_UART_Main_receiveData(UART_K230_INST);
            Vision_ParseByte(&g_vision, rx_data);
        }
    }
    // 溢出、帧错误、校验错误
    else if ((pending_irq == DL_UART_IIDX_OVERRUN_ERROR) ||
             (pending_irq == DL_UART_IIDX_BREAK_ERROR) ||
             (pending_irq == DL_UART_IIDX_FRAMING_ERROR) ||
             (pending_irq == DL_UART_IIDX_PARITY_ERROR))
    {
        // 发生错误时，必须清除标志位防止死锁！
        DL_UART_Main_clearInterruptStatus(UART_K230_INST,
            (DL_UART_INTERRUPT_OVERRUN_ERROR |
             DL_UART_INTERRUPT_BREAK_ERROR |
             DL_UART_INTERRUPT_FRAMING_ERROR |
             DL_UART_INTERRUPT_PARITY_ERROR));
    }
}


// 按键中断处理函数 (统一入口, GROUP1 = GPIOA + GPIOB)
/*
    通过中断改变标志位，在主循环中传参来切换模式
    ISR 内只做轻量业务 (切题/启停), 不做长耗时操作
*/
void GROUP1_IRQHandler(void)
{
    // ====== 检查并处理 GPIOA 端口的按键 ======
    uint32_t gpioA_status = DL_GPIO_getPendingInterrupt(GPIOA);

    if (gpioA_status == GPIO_BUTTON_BTN_START_IIDX) {
        // ====== 防抖处理 ======
        if ((g_sys_tick - last_time) > DEBOUNCE_TIME_MS) {
            Run_Toggle();
            last_time = g_sys_tick;
        }
    }
    else if (gpioA_status == GPIO_BUTTON_BTN_LAP_UP_IIDX) {
        if ((g_sys_tick - last_time) > DEBOUNCE_TIME_MS) {
            Task_Next();
            last_time = g_sys_tick;
        }
    }

    // ====== 检查并处理 GPIOB 端口的按键 ======
    uint32_t gpioB_status = DL_GPIO_getPendingInterrupt(GPIOB);

    if (gpioB_status == GPIO_BUTTON_BTN_LAP_DN_IIDX) {
        if ((g_sys_tick - last_time) > DEBOUNCE_TIME_MS) {
            // TODO: BTN_LAP_DN (PB1, 预留)
            last_time = g_sys_tick;
        }
    }
    /* BTN_RESET (PB14) 不存在, syscfg 只有3个按钮 */
}

// 软件串口 RX 采样定时器 (TIMG0, 9600×3=28800Hz, 每34.7μs中断)
// ISR 内只调用状态机, ~2μs, 不影响I2C时序
void SOFTUART_TIMER_INST_IRQHandler(void)
{
    switch (DL_Timer_getPendingInterrupt(SOFTUART_TIMER_INST)) {
    case DL_TIMER_IIDX_ZERO:
        SoftUART_TimerISR();   /* 3x过采样状态机 */
        break;
    default:
        break;
    }
}
