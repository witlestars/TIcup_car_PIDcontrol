/**
 * @file    button.c
 * @brief   4按钮 GPIO 中断驱动 + 事件队列
 *
 * 架构 (中断驱动, 替代原轮询方案):
 *   SysConfig 配置 4 个按钮 GPIO 下降沿中断 + 8 周期硬件消抖
 *   GROUP0_IRQHandler (MSPM0 默认 GPIO 中断组) 处理 GPIOA + GPIOB
 *   ISR 内做 20ms 软件消抖 (防硬件消抖残留抖动) + push 事件到 FIFO
 *   主循环 Button_Get_Event 取事件执行业务逻辑 (ISR 不做业务)
 *
 * 硬件接线 (按钮一端接 pin, 另一端接 GND, 内部上拉):
 *   BTN_START  = PA7  (GPIOA, GROUP0)
 *   BTN_LAP_UP = PA18 (GPIOA, GROUP0)
 *   BTN_MODE   = PB1  (GPIOB, GROUP0)
 *   BTN_RESET  = PB14 (GPIOB, GROUP0)
 *
 * 中断分组说明:
 *   MSPM0G3507 的 GPIO 中断默认走 GROUP0 (syscfg 未设 interruptGroup 时)
 *   GROUP0_IRQHandler 里同时检查 GPIOA 和 GPIOB 的 pending
 *   (队友 develop-CIJUN 分支用 GROUP1 且只查一次, PA7/PA18 进不来, 已修)
 */

#include "button.h"
#include "ti_msp_dl_config.h"

/* SysTick 1ms 时基 (定义在 main.c, ISR 内用来做软件消抖时间戳) */
extern volatile uint32_t g_sys_tick;

/* 按钮索引 */
#define N_BTN  4
enum { IDX_START = 0, IDX_LAP_UP, IDX_MODE, IDX_RESET };

/* 软件消抖时间戳 (ms), 每个按钮独立 */
static uint32_t s_last_tick[N_BTN];
#define DEBOUNCE_MS  20   /* 软件消抖间隔 (硬件已有 8 cycles 消抖, 这层是兜底) */

/* 事件队列 (ISR 生产, 主循环消费, 单生产单消费环形缓冲无需关中断) */
#define EVT_QUEUE_SIZE 8
static btn_event_t evt_queue[EVT_QUEUE_SIZE];
static volatile uint8_t evt_head = 0, evt_tail = 0;

void Button_Init(void)
{
    for (uint8_t i = 0; i < N_BTN; i++) {
        s_last_tick[i] = 0;
    }
    evt_head = evt_tail = 0;
    /* GPIO 中断已由 SYSCFG_DL_init() 启用 (syscfg interruptEn=true),
     * NVIC 也由 SysConfig 自动启用, 这里不需要手动 NVIC_EnableIRQ */
}

/* ISR 内调用: 带消抖的事件 push */
static void btn_isr_handle(uint8_t idx, btn_event_t evt, uint32_t now)
{
    if ((now - s_last_tick[idx]) < DEBOUNCE_MS) {
        return;   /* 消抖窗口内, 忽略 */
    }
    s_last_tick[idx] = now;

    /* push 事件到 FIFO (队列满则丢弃, 不阻塞 ISR) */
    uint8_t next = (uint8_t)((evt_tail + 1) % EVT_QUEUE_SIZE);
    if (next != evt_head) {
        evt_queue[evt_tail] = evt;
        evt_tail = next;
    }
}

/**
 * @brief GPIO GROUP0 中断服务程序
 *        MSPM0G3507 默认所有 GPIO 中断走 GROUP0 (syscfg 未改 interruptGroup)
 *        同时检查 GPIOA (PA7/PA18) 和 GPIOB (PB1/PB14) 的 pending
 * @note  DL_GPIO_getPendingInterrupt 会自动清除 pending 标志
 */
void GROUP0_IRQHandler(void)
{
    uint32_t now = g_sys_tick;   /* 主循环的 SysTick 1ms 时基 */

    /* 检查 GPIOA: BTN_START(PA7) + BTN_LAP_UP(PA18) */
    uint32_t irqA = DL_GPIO_getPendingInterrupt(GPIOA);
    switch (irqA) {
    case GPIO_BUTTON_BTN_START_IIDX:
        btn_isr_handle(IDX_START, BTN_EVENT_START, now);
        break;
    case GPIO_BUTTON_BTN_LAP_UP_IIDX:
        btn_isr_handle(IDX_LAP_UP, BTN_EVENT_LAP_UP, now);
        break;
    default:
        break;
    }

    /* 检查 GPIOB: BTN_MODE(PB1) + BTN_RESET(PB14) */
    uint32_t irqB = DL_GPIO_getPendingInterrupt(GPIOB);
    switch (irqB) {
    case GPIO_BUTTON_BTN_LAP_DN_IIDX:   /* syscfg 里还叫 LAP_DN, 实际是 MODE */
        btn_isr_handle(IDX_MODE, BTN_EVENT_MODE, now);
        break;
    case GPIO_BUTTON_BTN_RESET_IIDX:
        btn_isr_handle(IDX_RESET, BTN_EVENT_RESET, now);
        break;
    default:
        break;
    }
}

btn_event_t Button_Get_Event(void)
{
    if (evt_head == evt_tail) return BTN_EVENT_NONE;
    btn_event_t e = evt_queue[evt_head];
    evt_head = (uint8_t)((evt_head + 1) % EVT_QUEUE_SIZE);
    return e;
}

/* 读取4个按钮当前电平 (调试用, 给OLED显示)
 * 返回 bit0~bit3: 1=按下(低电平), 0=松开(高电平)
 * bit0=START(PA7), bit1=LAP_UP(PA18), bit2=MODE(PB1), bit3=RESET(PB14) */
uint8_t Button_Get_Raw_Level(void)
{
    uint8_t r = 0;
    if ((DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_7)  == 0)) r |= 0x01;
    if ((DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_18) == 0)) r |= 0x02;
    if ((DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_1)  == 0)) r |= 0x04;
    if ((DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_14) == 0)) r |= 0x08;
    return r;
}
