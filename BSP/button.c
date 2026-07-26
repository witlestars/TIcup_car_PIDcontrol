/**
 * @file    button.c
 * @brief   4按钮消抖 + 事件队列实现
 */

#include "button.h"
#include "ti_msp_dl_config.h"

/* 按钮 GPIO 宏 (SysConfig 里命名 GPIO_BUTTON)
 * 4个pin (按用户实际焊接位置):
 *   BTN_START  = PA7  (A07)
 *   BTN_LAP_UP = PA18 (A18)
 *   BTN_MODE   = PB1  (B01) ← 原 AIN_2 已移到 PA13, 原 LAP_DN 改为 MODE
 *   BTN_RESET  = PB14 (B14)
 */
#define BTN_START_PIN     DL_GPIO_PIN_7
#define BTN_LAP_UP_PIN    DL_GPIO_PIN_18
#define BTN_MODE_PIN      DL_GPIO_PIN_1
#define BTN_RESET_PIN     DL_GPIO_PIN_14
/* PA7/PA18 在 GPIOA, PB1/PB14 在 GPIOB, 分开读 */

/* 按钮索引 */
#define N_BTN  4
enum { IDX_START = 0, IDX_LAP_UP, IDX_MODE, IDX_RESET };

/* 消抖状态 */
static struct {
    uint8_t debounce;     /* 消抖计数器 0~3 */
    uint8_t stable;       /* 当前稳定电平 0=按下 1=松开 */
    uint8_t prev_stable;  /* 上一次稳定电平 */
} btn[N_BTN];

/* 事件队列 */
#define EVT_QUEUE_SIZE 4
static btn_event_t evt_queue[EVT_QUEUE_SIZE];
static uint8_t evt_head = 0, evt_tail = 0;

/* 读取4个按钮的原始电平 (0=按下, 1=松开) */
static uint8_t read_raw(uint8_t idx)
{
    /* PA7/PA18 在 GPIOA, PB1/PB14 在 GPIOB */
    switch (idx) {
        case IDX_START:   return (DL_GPIO_readPins(GPIOA, BTN_START_PIN)  > 0) ? 1 : 0;
        case IDX_LAP_UP:  return (DL_GPIO_readPins(GPIOA, BTN_LAP_UP_PIN) > 0) ? 1 : 0;
        case IDX_MODE:    return (DL_GPIO_readPins(GPIOB, BTN_MODE_PIN)   > 0) ? 1 : 0;
        case IDX_RESET:   return (DL_GPIO_readPins(GPIOB, BTN_RESET_PIN)  > 0) ? 1 : 0;
        default: return 1;
    }
}

void Button_Init(void)
{
    for (uint8_t i = 0; i < N_BTN; i++) {
        btn[i].debounce    = 0;
        btn[i].stable      = 1;   /* 默认松开 */
        btn[i].prev_stable = 1;
    }
    evt_head = evt_tail = 0;
}

/* 压入事件 */
static void push_event(btn_event_t e)
{
    uint8_t next = (evt_tail + 1) % EVT_QUEUE_SIZE;
    if (next != evt_head) {   /* 队列没满 */
        evt_queue[evt_tail] = e;
        evt_tail = next;
    }
}

void Button_Poll(void)
{
    for (uint8_t i = 0; i < N_BTN; i++) {
        uint8_t raw = read_raw(i);

        /* 消抖: 与当前稳定值不同则计数, 连续3次(30ms)确认 */
        if (raw != btn[i].stable) {
            btn[i].debounce++;
            if (btn[i].debounce >= 3) {
                btn[i].prev_stable = btn[i].stable;
                btn[i].stable = raw;
                btn[i].debounce = 0;

                /* 检测下降沿 (1→0 = 按下) */
                if (btn[i].prev_stable == 1 && btn[i].stable == 0) {
                    btn_event_t e = BTN_EVENT_NONE;
                    switch (i) {
                        case IDX_START:   e = BTN_EVENT_START;     break;
                        case IDX_LAP_UP:  e = BTN_EVENT_LAP_UP;    break;
                        case IDX_MODE:    e = BTN_EVENT_MODE;      break;
                        case IDX_RESET:   e = BTN_EVENT_RESET;     break;
                    }
                    if (e != BTN_EVENT_NONE) push_event(e);
                }
            }
        } else {
            btn[i].debounce = 0;
        }
    }
}

btn_event_t Button_Get_Event(void)
{
    if (evt_head == evt_tail) return BTN_EVENT_NONE;
    btn_event_t e = evt_queue[evt_head];
    evt_head = (evt_head + 1) % EVT_QUEUE_SIZE;
    return e;
}

/* 读取4个按钮当前电平 (调试用)
 * 返回 bit0~bit3: 1=按下(低电平), 0=松开(高电平)
 * bit0=START(PA7), bit1=LAP_UP(PA18), bit2=MODE(PB1), bit3=RESET(PB14)
 */
uint8_t Button_Get_Raw_Level(void)
{
    uint8_t r = 0;
    if (read_raw(IDX_START)   == 0) r |= 0x01;
    if (read_raw(IDX_LAP_UP)  == 0) r |= 0x02;
    if (read_raw(IDX_MODE)    == 0) r |= 0x04;
    if (read_raw(IDX_RESET)   == 0) r |= 0x08;
    return r;
}
