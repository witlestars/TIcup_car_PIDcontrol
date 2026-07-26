/**
 * @file    grey.c
 * @brief   8路数字灰度传感器驱动 — 实现
 * 
 * CD4051 模拟开关选通时序:
 *   1. AD0/AD1/AD2 写通道号
 *   2. delay_us(50) 等待开关稳定 + LM393比较完成
 *   3. 读 OUT 引脚电平
 */

#include "grey.h"
#include "ti_msp_dl_config.h"
#include "uart_debug.h"
#include "delay.h"

/** AD0|AD1|AD2 三根引脚掩码, 切换通道前先全清 */
#define GREY_ADDR_MASK  (GPIO_GREY_ADDR_ADDR0_PIN | \
                         GPIO_GREY_ADDR_ADDR1_PIN | \
                         GPIO_GREY_ADDR_ADDR2_PIN)

/** 超过此数判为传感器全覆盖(跑偏到暗面) → 无效 */
#define GREY_ACTIVE_MAX  6

/* ======================== 底层引脚操作 ======================== */

/**
 * @brief 选通指定传感器通道 (0~7 → CD4051的X0~X7)
 */
static void Grey_Set_Channel(uint8_t ch)
{
    DL_GPIO_clearPins(GPIO_GREY_ADDR_PORT, GREY_ADDR_MASK);
    if (ch & 0x01) DL_GPIO_setPins(GPIO_GREY_ADDR_PORT,
                                    GPIO_GREY_ADDR_ADDR0_PIN);
    if (ch & 0x02) DL_GPIO_setPins(GPIO_GREY_ADDR_PORT,
                                    GPIO_GREY_ADDR_ADDR1_PIN);
    if (ch & 0x04) DL_GPIO_setPins(GPIO_GREY_ADDR_PORT,
                                    GPIO_GREY_ADDR_ADDR2_PIN);
}

/**
 * @brief 读当前选通通道的 OUT 脚电平
 * @return 1=黑线, 0=白
 */
static uint8_t Grey_Read_OUT(void)
{
    return (DL_GPIO_readPins(GPIO_GREY_OUT_PORT,
                             GPIO_GREY_OUT_OUT_PIN) != 0) ? 1 : 0;
}

/* ======================== 对外 API ======================== */

/**
 * @brief 扫描全部8路传感器, 计算重心
 * 
 * 遍历流程:
 *   选通通道 → 50us稳定 → 读OUT → 下一通道
 * 
 * 重心: weighted_sum / active_count, 范围 -3.5~+3.5
 * 
 * 丢线判定:
 *   active_count == 0        → 全白, 丢线
 *   active_count >  6        → 全黑(传感器全在暗面), 丢线
 *   1 ≤ active_count ≤ 6     → 正常
 */
void Grey_Read(Grey_State_t *state)
{
    if (state == NULL) return;

    uint8_t i, active_count = 0;
    float weighted_sum = 0.0f;

    for (i = 0; i < GREY_CHANNEL_COUNT; i++) {
        Grey_Set_Channel(i);
        delay_us(50);                   /* CD4051切换+LM393比较建立 */

        state->value[i] = Grey_Read_OUT();
        if (state->value[i] == 1) {     /* 1=黑线 */
            state->active[i] = 1;
            active_count++;
            weighted_sum += ((float)i - 3.5f);  /* CH_i → -3.5~+3.5 */
        } else {
            state->active[i] = 0;
        }
    }

    /* 丢线判定: 0路或超过6路 → 无效 */
    if (active_count == 0 || active_count > GREY_ACTIVE_MAX) {
        state->centroid = 0.0f;
        state->valid = 0;
    } else {
        state->centroid = weighted_sum / (float)active_count;
        state->valid = 1;
    }
}

/**
 * @brief VOFA+ 调试: 发送8路原始值 (JustFloat协议, UART_DEBUG, 有线USB)
 */
void Grey_Debug_Send(Grey_State_t *state)
{
    float buf[8];
    uint8_t i;
    for (i = 0; i < 8; i++) {
        buf[i] = (float)state->value[i];
    }
    UART_Debug_Send(buf, 8);
}
