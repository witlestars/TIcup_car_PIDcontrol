/**
 * @file    grey.h
 * @brief   8路数字灰度传感器模块驱动 — 头文件
 * 
 * 硬件:
 *   模块: 亚博YB-MVX05 八路灰度 (CD4051模拟开关 + LM393比较器 x8)
 *   接线: AD0=PB4, AD1=PB5, AD2=PB2 (通道选择)
 *         OUT=PA15 (数字输入, CD4051输出脚 → MCU GPIO)
 * 
 * 读值规则:
 *   OUT=0 → 白(红外光被反射, LM393拉低OUT)
 *   OUT=1 → 黑(红外光被吸收, LM393开漏悬空, 上拉至高)
 * 
 * 重心计算:
 *   CH0~CH7 映射到 -3.5~+3.5, 中心在通道3.5处
 *   centroid = Σ(value[i] * (i-3.5)) / active_count
 *   active_count > 6 → 判为全黑跑偏 → valid=0
 */

#ifndef __GREY_H
#define __GREY_H
#include "stdlib.h"
#include "ti_msp_dl_config.h"

#define GREY_CHANNEL_COUNT  8        /* 8路传感器 */

/** 灰度传感器完整状态 */
typedef struct {
    uint8_t  value[GREY_CHANNEL_COUNT];   /* 0=白(反光), 1=黑(吸红外) */
    uint8_t  active[GREY_CHANNEL_COUNT];  /* 1=本轮检测到黑线 */
    float    centroid;                     /* 重心 -3.5~+3.5, 0=正中间 */
    uint8_t  valid;                        /* 1=有效, 0=丢线/全黑跑偏 */
} Grey_State_t;

/** 遍历8路, 更新state->value/centroid/valid */
void Grey_Read(Grey_State_t *state);

/** VOFA+调试: 发送8路原始值 (JustFloat, UART_DEBUG) */
void Grey_Debug_Send(Grey_State_t *state);

#endif
