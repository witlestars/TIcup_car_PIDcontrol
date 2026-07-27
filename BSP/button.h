/**
 * @file    button.h
 * @brief   4按钮 GPIO 中断驱动 (PA7/PA18/PB1/PB14, 下降沿+硬件消抖)
 *          MSPM0G3507 GPIO 中断走 GROUP1, ISR 设标志位, 主循环 Button_HandleEvents() 执行业务
 */

#ifndef __BUTTON_H
#define __BUTTON_H
#include <stdint.h>

void    Button_Init(void);           /* 清标志位 + 使能 GROUP1 NVIC (SysConfig 只开外设级) */
void    Button_HandleEvents(void);   /* 主循环10ms调: 检查标志位执行业务 */
uint8_t Button_Get_Raw_Level(void);  /* 调试用: 读4按钮电平 bit0~bit3 */

#endif
