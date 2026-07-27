/**
 * @file    oled.h
 * @brief   0.96寸 OLED (SSD1306) I2C 驱动
 *
 * 硬件:
 *   SSD1306 I2C 地址 0x3C (与驱动板0x26, JY61P 0x50 共线)
 *   SCL → PA16, SDA → PA1 (共用软件I2C)
 *   VCC → 3.3V, GND → GND
 *
 * 分辨率: 128×64, 4行×16字符 (8×16字体)
 */

#ifndef __OLED_H
#define __OLED_H
#include <stdint.h>

#define OLED_WIDTH   128
#define OLED_HEIGHT  64
#define OLED_COLS    16    /* 每行字符数 (8像素宽) */
#define OLED_ROWS    4     /* 行数 (16像素高) */

/* OLED 在线标志 (1=检测到, 0=未接, 所有操作自动跳过) */
extern uint8_t g_oled_present;

/** 初始化 OLED (SSD1306 标准序列) */
void OLED_Init(void);

/** 关显示 (发 0xAE, 屏幕黑屏但 GDDRAM 保留) */
void OLED_PowerOff(void);

/** 清屏 */
void OLED_Clear(void);

/** 设置光标位置 (row: 0-3, col: 0-15) */
void OLED_SetCursor(uint8_t row, uint8_t col);

/** 在当前光标位置打印字符串 (自动换行) */
void OLED_Print(const char *str);

/** 格式化打印 (类似 printf) */
void OLED_Printf(const char *fmt, ...);

/** 在指定位置打印字符串 */
void OLED_PrintAt(uint8_t row, uint8_t col, const char *str);

/** 格式化打印到指定位置 */
void OLED_PrintfAt(uint8_t row, uint8_t col, const char *fmt, ...);

#endif
