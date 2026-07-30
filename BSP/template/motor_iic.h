/**
 * @file    motor_iic.h
 * @brief   驱动板专用硬件 I2C 接口 (基于 SysConfig I2C_Motor)
 */
#ifndef __MOTOR_IIC_H
#define __MOTOR_IIC_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

/* IIC 读写函数 (硬件驱动版，保持原 API 格式兼容上层代码) */
int motor_i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
int motor_i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);

#endif
