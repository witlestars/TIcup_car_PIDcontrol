/**
 * @file    motor_iic.h
 * @brief   驱动板专用软件 I2C (PB11 SDA / PB12 SCL)
 *          物理隔离 OLED/IMU 用的 PA17/PA15, 避免互相干扰
 */
#ifndef __MOTOR_IIC_H
#define __MOTOR_IIC_H

#include "ti_msp_dl_config.h"
#include "stdint.h"
#include "delay.h"

#define u8 uint8_t

/* IO 方向设置 (用 MOTOR_I2C 组的 pin, 与 OLED/IMU 的 I2C 组物理隔离)
 * SDA_IN(): 先 initDigitalInput, 再 disableOutput (释放总线)
 * SDA_OUT(): 先设高再 enable, 避免瞬间低电平产生误起始信号 */
#define MOTOR_SDA_IN()  { DL_GPIO_initDigitalInput(MOTOR_I2C_MSDA_IOMUX);            \
                          DL_GPIO_disableOutput(MOTOR_I2C_PORT, MOTOR_I2C_MSDA_PIN); }
#define MOTOR_SDA_OUT() { DL_GPIO_initDigitalOutput(MOTOR_I2C_MSDA_IOMUX);           \
                          DL_GPIO_setPins(MOTOR_I2C_PORT, MOTOR_I2C_MSDA_PIN);       \
                          DL_GPIO_enableOutput(MOTOR_I2C_PORT, MOTOR_I2C_MSDA_PIN); }

/* IO 操作函数 (用 MOTOR_I2C 组) */
#define MOTOR_SCL(x)    ( (x) ? DL_GPIO_setPins(MOTOR_I2C_PORT,MOTOR_I2C_MSCL_PIN) : DL_GPIO_clearPins(MOTOR_I2C_PORT,MOTOR_I2C_MSCL_PIN) )
#define MOTOR_SDA(x)    ( (x) ? DL_GPIO_setPins(MOTOR_I2C_PORT,MOTOR_I2C_MSDA_PIN) : DL_GPIO_clearPins(MOTOR_I2C_PORT,MOTOR_I2C_MSDA_PIN) )
#define MOTOR_SDA_GET() ( ( ( DL_GPIO_readPins(MOTOR_I2C_PORT,MOTOR_I2C_MSDA_PIN) & MOTOR_I2C_MSDA_PIN ) > 0 ) ? 1 : 0 )

/* IIC 操作函数 (motor_ 前缀, 与 OLED/IMU 用的 IOI2C 区分) */
int  Motor_IIC_Start(void);
void Motor_IIC_Stop(void);
void Motor_IIC_Send_Byte(u8 txd);
u8    Motor_IIC_Read_Byte(void);
int   Motor_IIC_Wait_Ack(void);
void  Motor_IIC_Send_Ack(unsigned char ack);

int motor_i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
int motor_i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);

#endif
