#ifndef __BSP_MOTOR_IIC_H_
#define __BSP_MOTOR_IIC_H_

#include "ti_msp_dl_config.h"
#include "motor_iic.h"   /* 驱动板专用 I2C (PB11/PB12), 与 OLED/IMU 物理隔离 */
#include "string.h"
#include "stdlib.h"
#include "stdio.h"

#define Motor_model_ADDR    0x26   /* 驱动板 I2C 地址 */

typedef enum __Motor_IIC_ADDR_
{
    /* 写寄存器 */
    MOTOR_TYPE_REG      = 0x01,   /* 电机类型 */
    MOTOR_DeadZONE_REG  = 0x02,   /* 死区配置 */
    MOTOR_PluseLine_REG = 0x03,   /* 磁环线数 */
    MOTOR_PlusePhase_REG= 0x04,   /* 减速比 */
    WHEEL_DIA_REG       = 0x05,   /* 轮径 */
    SPEED_Control_REG   = 0x06,   /* 速度控制 */
    PWM_Control_REG     = 0x07,   /* PWM 控制 */

    /* 读寄存器 */
    READ_TEN_M1Enconer_REG = 0x10, /* M1 编码器 10ms 实时脉冲 */
    READ_TEN_M2Enconer_REG = 0x11, /* M2 编码器 10ms 实时脉冲 */
    READ_TEN_M3Enconer_REG = 0x12, /* M3 编码器 10ms 实时脉冲 */
    READ_TEN_M4Enconer_REG = 0x13, /* M4 编码器 10ms 实时脉冲 */

    READ_ALLHigh_M1_REG = 0x20,    /* M1 总脉冲 高位 */
    READ_ALLLOW_M1_REG  = 0x21,    /* M1 总脉冲 低位 */
    READ_ALLHigh_M2_REG = 0x22,    /* M2 总脉冲 高位 */
    READ_ALLLOW_M2_REG  = 0x23,    /* M2 总脉冲 低位 */
    READ_ALLHigh_M3_REG = 0x24,    /* M3 总脉冲 高位 */
    READ_ALLLOW_M3_REG  = 0x25,    /* M3 总脉冲 低位 */
    READ_ALLHigh_M4_REG = 0x26,    /* M4 总脉冲 高位 */
    READ_ALLLOW_M4_REG  = 0x27,    /* M4 总脉冲 低位 */

    IIC_REG_MAX                   /* 寄存器数量上限 */

} Motor_IIC_ADDR_t;

/* 编码器变量，供外部使用 */
extern int Encoder_Offset[4];   /* 10ms 实时脉冲差值 */
extern int Encoder_Now[4];      /* 累计脉冲总数 */

void control_speed(int16_t m1, int16_t m2, int16_t m3, int16_t m4);
void control_pwm(int16_t m1, int16_t m2, int16_t m3, int16_t m4);
void Set_motor_type(uint8_t data);
void Read_10_Enconder(void);
void Read_ALL_Enconder(void);
void Set_motor_deadzone(uint16_t data);
void Set_Pluse_line(uint16_t data);
void Set_Pluse_Phase(uint16_t data);
void Set_Wheel_dis(float data);

#endif
