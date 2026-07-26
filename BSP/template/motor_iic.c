/**
 * @file    motor_iic.c
 * @brief   驱动板专用软件 I2C 实现 (PB11 SDA / PB12 SCL)
 *          与 OLED/IMU 用的 IOI2C (PA17/PA15) 物理隔离
 */

#include "motor_iic.h"

/**
 * 模拟 IIC 起始信号
 */
int Motor_IIC_Start(void)
{
    MOTOR_SDA_OUT();
    MOTOR_SDA(1);
    MOTOR_SCL(1);
    delay_us(1);
    MOTOR_SDA(0);
    delay_us(1);
    MOTOR_SCL(0);
    return 1;
}

/**
 * 模拟 IIC 停止信号
 */
void Motor_IIC_Stop(void)
{
    MOTOR_SDA_OUT();
    MOTOR_SCL(0);
    MOTOR_SDA(0);
    MOTOR_SCL(1);
    delay_us(1);
    MOTOR_SDA(1);
    delay_us(1);
}

/**
 * IIC 等待应答信号
 * 返回: 0=收到应答, 1=未收到应答
 */
int Motor_IIC_Wait_Ack(void)
{
    char ack = 0;
    unsigned char ack_flag = 10;
    MOTOR_SCL(0);
    MOTOR_SDA(1);
    MOTOR_SDA_IN();

    MOTOR_SCL(1);
    while ((MOTOR_SDA_GET() == 1) && (ack_flag))
    {
        ack_flag--;
        delay_us(1);
    }

    if (ack_flag <= 0)
    {
        Motor_IIC_Stop();
        return 1;
    }
    else
    {
        MOTOR_SCL(0);
        MOTOR_SDA_OUT();
    }
    return ack;
}

/**
 * 主机发送应答/非应答信号
 */
void Motor_IIC_Send_Ack(unsigned char ack)
{
    MOTOR_SDA_OUT();
    MOTOR_SCL(0);
    MOTOR_SDA(0);
    delay_us(5);
    if (!ack) MOTOR_SDA(0);
    else      MOTOR_SDA(1);
    MOTOR_SCL(1);
    delay_us(5);
    MOTOR_SCL(0);
    MOTOR_SDA(1);
}

/**
 * IIC 发送一个字节
 */
void Motor_IIC_Send_Byte(u8 txd)
{
    int i = 0;
    MOTOR_SDA_OUT();
    MOTOR_SCL(0);
    for (i = 0; i < 8; i++)
    {
        MOTOR_SDA((txd & 0x80) >> 7);
        delay_us(1);
        MOTOR_SCL(1);
        delay_us(5);
        MOTOR_SCL(0);
        delay_us(5);
        txd <<= 1;
    }
}

/**
 * IIC 读取一个字节
 */
u8 Motor_IIC_Read_Byte(void)
{
    unsigned char i, receive = 0;
    MOTOR_SDA_IN();
    for (i = 0; i < 8; i++)
    {
        MOTOR_SCL(0);
        delay_us(5);
        MOTOR_SCL(1);
        delay_us(5);
        receive <<= 1;
        if (MOTOR_SDA_GET())
        {
            receive |= 1;
        }
        delay_us(5);
    }
    MOTOR_SCL(0);
    return receive;
}

/**
 * IIC 写数据到寄存器 (驱动板专用)
 * 返回: 0=成功, 非0=失败
 */
int motor_i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
    uint16_t i = 0;
    Motor_IIC_Start();
    Motor_IIC_Send_Byte((addr << 1) | 0);
    if (Motor_IIC_Wait_Ack() == 1) { Motor_IIC_Stop(); return 1; }
    Motor_IIC_Send_Byte(reg);
    if (Motor_IIC_Wait_Ack() == 1) { Motor_IIC_Stop(); return 2; }

    for (i = 0; i < len; i++)
    {
        Motor_IIC_Send_Byte(data[i]);
        if (Motor_IIC_Wait_Ack() == 1) { Motor_IIC_Stop(); return (3 + i); }
    }
    Motor_IIC_Stop();
    return 0;
}

/**
 * IIC 读寄存器数据 (驱动板专用)
 * 返回: 0=成功, 非0=失败
 */
int motor_i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    uint8_t i;
    Motor_IIC_Start();
    Motor_IIC_Send_Byte((addr << 1) | 0);
    if (Motor_IIC_Wait_Ack() == 1) { Motor_IIC_Stop(); return 1; }
    Motor_IIC_Send_Byte(reg);
    if (Motor_IIC_Wait_Ack() == 1) { Motor_IIC_Stop(); return 2; }

    Motor_IIC_Start();
    Motor_IIC_Send_Byte((addr << 1) | 1);
    if (Motor_IIC_Wait_Ack() == 1) { Motor_IIC_Stop(); return 3; }

    for (i = 0; i < (len - 1); i++)
    {
        buf[i] = Motor_IIC_Read_Byte();
        Motor_IIC_Send_Ack(0);
    }
    buf[i] = Motor_IIC_Read_Byte();
    Motor_IIC_Send_Ack(1);
    Motor_IIC_Stop();
    return 0;
}
