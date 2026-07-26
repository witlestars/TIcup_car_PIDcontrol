#include "IOI2C.h"

/**
 * 模拟 IIC 起始信号
 * 返回: 1
 */
int IIC_Start(void)
{
    SDA_OUT();
    SDA(1);
    SCL(1);
    delay_us(1);
    SDA(0);
    delay_us(1);
    SCL(0);
    return 1;
}

/**
 * 模拟 IIC 停止信号
 */
void IIC_Stop(void)
{
    SDA_OUT();
    SCL(0);
    SDA(0);
    SCL(1);
    delay_us(1);
    SDA(1);
    delay_us(1);
}

/**
 * IIC 等待应答信号
 * 返回: 0=收到应答, 1=未收到应答
 */
int IIC_Wait_Ack(void)
{
    char ack = 0;
    unsigned char ack_flag = 10;
    SCL(0);
    SDA(1);
    SDA_IN();

    SCL(1);
    while ((SDA_GET() == 1) && (ack_flag))
    {
        ack_flag--;
        delay_us(1);
    }

    if (ack_flag <= 0)
    {
        IIC_Stop();
        return 1;
    }
    else
    {
        SCL(0);
        SDA_OUT();
    }
    return ack;
}

/**
 * 主机发送应答/非应答信号
 * 参数: ack=0 发送应答, ack=1 发送非应答
 */
void IIC_Send_Ack(unsigned char ack)
{
    SDA_OUT();
    SCL(0);
    SDA(0);
    delay_us(5);
    if (!ack) SDA(0);
    else      SDA(1);
    SCL(1);
    delay_us(5);
    SCL(0);
    SDA(1);
}

/**
 * IIC 发送一个字节
 * 参数: txd=要发送的字节数据
 */
void IIC_Send_Byte(u8 txd)
{
    int i = 0;
    SDA_OUT();
    SCL(0);  /* 拉低时钟开始数据传输 */
    for (i = 0; i < 8; i++)
    {
        SDA((txd & 0x80) >> 7);
        delay_us(1);
        SCL(1);
        delay_us(5);
        SCL(0);
        delay_us(5);
        txd <<= 1;
    }
}

/**
 * IIC 写数据到寄存器
 * 参数: addr=设备地址, reg=寄存器地址, len=字节数, data=数据
 * 返回: 0=写入成功, 非0=失败
 */
int i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
    uint16_t i = 0;
    IIC_Start();
    IIC_Send_Byte((addr << 1) | 0);
    if (IIC_Wait_Ack() == 1) { IIC_Stop(); return 1; }
    IIC_Send_Byte(reg);
    if (IIC_Wait_Ack() == 1) { IIC_Stop(); return 2; }

    for (i = 0; i < len; i++)
    {
        IIC_Send_Byte(data[i]);
        if (IIC_Wait_Ack() == 1) { IIC_Stop(); return (3 + i); }
    }
    IIC_Stop();
    return 0;
}

/**
 * IIC 读寄存器数据
 * 参数: addr=设备地址, reg=寄存器地址, len=字节数, buf=接收缓冲区
 * 返回: 0=读取成功, 非0=失败
 */
int i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    uint8_t i;
    IIC_Start();
    IIC_Send_Byte((addr << 1) | 0);
    if (IIC_Wait_Ack() == 1) { IIC_Stop(); return 1; }
    IIC_Send_Byte(reg);
    if (IIC_Wait_Ack() == 1) { IIC_Stop(); return 2; }

    IIC_Start();
    IIC_Send_Byte((addr << 1) | 1);
    if (IIC_Wait_Ack() == 1) { IIC_Stop(); return 3; }

    for (i = 0; i < (len - 1); i++)
    {
        buf[i] = IIC_Read_Byte();
        IIC_Send_Ack(0);   /* 发送应答 */
    }
    buf[i] = IIC_Read_Byte();
    IIC_Send_Ack(1);       /* 发送非应答 */
    IIC_Stop();
    return 0;
}

/**
 * IIC 读取一个字节
 * 返回: 读取到的数据
 */
u8 IIC_Read_Byte(void)
{
    unsigned char i, receive = 0;
    SDA_IN();  /* SDA 设为输入 */
    for (i = 0; i < 8; i++)
    {
        SCL(0);
        delay_us(5);
        SCL(1);
        delay_us(5);
        receive <<= 1;
        if (SDA_GET())
        {
            receive |= 1;
        }
        delay_us(5);
    }
    SCL(0);
    return receive;
}
