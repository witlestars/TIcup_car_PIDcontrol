#ifndef __IOI2C_H
#define __IOI2C_H

#include "ti_msp_dl_config.h"
#include "stdint.h"
#include "delay.h"

#define u8 uint8_t

/* IO 方向设置
 * SDA_IN(): 先 initDigitalInputFeatures 带内部上拉 (代替无上拉的 initDigitalInput),
 *           再 disableOutput (释放总线).
 *           顺序不能反: initDigitalInput 可能重新使能输出,
 *           必须之后 disableOutput, 否则从机 ACK 读不到.
 *           内部上拉约30kΩ, 偏弱但应急可用 (理想是外部4.7kΩ, 已焊板子没法加).
 * SDA_OUT(): 切回输出, 先设高再 enable, 避免瞬间低电平产生误起始信号 */
#define SDA_IN()  { DL_GPIO_initDigitalInputFeatures(I2C_SDA_IOMUX, \
                          DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP, \
                          DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE); \
                    DL_GPIO_disableOutput(I2C_PORT, I2C_SDA_PIN); }
#define SDA_OUT() { DL_GPIO_initDigitalOutput(I2C_SDA_IOMUX);         \
                    DL_GPIO_setPins(I2C_PORT, I2C_SDA_PIN);           \
                    DL_GPIO_enableOutput(I2C_PORT, I2C_SDA_PIN); }

/* IO 操作函数 */
#define SCL(x)    ( (x) ? DL_GPIO_setPins(I2C_PORT,I2C_SCL_PIN) : DL_GPIO_clearPins(I2C_PORT,I2C_SCL_PIN) )   /* SCL */
#define SDA(x)    ( (x) ? DL_GPIO_setPins(I2C_PORT,I2C_SDA_PIN) : DL_GPIO_clearPins(I2C_PORT,I2C_SDA_PIN) )   /* SDA */
#define SDA_GET() ( ( ( DL_GPIO_readPins(I2C_PORT,I2C_SDA_PIN) & I2C_SDA_PIN ) > 0 ) ? 1 : 0 )                /* 读取 SDA */

/* IIC 全部操作函数 */
int  IIC_Start(void);           /* 发送 IIC 起始信号 */
void IIC_Stop(void);            /* 发送 IIC 停止信号 */
void IIC_Send_Byte(u8 txd);    /* IIC 发送一个字节 */
u8   IIC_Read_Byte(void);       /* IIC 读取一个字节 */
int  IIC_Wait_Ack(void);        /* IIC 等待应答信号 */
void IIC_Ack(void);             /* IIC 发送应答信号 */
void IIC_NAck(void);            /* IIC 发送非应答信号 */

int i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
int i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);

#endif
