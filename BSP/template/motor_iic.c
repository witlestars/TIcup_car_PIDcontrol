/**
 * @file    motor_iic.c
 * @brief   驱动板专用硬件 I2C 实现 (调用 TI DriverLib)
 */

#include "motor_iic.h"

// 超时防死区，保护 10ms 控制循环不会因为电机离线而卡死
#define I2C_TIMEOUT 100000 

/**
 * IIC 写数据到寄存器 (硬件 I2C 版)
 * 返回: 0=成功, 非0=失败(超时或NACK)
 */
int motor_i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
    uint32_t timeout = I2C_TIMEOUT;
    
    // 1. 等待总线空闲
    while (!(DL_I2C_getControllerStatus(I2C_Motor_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (--timeout == 0) return 1;
    }

    // 2. 发起主机 TX 传输：总长度 = 1字节(寄存器地址) + 数据长度(len)
    DL_I2C_startControllerTransfer(I2C_Motor_INST, addr, DL_I2C_CONTROLLER_DIRECTION_TX, len + 1);

    // 3. 将寄存器地址塞入 TX FIFO
    timeout = I2C_TIMEOUT;
    while (DL_I2C_isControllerTXFIFOFull(I2C_Motor_INST)) {
        if (--timeout == 0) return 2;
    }
    DL_I2C_transmitControllerData(I2C_Motor_INST, reg);

    // 4. 循环将待发送数据塞入 TX FIFO
    for (uint8_t i = 0; i < len; i++) {
        timeout = I2C_TIMEOUT;
        while (DL_I2C_isControllerTXFIFOFull(I2C_Motor_INST)) {
            if (--timeout == 0) return (3 + i);
        }
        DL_I2C_transmitControllerData(I2C_Motor_INST, data[i]);
    }

    // 5. 等待所有数据传输完成，总线恢复空闲
    timeout = I2C_TIMEOUT;
    while (!(DL_I2C_getControllerStatus(I2C_Motor_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (--timeout == 0) return 0xFF;
    }

    // 6. 检查是否存在 NACK 或其他硬件错误
    if (DL_I2C_getControllerStatus(I2C_Motor_INST) & DL_I2C_CONTROLLER_STATUS_ERROR) {
        DL_I2C_flushControllerTXFIFO(I2C_Motor_INST);  /* NACK 后清 FIFO 防锁死 */
        return 0xEE;
    }

    return 0;
}

/**
 * IIC 读寄存器数据 (硬件 I2C 版)
 * 返回: 0=成功, 非0=失败
 */
int motor_i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    uint32_t timeout = I2C_TIMEOUT;

    // 1. 等待总线空闲
    while (!(DL_I2C_getControllerStatus(I2C_Motor_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (--timeout == 0) return 1;
    }

    // ========== 阶段一：向从机写入“要读取的寄存器地址” ==========
    DL_I2C_startControllerTransfer(I2C_Motor_INST, addr, DL_I2C_CONTROLLER_DIRECTION_TX, 1);
    
    timeout = I2C_TIMEOUT;
    while (DL_I2C_isControllerTXFIFOFull(I2C_Motor_INST)) {
        if (--timeout == 0) return 2;
    }
    DL_I2C_transmitControllerData(I2C_Motor_INST, reg);

    timeout = I2C_TIMEOUT;
    while (!(DL_I2C_getControllerStatus(I2C_Motor_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (--timeout == 0) return 3;
    }

    if (DL_I2C_getControllerStatus(I2C_Motor_INST) & DL_I2C_CONTROLLER_STATUS_ERROR) {
        DL_I2C_flushControllerTXFIFO(I2C_Motor_INST);  /* NACK 后清 FIFO 防锁死 */
        return 0xEE;
    }

    // ========== 阶段二：转换方向，从从机读取数据 ==========
    DL_I2C_startControllerTransfer(I2C_Motor_INST, addr, DL_I2C_CONTROLLER_DIRECTION_RX, len);

    // 循环从 RX FIFO 读取数据
    for (uint8_t i = 0; i < len; i++) {
        timeout = I2C_TIMEOUT;
        // 阻塞等待硬件把数据接收完毕放到 FIFO 里
        while (DL_I2C_isControllerRXFIFOEmpty(I2C_Motor_INST)) {
            if (--timeout == 0) return (4 + i);
        }
        buf[i] = DL_I2C_receiveControllerData(I2C_Motor_INST);
    }

    // 等待传输彻底结束
    timeout = I2C_TIMEOUT;
    while (!(DL_I2C_getControllerStatus(I2C_Motor_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (--timeout == 0) return 0xFF;
    }

    return 0;
}
