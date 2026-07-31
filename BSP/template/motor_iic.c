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
 * IIC 读寄存器数据 (硬件 I2C 版, Repeated Start 时序)
 *
 * 关键: 驱动板要求 "写寄存器地址 → Repeated Start → 读数据",
 *       不能在中间发 STOP (否则驱动板内部寄存器指针复位, 读到 0x00)
 *       用 DL_I2C_startControllerTransferAdvanced 手动控制 START/STOP
 *
 * 参照 TI 官方例程 i2c_controller_rw_repeated_start:
 *   1. 先填 TX FIFO, 再启动传输 (顺序不能反, 否则控制器空转卡死)
 *   2. 阶段一 TX: START=1, STOP=0 (不发 STOP, 保持寄存器指针)
 *   3. 阶段二 RX: START=1, STOP=1 (Repeated Start + STOP)
 *
 * 返回: 0=成功, 非0=失败
 */
int motor_i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    uint32_t timeout;

    // 1. 等待总线空闲
    timeout = I2C_TIMEOUT;
    while (!(DL_I2C_getControllerStatus(I2C_Motor_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (--timeout == 0) return 1;
    }

    // 清除残留的中断标志
    DL_I2C_clearInterruptStatus(I2C_Motor_INST,
        (DL_I2C_INTERRUPT_CONTROLLER_NACK | DL_I2C_INTERRUPT_CONTROLLER_START |
         DL_I2C_INTERRUPT_CONTROLLER_STOP | DL_I2C_INTERRUPT_CONTROLLER_TX_DONE |
         DL_I2C_INTERRUPT_CONTROLLER_RX_DONE));

    // ========== 阶段一：写寄存器地址 (START=1, STOP=0) ==========
    // 先填 TX FIFO (官方顺序: 先填数据再启动传输)
    timeout = I2C_TIMEOUT;
    while (DL_I2C_isControllerTXFIFOFull(I2C_Motor_INST)) {
        if (--timeout == 0) return 2;
    }
    DL_I2C_transmitControllerData(I2C_Motor_INST, reg);

    // 再启动 TX 传输 (START=1, STOP=0, 不发 STOP!)
    DL_I2C_startControllerTransferAdvanced(I2C_Motor_INST, addr,
        DL_I2C_CONTROLLER_DIRECTION_TX, 1,
        DL_I2C_CONTROLLER_START_ENABLE,
        DL_I2C_CONTROLLER_STOP_DISABLE,
        DL_I2C_CONTROLLER_ACK_DISABLE);

    // 等待 1 字节 TX 完成 (用 TX_DONE 原始标志, 无需使能中断)
    // 不发 STOP 时总线不会 IDLE, 所以不能等 IDLE
    timeout = I2C_TIMEOUT;
    while (1) {
        uint32_t raw = DL_I2C_getRawInterruptStatus(I2C_Motor_INST,
            (DL_I2C_INTERRUPT_CONTROLLER_NACK | DL_I2C_INTERRUPT_CONTROLLER_TX_DONE));
        if (raw & DL_I2C_INTERRUPT_CONTROLLER_NACK) {
            DL_I2C_flushControllerTXFIFO(I2C_Motor_INST);
            // 发 STOP 释放总线
            DL_I2C_startControllerTransferAdvanced(I2C_Motor_INST, addr,
                DL_I2C_CONTROLLER_DIRECTION_TX, 0,
                DL_I2C_CONTROLLER_START_DISABLE,
                DL_I2C_CONTROLLER_STOP_ENABLE,
                DL_I2C_CONTROLLER_ACK_DISABLE);
            return 0xEE;
        }
        if (raw & DL_I2C_INTERRUPT_CONTROLLER_TX_DONE) {
            break;
        }
        if (--timeout == 0) return 3;
    }

    // ========== 阶段二：Repeated Start 读数据 (START=1, STOP=1) ==========
    DL_I2C_startControllerTransferAdvanced(I2C_Motor_INST, addr,
        DL_I2C_CONTROLLER_DIRECTION_RX, len,
        DL_I2C_CONTROLLER_START_ENABLE,
        DL_I2C_CONTROLLER_STOP_ENABLE,
        DL_I2C_CONTROLLER_ACK_DISABLE);

    // 循环从 RX FIFO 读取数据
    for (uint8_t i = 0; i < len; i++) {
        timeout = I2C_TIMEOUT;
        while (DL_I2C_isControllerRXFIFOEmpty(I2C_Motor_INST)) {
            if (--timeout == 0) return (4 + i);
        }
        buf[i] = DL_I2C_receiveControllerData(I2C_Motor_INST);
    }

    // 等待传输彻底结束 (发了 STOP, 总线会回到 IDLE)
    timeout = I2C_TIMEOUT;
    while (!(DL_I2C_getControllerStatus(I2C_Motor_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (--timeout == 0) return 0xFF;
    }

    return 0;
}
