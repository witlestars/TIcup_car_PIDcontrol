/**
 * ZDT_X42S 闭环步进电机驱动实现 (适配 TI MSPM0G3507 - 纯DMA发送优化版)
 */

#include "ZDT_X42S_Driver.h"

#define SYNC_FLAG  0

volatile bool g_motor_tx_busy = false;
/**
 * @brief  发送命令 (使用 MSPM0 的 DMA)
 */
static int ZDT_Motor_SendCommand(ZDT_MotorTypeDef *motor, uint8_t *tx_data, uint16_t tx_len)
{
    if (motor == NULL || tx_data == NULL || tx_len == 0 || g_motor_tx_busy) {
        return -1;
    }

    DL_DMA_disableChannel(DMA, motor->dma_tx_ch);

    /* 1. 设置源地址 (你的指令数组 tx_data) */
    DL_DMA_setSrcAddr(DMA, motor->dma_tx_ch, (uint32_t)tx_data);
    /* 2. 设置目的地址 (刚刚修改的 TXDATA 寄存器) */
    DL_DMA_setDestAddr(DMA, motor->dma_tx_ch, (uint32_t)(&motor->uart_inst->TXDATA));
    /* 3. 设置传输长度 (本次指令的字节数) */
    DL_DMA_setTransferSize(DMA, motor->dma_tx_ch, tx_len);
    /* 4. 使能 DMA 通道 (非常重要，否则不会发送) */
    g_motor_tx_busy = true;
    DL_DMA_enableChannel(DMA, motor->dma_tx_ch);
    
    return 0;
}


/**
 * @brief  初始化ZDT闭环步进电机
 */
int ZDT_Motor_Init(ZDT_MotorTypeDef *motor, UART_Regs *uart_inst, uint8_t dma_tx_ch, uint8_t motor_id)
{
    if (motor == NULL || uart_inst == NULL || motor_id == 0) {
        return -1;
    }

    motor->uart_inst = uart_inst;
    motor->dma_tx_ch = dma_tx_ch;
    motor->motor_id = motor_id;
    motor->status.motor_id = motor_id;
    motor->status.enabled = false;

    motor->control_mode = ZDT_MODE_POSITION;
    motor->target_position = 0;
    motor->position_speed = 150;

    return 0;
}

int ZDT_Motor_Enable(ZDT_MotorTypeDef *motor, bool enable)
{
    static uint8_t cmd[8];
    cmd[0] = motor->motor_id;      
    cmd[1] = 0xF3;                 
    cmd[2] = 0xAB;                 
    cmd[3] = enable ? 0x01 : 0x00; 
    cmd[4] = SYNC_FLAG;            
    cmd[5] = 0x6B;                 

    int result = ZDT_Motor_SendCommand(motor, cmd, 6);
    if (result == 0) {
        motor->status.enabled = enable;
    }
    return result;
}

int ZDT_Motor_SetPosition(ZDT_MotorTypeDef *motor, float angle, uint16_t speed)
{
    int32_t pulse = ANGLE_TO_PULSE(angle);
    return ZDT_Motor_SetPosition_Pulse(motor, pulse, speed);
}

int ZDT_Motor_SetPosition_Pulse(ZDT_MotorTypeDef *motor, int32_t pulse, uint16_t speed)
{
    static uint8_t cmd[16];
    uint8_t dir = 0;
    int32_t pulse_abs = pulse;

    if (pulse < 0) {
        dir = 1;  
        pulse_abs = -pulse;
    }

    cmd[0] = motor->motor_id;      
    cmd[1] = 0xFD;                 
    cmd[2] = dir;                  
    cmd[3] = (uint8_t)(speed >> 8); 
    cmd[4] = (uint8_t)(speed & 0xFF); 
    cmd[5] = 0;                    
    cmd[6] = (uint8_t)(pulse_abs >> 24); 
    cmd[7] = (uint8_t)(pulse_abs >> 16); 
    cmd[8] = (uint8_t)(pulse_abs >> 8);  
    cmd[9] = (uint8_t)(pulse_abs & 0xFF); 
    cmd[10] = 1;                   
    cmd[11] = SYNC_FLAG;           
    cmd[12] = 0x6B;                

    return ZDT_Motor_SendCommand(motor, cmd, 13);
}

int ZDT_Motor_SetSpeed(ZDT_MotorTypeDef *motor, int16_t speed)
{
    /* 1. 检查 DMA 是否忙碌 */
    if (g_motor_tx_busy == true) {
        // DMA 还在发送上一帧，直接返回，保护内存不被覆盖
        return -1; 
    }

    static uint8_t cmd[8];
    uint8_t dir = 0;
    int16_t speed_abs = speed;

    if (speed < 0) {
        dir = 1;  
        speed_abs = -speed;
    }

    cmd[0] = motor->motor_id;      
    cmd[1] = 0xF6;                 
    cmd[2] = dir;                  
    cmd[3] = (uint8_t)(speed_abs >> 8); 
    cmd[4] = (uint8_t)(speed_abs & 0xFF); 
    cmd[5] = 0;                    
    cmd[6] = SYNC_FLAG;            
    cmd[7] = 0x6B;                 

    return ZDT_Motor_SendCommand(motor, cmd, 8);
}

int ZDT_Motor_Stop(ZDT_MotorTypeDef *motor)
{
    static uint8_t cmd[6];
    cmd[0] = motor->motor_id;      
    cmd[1] = 0xFE;                 
    cmd[2] = 0x98;                 
    cmd[3] = SYNC_FLAG;            
    cmd[4] = 0x6B;                 

    return ZDT_Motor_SendCommand(motor, cmd, 5);
}

int ZDT_Motor_ReturnZero(ZDT_MotorTypeDef *motor, uint16_t speed)
{
    return ZDT_Motor_ReturnZeroEx(motor, speed, ZDT_HOMING_DIR_DEFAULT);
}

int ZDT_Motor_ReturnZeroEx(ZDT_MotorTypeDef *motor, uint16_t speed, ZDT_HomingDir_e direction)
{
    static uint8_t cmd[6];
    uint8_t homing_mode = 0;

    switch (direction)
    {
        case ZDT_HOMING_DIR_DEFAULT:
        case ZDT_HOMING_DIR_CW: homing_mode = 0; break;
        case ZDT_HOMING_DIR_CCW: homing_mode = 0; break;
        case ZDT_HOMING_DIR_LIMIT: homing_mode = 2; break;
        default: homing_mode = 0; break;
    }

    cmd[0] = motor->motor_id;      
    cmd[1] = 0x9A;                 
    cmd[2] = homing_mode;          
    cmd[3] = SYNC_FLAG;            
    cmd[4] = 0x6B;                 

    return ZDT_Motor_SendCommand(motor, cmd, 5);
}

int ZDT_Motor_ZeroPosition(ZDT_MotorTypeDef *motor)
{
    static uint8_t cmd[5];
    cmd[0] = motor->motor_id;      
    cmd[1] = 0x0A;                 
    cmd[2] = 0x6D;                 
    cmd[3] = 0x6B;                 

    return ZDT_Motor_SendCommand(motor, cmd, 4);
}

int ZDT_Motor_ClearStall(ZDT_MotorTypeDef *motor)
{
    static uint8_t cmd[5];
    cmd[0] = motor->motor_id;
    cmd[1] = 0x0E;
    cmd[2] = 0x52;
    cmd[3] = 0x6B;
    return ZDT_Motor_SendCommand(motor, cmd, 4);
}

int ZDT_Motor_ChangeID(ZDT_MotorTypeDef *motor, uint8_t new_id)
{
    static uint8_t cmd[5];
    cmd[0] = motor->motor_id;
    cmd[1] = 0xAE;
    cmd[2] = new_id;
    cmd[3] = 0x6B;
    int result = ZDT_Motor_SendCommand(motor, cmd, 4);
    if (result == 0) {
        motor->motor_id = new_id;
        motor->status.motor_id = new_id;
    }
    return result;
}

int ZDT_Motor_SetMode(ZDT_MotorTypeDef *motor, bool closed_loop)
{
    static uint8_t cmd[7];
    cmd[0] = motor->motor_id;
    cmd[1] = 0x46;
    cmd[2] = 0x69;
    cmd[3] = 0;                    
    cmd[4] = closed_loop ? 0x02 : 0x01;  
    cmd[5] = 0x6B;
    return ZDT_Motor_SendCommand(motor, cmd, 6);
}

int ZDT_Motor_SetControlMode(ZDT_MotorTypeDef *motor, ZDT_ControlMode_e mode)
{
    if (motor == NULL) return -1;
    motor->control_mode = mode;
    return 0;
}

void ZDT_Motor_SetTargetPosition(ZDT_MotorTypeDef *motor, float angle, uint16_t speed)
{
    if (motor == NULL) return;
    motor->target_position = angle;
    motor->position_speed = speed;
}
