/**
 * ZDT_X42S 闭环步进电机驱动 (适配 TI MSPM0G3507 - 纯DMA发送优化版)
 * 通信方式: 串口TTL/RS232/RS485 (Modbus RTU)
 */

#ifndef __ZDT_X42S_DRIVER_H
#define __ZDT_X42S_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

/* 引入 TI MSPM0 配置头文件，替代原有的 main.h */
#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>

/* 电机数量定义 */
#define MAX_MOTOR_COUNT   4
/* 电机ID默认 */
#define MOTOR_ID_DEFAULT  1

/* 编码器分辨率 */
#define ENCODER_RESOLUTION  6400 
#define ANGLE_TO_PULSE(angle)  ((int32_t)((angle) * ENCODER_RESOLUTION / 360.0))
#define PULSE_TO_ANGLE(pulse)  ((float)((pulse) * 360.0 / ENCODER_RESOLUTION))

extern volatile bool g_motor_tx_busy;



/* 电机控制模式 */
typedef enum {
    ZDT_MODE_SPEED = 0,    
    ZDT_MODE_POSITION = 1  
} ZDT_ControlMode_e;

/* 回零方向 */
typedef enum {
    ZDT_HOMING_DIR_DEFAULT = 0,  
    ZDT_HOMING_DIR_CW = 1,       
    ZDT_HOMING_DIR_CCW = 2,      
    ZDT_HOMING_DIR_LIMIT = 3     
} ZDT_HomingDir_e;

/* 当前驱动命令采用 Emm 固件格式，速度字段单位为 RPM。 */
#define RPM_TO_INTERNAL(rpm)  ((int16_t)(rpm))

/* 电机状态结构体 (精简版) */
typedef struct {
    uint8_t motor_id;           
    bool enabled;               
} Motor_StatusTypeDef;

/* 驱动参数结构体 (替换为 TI 的底层配置) */
typedef struct {
    UART_Regs *uart_inst;       /* UART寄存器实例，例如 UART0 */
    uint8_t dma_tx_ch;          /* DMA发送通道，例如 DMA_CH0 */
    uint8_t motor_id;           /* 电机ID */
    
    Motor_StatusTypeDef status; /* 电机状态 */

    /* 控制模式 */
    ZDT_ControlMode_e control_mode;     
    float target_position;              
    uint16_t position_speed;
} ZDT_MotorTypeDef;


int ZDT_Motor_Init(ZDT_MotorTypeDef *motor, UART_Regs *uart_inst, uint8_t dma_tx_ch, uint8_t motor_id);
int ZDT_Motor_Enable(ZDT_MotorTypeDef *motor, bool enable);
int ZDT_Motor_SetPosition(ZDT_MotorTypeDef *motor, float angle, uint16_t speed);
int ZDT_Motor_SetPosition_Pulse(ZDT_MotorTypeDef *motor, int32_t pulse, uint16_t speed);
int ZDT_Motor_SetSpeed(ZDT_MotorTypeDef *motor, int16_t speed);
int ZDT_Motor_Stop(ZDT_MotorTypeDef *motor);
int ZDT_Motor_ReturnZero(ZDT_MotorTypeDef *motor, uint16_t speed);
int ZDT_Motor_ReturnZeroEx(ZDT_MotorTypeDef *motor, uint16_t speed, ZDT_HomingDir_e direction);
int ZDT_Motor_ZeroPosition(ZDT_MotorTypeDef *motor);
int ZDT_Motor_ClearStall(ZDT_MotorTypeDef *motor);
int ZDT_Motor_SetControlMode(ZDT_MotorTypeDef *motor, ZDT_ControlMode_e mode);
void ZDT_Motor_SetTargetPosition(ZDT_MotorTypeDef *motor, float angle, uint16_t speed);
int ZDT_Motor_ChangeID(ZDT_MotorTypeDef *motor, uint8_t new_id);
int ZDT_Motor_SetMode(ZDT_MotorTypeDef *motor, bool closed_loop);
int ZDT_Motor_ReadPosition(ZDT_MotorTypeDef *motor);

#ifdef __cplusplus
}
#endif

#endif /* __ZDT_X42S_DRIVER_H */
