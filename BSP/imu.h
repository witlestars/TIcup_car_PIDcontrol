#ifndef __IMU_H__
#define __IMU_H__

#include <stdint.h>

/* IMU 数据结构体 */
typedef struct {
    // 角度数据 (单位: 度 °)
    float Roll;
    float Pitch;
    float Yaw;
    
    // 加速度数据 (单位: g)
    float AccX;
    float AccY;
    float AccZ;
    
    // 角速度数据 (单位: °/s)
    float GyroX;
    float GyroY;
    float GyroZ;
    
    // 状态标志位（可选：用于主循环判断是否收到了最新数据）
    uint8_t update_flag; 
} IMU_Data_t;

/* 
 * 声明全局 IMU 数据变量 
 * 使用 volatile 关键字防止编译器优化，确保主循环每次都从内存读取中断更新后的最新值
 */
extern volatile IMU_Data_t g_imu_data;

/* 函数声明 */
void IMU_Init(void);
void IMU_UART_ParseByte(uint8_t rx_byte);

#endif /* __IMU_H__ */
