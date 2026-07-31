#include "imu.h"
#include "ti_msp_dl_config.h" // 包含 MSPM0 库头文件

/* 定义全局 IMU 数据结构体 */
volatile IMU_Data_t g_imu_data = {0};

/**
 * @brief  IMU 模块初始化 (软件层面)
 * @note   硬件 UART 初始化已由 SysConfig 自动生成的 SYSCFG_DL_init() 完成
 */
void IMU_Init(void)
{
    // 初始化清零数据
    g_imu_data.Roll = 0.0f;
    g_imu_data.Pitch = 0.0f;
    g_imu_data.Yaw = 0.0f;
    g_imu_data.update_flag = 0;
    // 使能 UART 接收中断，确保接收到的字节能触发中断并调用解析函数
    NVIC_EnableIRQ(UART_IMU_INST_INT_IRQN);
}

/**
 * @brief  IMU 串口数据解析函数 (状态机)
 * @param  rx_byte: 串口单次接收到的 1 个字节数据
 * @note   !!! 此函数必须放置在 UART 接收中断服务函数 (RX ISR) 内部运行 !!!
 */
void IMU_UART_ParseByte(uint8_t rx_byte)
{
    static uint8_t rx_buffer[11]; // 静态数组，用于存放一帧 11 字节的数据
    static uint8_t rx_cnt = 0;    // 接收计数器（状态机状态）
    
    // 状态 0：寻找帧头 0x55
    if (rx_cnt == 0) {
        if (rx_byte == 0x55) {
            rx_buffer[0] = rx_byte;
            rx_cnt = 1; 
        }
    } 
    // 状态 1：接收剩余的 10 个字节
    else {
        rx_buffer[rx_cnt] = rx_byte;
        rx_cnt++;
        
        // 当收满 11 个字节时，进行校验和解析
        if (rx_cnt == 11) {
            uint8_t checksum = 0;
            
            // JY61P 校验和算法：前 10 个字节相加，保留低 8 位
            for (int i = 0; i < 10; i++) {
                checksum += rx_buffer[i];
            }
            
            // 校验和匹配，数据有效
            if (checksum == rx_buffer[10]) {
                
                // 1. 解析角度包 (0x53)
                if (rx_buffer[1] == 0x53) {
                    float roll  = ((int16_t)(rx_buffer[3] << 8 | rx_buffer[2])) / 32768.0f * 180.0f;
                    float pitch = ((int16_t)(rx_buffer[5] << 8 | rx_buffer[4])) / 32768.0f * 180.0f;
                    float yaw   = ((int16_t)(rx_buffer[7] << 8 | rx_buffer[6])) / 32768.0f * 180.0f;

                    /* 异常值过滤: JY61P 角度范围 ±180°, 超范围说明帧错位, 丢弃 */
                    if (yaw > 180.0f || yaw < -180.0f ||
                        roll > 180.0f || roll < -180.0f ||
                        pitch > 180.0f || pitch < -180.0f) {
                        rx_cnt = 0;
                        return;
                    }

                    g_imu_data.Roll  = roll;
                    g_imu_data.Pitch = pitch;
                    g_imu_data.Yaw   = yaw;

                    // 标记角度数据已更新，主循环 PID 可根据此标志位进行控制
                    g_imu_data.update_flag = 1;
                }
                // 2. 解析角速度包 (0x52) - JY61P 量程默认 2000 °/s
                else if (rx_buffer[1] == 0x52) {
                    g_imu_data.GyroX = ((int16_t)(rx_buffer[3] << 8 | rx_buffer[2])) / 32768.0f * 2000.0f;
                    g_imu_data.GyroY = ((int16_t)(rx_buffer[5] << 8 | rx_buffer[4])) / 32768.0f * 2000.0f;
                    g_imu_data.GyroZ = ((int16_t)(rx_buffer[7] << 8 | rx_buffer[6])) / 32768.0f * 2000.0f;
                }
                // 3. 解析加速度包 (0x51) - JY61P 量程默认 16g
                else if (rx_buffer[1] == 0x51) {
                    g_imu_data.AccX  = ((int16_t)(rx_buffer[3] << 8 | rx_buffer[2])) / 32768.0f * 16.0f;
                    g_imu_data.AccY  = ((int16_t)(rx_buffer[5] << 8 | rx_buffer[4])) / 32768.0f * 16.0f;
                    g_imu_data.AccZ  = ((int16_t)(rx_buffer[7] << 8 | rx_buffer[6])) / 32768.0f * 16.0f;
                }
            }
            
            // 一帧处理完毕（无论校验成功与否），计数器归零，准备接收下一帧的 0x55
            rx_cnt = 0; 
        }
    }
}

