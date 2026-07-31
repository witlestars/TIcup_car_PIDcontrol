/*
    用于任务切换
*/

#include <stdbool.h>
#include "task.h"
#include "balance.h"
#include "track.h"
#include "odometry.h"
#include "motor.h"
#include "oled.h"
#include "imu.h"
#include "vision_protocol.h"
#include "vofa.h"
#include "ZDT_X42S_Driver.h"

#define AB_DISTANCE_MM 1500   /* A→B 距离 (mm), 实测 1.5m */
#define AB_TIME_LIMIT_MS 8000 /* A→B 超时限制 (ms) */

// 在按钮中断中修改标志位
volatile ChassisTask_e g_chassis_task = Chassis_stop; // 小车任务标志位
volatile BalanceTask_e g_balance_task = Balance_stop; // 平衡任务标志位
volatile bool g_running = false;                      // 任务运行标志位

// 用于统计任务运行时间
static uint32_t task_start_tick = 0; // 任务开始时间戳
extern uint32_t volatile g_sys_tick;          // 全局系统时基 (ms)
static bool first_time = true;

void Chassis_Task()
{
    if (g_running == true)
    {
        if (first_time)
        {
            // 任务启动更新当前任务时间
            task_start_tick = g_sys_tick;
            Odom_Init();  /* 清零里程计, 防止上次里程累加 */
            first_time = false;
        }

        switch (g_chassis_task)
        {
        case Chassis_stop:
            Motor_Stop();
            break;
        case A_to_B:
            /* 第4题: A→B = 槽口型一段直道, 纯直线行驶, 无转弯
             * 判断到B: 里程计累计距离 ≥ AB_DISTANCE_MM
             * 兜底: 时间 ≥ 8s 强制停车 (防超时)
             * 巡线: 仍用 Track_Loop 做灰度PD保持直行 (直道上就是直行) */
            Odom_Update(); /* 更新里程计 */
            Track_Loop();  /* 灰度PD巡线 (直道上保持直行) */
            /* 主判断: 走够 AB 距离 */
            if (Odom_Get_Total_Dist() >= AB_DISTANCE_MM)
            {
                Motor_Stop();
                g_chassis_task = Chassis_stop;
                g_running = false;
            }
            /* 超时强制停车 (8s) */
            else if ((uint32_t)(g_sys_tick - task_start_tick) >= AB_TIME_LIMIT_MS)
            {
                Motor_Stop();
                g_chassis_task = Chassis_stop;
                g_running = false;
            }
            break;
        case One_Lap:
            /* 第2/5题: 一圈 = 槽口型完整一圈 (两直道+两半圆弯)
             * 用过弯计数: 槽口型一圈有2个半圆弯, 过2弯=1圈?
             * TODO: 实测槽口型一圈的弯道数, 调整阈值 */
            Odom_Update();
            Track_Loop();
            if (g_corner_count >= 2)
            { /* 槽口型2个弯=1圈 (待实测确认) */
                Motor_Stop();
                g_chassis_task = Chassis_stop;
                g_running = false;
            }
            break;
        }
    }
    else {
        Motor_Stop();
        first_time = true;
    }
}

// 平衡任务
void Balance_Task(void)
{
    // 只有在全局运行标志为 true 时才执行控制
    if (g_running == true) 
    {
            /* --- 1. 目标设定 --- */
            float target_pos = 0.0f; // 调参时，目标位置固定在物理中心 (0 mm)

            /* --- 2. 传感器数据预处理 --- */
            // 将视觉回传的 0.1mm 单位转换为 mm
            float vision_pos = g_vision_data.position_01mm / 10.0f; 
            // 直接读取 K230 传回的真实速度 (mm/s)
            float vision_vel = g_vision_data.velocity_mm_s;
            // 读取 K230 数据包中的延时标签 (ms)
            uint16_t age_ms  = g_vision_data.age_ms;
            
            // 读取陀螺仪角速度 (请根据你实际安装的方向选择 Gyro_X 或 Gyro_Y)
            float gyro_rate  = g_imu_data.GyroY; 

            /* --- 3. 调用核心控制算法 --- */
            // 该函数内部已包含 "延时推算补偿" 和 "视觉真实速度替换微分" 逻辑
            Balance_PID(target_pos, vision_pos, vision_vel, age_ms, gyro_rate);

            /* --- 4. VOFA+ 实时波形反馈 (DMA 触发) --- */
            // 计算用于可视化的预测位置
            float predicted_pos = vision_pos + (vision_vel * (age_ms / 1000.0f));
            
            // 根据 vofa.c 中定义的接口，发送 3 个关键浮点数据[cite: 6, 7]
            // CH0(target):  目标位置
            // CH1(current): 补偿后的预测位置 (观察回中平滑度)
            // CH2(output):  真实视觉速度 (观察阻尼效果与抖动)
            VOFA_SendWaveData(target_pos, predicted_pos, vision_vel); 
    }
    else 
    {
        // 如果 g_running 为 false，作为安全兜底，强制停机[cite: 8]
        Balance_MotorSetSpeed(0); 
    }
}

// 在主循环中以100ms为周期调度
void OLED_Task()
{
    if (g_oled_present == 0)
        return; // 屏幕没插则跳过

    // 第一行：标题栏，显示系统状态
    if (g_running == true)
    {
        OLED_PrintfAt(0, 0, "=== RUNNING ===");
    }
    else
    {
        OLED_PrintfAt(0, 0, "=== WAITING ===");
    }

    // 第二行：显示底盘 (Chassis) 任务
    OLED_PrintfAt(1, 0, "Car : %-2d", g_chassis_task);
    // 第三行：显示平衡 (Balance) 任务
    OLED_PrintfAt(2, 0, "Bal : %-2d", g_balance_task);

    // 第四行：显示里程计总里程 (mm) + Yaw, 方便调试编码器方向
    OLED_PrintfAt(3, 0, "D:%5.0f Y:%.0f", Odom_Get_Total_Dist(), g_imu_data.Yaw);
}
