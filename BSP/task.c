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

#define AB_DISTANCE_MM 1000   /* A→B 距离 (mm) */
#define AB_TIME_LIMIT_MS 8000 /* A→B 超时限制 (ms) */

// 在按钮中断中修改标志位
volatile ChassisTask_e g_chassis_task = Chassis_stop; // 小车任务标志位
volatile BalanceTask_e g_balance_task = Balance_stop; // 平衡任务标志位
volatile bool g_running = false;                      // 任务运行标志位

// 用于统计任务运行时间
static uint32_t task_start_tick = 0; // 任务开始时间戳
extern uint32_t g_sys_tick;          // 全局系统时基 (ms)
bool first_time = true;

void Chassis_Task()
{
    if (g_running == true)
    {
        if (first_time)
        {
            // 任务启动更新当前任务时间
            task_start_tick = g_sys_tick;
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

    // 第四行：预留显示一些动态数据，比如速度或者陀螺仪角度
    OLED_PrintfAt(3, 0, "Yaw: %.1f", g_imu_data.Yaw);
}
