/*
    用于任务切换
*/

#include <stdbool.h>
#include "task.h"
#include "balance.h"
#include "track.h"
#include "odometry.h"
#include "motor.h"
#include "bsp_motor_iic.h"  /* Encoder_Offset, g_i2c_err_m2/m4 调试 */
#include "oled.h"
#include "imu.h"

#define AB_DISTANCE_MM   1500   /* CAR3: A→B 距离 (mm) */
#define AB_TIME_LIMIT_MS 8000   /* CAR3: A→B 超时 (ms) */

#define LAP_TIME_LIMIT_CAR1   20000  /* CAR1 一圈超时 20s */
#define LAP_TIME_LIMIT_CAR45  30000  /* CAR4/5 一圈超时 30s */
#define LAP_MIN_CORNER     2      /* 至少过2个弯才算一圈 */
#define FINISH_LINE_DIST_MM  6400.0f  /* 终止线总里程阈值(同track.c LAP_PERIMETER_MM) */

// 在按钮中断中修改标志位
volatile ChassisTask_e g_chassis_task = Chassis_stop; // 小车任务标志位
volatile BalanceTask_e g_balance_task = Balance_stop; // 平衡任务标志位
volatile bool g_running = false;                      // 任务运行标志位

// 用于统计任务运行时间
static uint32_t task_start_tick = 0; // 任务开始时间戳
static uint32_t task_finish_time_ms[ChassisTaskNum] = {0}; // 各题最近一次用时
static ChassisTask_e running_task = Chassis_stop; // 本次启动时选择的题目
static ChassisTask_e last_finished_task = Chassis_stop; // 最近结束的题目
extern uint32_t g_sys_tick;          // 全局系统时基 (ms)
static bool first_time = true;

/* 纯灰度检测停车: 起步在终止线上全亮 → 走出终止线(不全亮) → 回到终止线全亮才停 */
static uint8_t left_start_line = 0;  /* 0=还没走出终止线, 1=已走出(可检测回终止线) */
static uint8_t finish_cnt = 0;       /* 终止线全亮持续帧计数 (防偶发误触发) */
#define FINISH_HOLD_TICKS  3          /* 连续3帧(30ms)全亮才确认停车 */

/* ─── 各CAR模式参数配置 ───
 * CAR1: 分段PD (直道460/P8/D28, 弯道250/P19/D20)
 * CAR3: A_to_B 单PD (B200/P12/D30, 跑1500mm停)
 * CAR4/5: 单PD一圈 (B230/P17/D27, 当前调好) */
static void Task_ApplyConfig(ChassisTask_e task)
{
    switch (task) {
    case CAR1:
        /* 分段PD: 恢复之前注释的那一套 */
        g_track_mode = 1;
        g_track_cfg.base_speed = 460;
        g_track_cfg.turn_p     = 8.0f;
        g_track_cfg.turn_d     = 28.0f;
        g_track_cfg.curve_p    = 19.0f;
        g_track_cfg.curve_d    = 20.0f;
        break;
    case CAR2:
        /* 占位, 不运动 */
        g_track_mode = 0;
        break;
    case CAR3:
        /* A_to_B: B200调好的单PD */
        g_track_mode = 0;
        g_track_cfg.base_speed = 200;
        g_track_cfg.turn_p     = 12.0f;
        g_track_cfg.turn_d     = 30.0f;
        g_track_cfg.curve_p    = 0.0f;
        g_track_cfg.curve_d    = 0.0f;
        break;
    case CAR4:
    case CAR5:
        /* 单PD一圈: 当前调好的B230/P17/D27, P+2 */
        g_track_mode = 0;
        g_track_cfg.base_speed = 230;
        g_track_cfg.turn_p     = 19.0f;
        g_track_cfg.turn_d     = 27.0f;
        g_track_cfg.curve_p    = 0.0f;
        g_track_cfg.curve_d    = 0.0f;
        break;
    default:
        break;
    }
}

void Chassis_Task()
{
    if (g_running == true)
    {
        if (first_time)
        {
            // 任务启动更新当前任务时间
            task_start_tick = g_sys_tick;
            running_task = g_chassis_task;
            Odom_Init();  /* 清零里程计, 防止上次里程累加 */
            Track_Reset();  /* 复位巡线状态(curve_in_curve等) */
            Task_ApplyConfig(g_chassis_task);  /* 根据CAR模式设置PD参数 */
            left_start_line = 0;  /* 复位: 起步在终止线上, 还没走出 */
            finish_cnt = 0;      /* 复位终止线持续计数 */
            first_time = false;
        }

        switch (g_chassis_task)
        {
        case Chassis_stop:
            Motor_Stop();
            break;

        case CAR1:
            /* 分段PD跑一圈: 弯道降速250, 过弯计数+4亮1帧停车 */
            Odom_Update();
            Track_Loop();

            /* 超时保护 (CAR1: 20s) */
            if ((uint32_t)(g_sys_tick - task_start_tick) >= LAP_TIME_LIMIT_CAR1)
            {
                Motor_Stop();
                g_chassis_task = Chassis_stop;
                g_running = false;
                break;
            }

            /* 停车判断: 过弯≥2 + 灰度4亮1帧立即停 */
            if (g_corner_count >= LAP_MIN_CORNER &&
                g_dbg.active_cnt >= 4)
            {
                Motor_Stop();
                g_chassis_task = Chassis_stop;
                g_running = false;
            }
            break;

        case CAR4:
        case CAR5:
            /* 单PD跑一圈: 过弯≥2 + 灰度4亮1帧停车 */
            Odom_Update();
            Track_Loop();

            /* 超时保护 (CAR4/5: 30s) */
            if ((uint32_t)(g_sys_tick - task_start_tick) >= LAP_TIME_LIMIT_CAR45)
            {
                Motor_Stop();
                g_chassis_task = Chassis_stop;
                g_running = false;
                break;
            }

            /* 停车判断: 过弯≥2 + 里程到位 + 灰度3亮1帧立即停 */
            if (g_corner_count >= LAP_MIN_CORNER &&
                Odom_Get_Total_Dist() >= FINISH_LINE_DIST_MM &&
                g_dbg.active_cnt >= 3)
            {
                Motor_Stop();
                g_chassis_task = Chassis_stop;
                g_running = false;
            }
            break;

        case CAR2:
            /* 占位: 等数据, 不运动 */
            Motor_Stop();
            break;

        case CAR3:
            /* A_to_B: 单PD跑1500mm停车
             * 超时保护: 8s */
            Odom_Update();
            Track_Loop();

            /* 超时保护 (8s) */
            if ((uint32_t)(g_sys_tick - task_start_tick) >= AB_TIME_LIMIT_MS)
            {
                Motor_Stop();
                g_chassis_task = Chassis_stop;
                g_running = false;
                break;
            }

            /* 停车判断: 总里程≥1500mm */
            if (Odom_Get_Total_Dist() >= AB_DISTANCE_MM)
            {
                Motor_Stop();
                g_chassis_task = Chassis_stop;
                g_running = false;
            }
            break;

        default:
            Motor_Stop();
            break;
        }

        if (g_running == false && running_task > Chassis_stop &&
            running_task < ChassisTaskNum)
        {
            task_finish_time_ms[running_task] =
                (uint32_t)(g_sys_tick - task_start_tick);
            last_finished_task = running_task;
            first_time = true;
        }
    }
    else {
        Motor_Stop();
        /* 按键手动停止时，在下一个控制周期锁存本题用时。 */
        if (first_time == false && running_task > Chassis_stop &&
            running_task < ChassisTaskNum)
        {
            task_finish_time_ms[running_task] =
                (uint32_t)(g_sys_tick - task_start_tick);
            last_finished_task = running_task;
        }
        first_time = true;
    }
}

// 平衡任务占位 (板球系统 PID + 陀螺仪前馈, 待实现)
void Balance_Task(void)
{
    /* TODO: 按 g_balance_task 分支实现 Balance_0/1/2 */
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

    // 第二行：显示底盘 (Chassis) 任务 + 过弯数
    OLED_PrintfAt(1, 0, "Car:%d C:%d", g_chassis_task, g_corner_count);
    // 第三行：显示边距离(edge_dist) + 总里程, 调试弯道检测触发
    OLED_PrintfAt(2, 0, "E:%4d D:%4d",
                  (int)Odom_Get_Edge_Dist(),
                  (int)Odom_Get_Total_Dist());

    OLED_ClearArea(3, 0, 16);
    if (g_running == true)
    {
        uint32_t elapsed_ms = (uint32_t)(g_sys_tick - task_start_tick);
        OLED_PrintfAt(3, 0, "C%d T:%lu.%02lu",
                      (int)running_task,
                      (unsigned long)(elapsed_ms / 1000U),
                      (unsigned long)((elapsed_ms % 1000U) / 10U));
    }
    else
    {
        ChassisTask_e display_task =
            (g_chassis_task != Chassis_stop) ?
            g_chassis_task : last_finished_task;
        uint32_t finish_ms = task_finish_time_ms[display_task];
        OLED_PrintfAt(3, 0, "C%d T:%lu.%02lu",
                      (int)display_task,
                      (unsigned long)(finish_ms / 1000U),
                      (unsigned long)((finish_ms % 1000U) / 10U));
    }
}
