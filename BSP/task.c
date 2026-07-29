/*
    用于任务切换
*/

#include "task.h"
#include "balance.h"
#include "track.h"
#include "motor.h"
#include "imu.h"
#include "k230.h"
#include "vision_protocol.h"
#include "odometry.h"

uint8_t g_task_id      = 1;            /* 当前题目号 1~5 */
uint8_t g_running      = 0;            /* 上电默认停止 */
ChassisTask_e g_chassis_task = Chassis_stop;  /* 小车任务标志位 */
BalanceTask_e g_balance_task = Balance_stop;  /* 平衡任务标志位 */

/* AB间距 = 槽口型直道长度 (mm, 实测标定, H题直道段长度)
 * 注意: 这是 A→B 直线段距离, 无转弯 */
#define AB_DISTANCE_MM      1500.0f
/* AB 限时 8s (H题第4题要求 ≤8s) */
#define AB_TIME_LIMIT_MS    8000u

/* 任务起始时间戳 (按 START 时清零) */
static uint32_t g_task_start_tick = 0;

/* Balance_0 序列状态机 (Task_OnStart 复位) */
static uint8_t  bal0_phase = 0;   /* 0=未启动 1=去+5 2=去-5 3=停-5 */
static uint32_t bal0_tick  = 0;   /* 阶段内计时 */

/* 根据 g_task_id 映射到底盘/平衡任务 */
void Task_ApplyTaskId(void)
{
    switch (g_task_id)
    {
    case 1:   /* 图传 (不控球, 不动车) */
    case 2:   /* 循线一圈 (不控球, 后续在 Chassis_Task 实现) */
        g_chassis_task = (g_task_id == 2) ? One_Lap : Chassis_stop;
        g_balance_task = Balance_stop;
        break;
    case 3:   /* 静止控球 (锁定中心) */
        g_chassis_task = Chassis_stop;
        g_balance_task = Balance_1;
        break;
    case 4:   /* A→B + 控球 (1/4圈, ≤8s, 球误差≤1cm) */
        g_chassis_task = A_to_B;
        g_balance_task = Balance_2;
        break;
    case 5:   /* 一圈 + 控球 */
        g_chassis_task = One_Lap;
        g_balance_task = Balance_2;
        break;
    default:
        g_chassis_task = Chassis_stop;
        g_balance_task = Balance_stop;
        break;
    }
}

/* 任务启动时调用 (main.c Run_Toggle 里 g_running 0→1 时调) */
void Task_OnStart(void)
{
    g_task_start_tick = g_sys_tick;
    Odom_Reset();
    Track_Reset();
    /* 复位 Balance_0 序列状态机 (下次进 Balance_0 从阶段0开始) */
    bal0_phase = 0;
    bal0_tick  = 0;
}

void Chassis_Task(ChassisTask_e task)
{
    switch (task)
    {
    case Chassis_stop:
        Motor_Stop();
        break;
    case A_to_B:
        /* 第4题: A→B = 槽口型一段直道, 纯直线行驶, 无转弯
         * 判断到B: 里程计累计距离 ≥ AB_DISTANCE_MM
         * 兜底: 时间 ≥ 8s 强制停车 (防超时)
         * 巡线: 仍用 Track_Loop 做灰度PD保持直行 (直道上就是直行) */
        Odom_Update();              /* 更新里程计 */
        Track_Loop();               /* 灰度PD巡线 (直道上保持直行) */
        /* 主判断: 走够 AB 距离 */
        if (Odom_Get_Total_Dist() >= AB_DISTANCE_MM) {
            Motor_Stop();
            g_chassis_task = Chassis_stop;
            g_running = 0;
        }
        /* 超时强制停车 (8s) */
        else if ((uint32_t)(g_sys_tick - g_task_start_tick) >= AB_TIME_LIMIT_MS) {
            Motor_Stop();
            g_chassis_task = Chassis_stop;
            g_running = 0;
        }
        break;
    case One_Lap:
        /* 第2/5题: 一圈 = 槽口型完整一圈 (两直道+两半圆弯)
         * 用过弯计数: 槽口型一圈有2个半圆弯, 过2弯=1圈?
         * TODO: 实测槽口型一圈的弯道数, 调整阈值 */
        Odom_Update();
        Track_Loop();
        if (g_corner_count >= 2) {   /* 槽口型2个弯=1圈 (待实测确认) */
            Motor_Stop();
            g_chassis_task = Chassis_stop;
            g_running = 0;
        }
        break;
    default:
        break;
    }
}

void Balance_Task(BalanceTask_e task)
{
    /* 视觉坐标 (g_vision 由 UART_K230 ISR 实时更新)
     * target_x/y 像素坐标, target_found=1=找到球
     * 以画面中心为原点, 取 X 方向作为摆杆控制量 (像素)
     *
     * !! 坐标系标定 (待实测) !!
     * VISION_CENTER_X/Y 需根据 K230 实际输出分辨率设置:
     *   - 若 K230 输出 800×480 → 中心 (400, 240)
     *   - 若 K230 输出 1280×720 → 中心 (640, 360)
     *   - 若 K230 输出 640×480 → 中心 (320, 240)
     * 标定方法: K230 对准摆杆中心, 读 g_vision.target_x/y, 即为中心值 */
    extern Vision_ProtocolTypeDef g_vision;

    /* K230 画面中心 (待实测标定, 默认假设 1280×720) */
    #define VISION_CENTER_X     640
    #define VISION_CENTER_Y     360

    float cur_pos = 0.0f;       /* 当前球位置 (像素, 0=中心) */
    float target_pos = 0.0f;    /* 目标位置 (像素) */

    if (g_vision.target_found) {
        cur_pos = (float)(g_vision.target_x - VISION_CENTER_X);
    }

    switch (task)
    {
    case Balance_stop:
        /* 不控球, 不下发指令 */
        break;

    case Balance_0:
        /* 第3题演示: 中心 → +5cm → -5cm → 停 -5cm
         * +5cm/-5cm 对应摆杆刻度, 需标定像素→cm换算
         * 暂用像素近似: 假设视野宽度对应摆杆±10cm, 640像素=10cm
         * → 1cm ≈ 64像素, 5cm ≈ 320像素
         * 序列状态机 (10ms/tick):
         *   阶段0: 等 START 启动, 进阶段1
         *   阶段1: 目标=+320像素(+5cm), 到位后停留1s, 进阶段2
         *   阶段2: 目标=-320像素(-5cm), 到位后停留1s, 进阶段3
         *   阶段3: 停在-5cm (目标=-320), 持续控球 */
        {
            /* 像素→cm 标定 (待实测调整): 视野宽 1280像素 = 摆杆±10cm → 64像素/cm */
            #define BAL_PIXELS_PER_CM   64.0f
            #define BAL_TARGET_5CM      (5.0f * BAL_PIXELS_PER_CM)   /* ±5cm 对应像素 */
            #define BAL_ARRIVE_TH       (1.0f * BAL_PIXELS_PER_CM)   /* 到位阈值 1cm */
            #define BAL_HOLD_TICKS      100   /* 到位后停留 100×10ms=1s */

            switch (bal0_phase) {
            case 0:   /* 启动: 进阶段1, 目标+5cm */
                bal0_phase = 1;
                bal0_tick  = 0;
                target_pos = BAL_TARGET_5CM;
                break;
            case 1:   /* 去+5cm, 到位停留1s */
                target_pos = BAL_TARGET_5CM;
                if ((cur_pos - target_pos) >= -BAL_ARRIVE_TH &&
                    (cur_pos - target_pos) <=  BAL_ARRIVE_TH) {
                    bal0_tick++;
                    if (bal0_tick >= BAL_HOLD_TICKS) {
                        bal0_phase = 2;
                        bal0_tick  = 0;
                    }
                } else {
                    bal0_tick = 0;
                }
                break;
            case 2:   /* 去-5cm, 到位停留1s */
                target_pos = -BAL_TARGET_5CM;
                if ((cur_pos - target_pos) >= -BAL_ARRIVE_TH &&
                    (cur_pos - target_pos) <=  BAL_ARRIVE_TH) {
                    bal0_tick++;
                    if (bal0_tick >= BAL_HOLD_TICKS) {
                        bal0_phase = 3;
                        bal0_tick  = 0;
                    }
                } else {
                    bal0_tick = 0;
                }
                break;
            case 3:   /* 停在-5cm */
            default:
                target_pos = -BAL_TARGET_5CM;
                break;
            }
            Balance_PID(target_pos, cur_pos, g_imu_data.GyroY);
        }
        break;

    case Balance_1:
        /* 第3题进阶: 锁定中心, 目标=0 (画面中心) */
        target_pos = 0.0f;
        Balance_PID(target_pos, cur_pos, g_imu_data.GyroY);
        break;

    case Balance_2:
        /* 第4/5题: 锁定任意位置 (暂用中心, 后续接视觉给定目标) */
        target_pos = 0.0f;
        Balance_PID(target_pos, cur_pos, g_imu_data.GyroY);
        break;

    default:
        break;
    }
}
