#ifndef TASK_H
#define TASK_H

#include <stdint.h>

/* H题题目号 (按钮 LAP_UP 切换, 1~5 循环)
 *   1=图传 2=循线一圈 3=静止控球 4=A→B 5=一圈稳球
 * 对应映射:
 *   1,2 → Chassis_stop + Balance_stop       (纯视觉/巡线, 不控球)
 *   3   → Chassis_stop + Balance_1           (锁定中心)
 *   4   → A_to_B     + Balance_2             (A→B + 控球)
 *   5   → One_Lap    + Balance_2             (一圈 + 控球) */
extern uint8_t g_task_id;

/* 运行标志: 1=运行 0=停止 (BTN_START 切换, 上电默认停止) */
extern uint8_t g_running;

/* 1ms 时基 (定义在 main.c) */
extern volatile uint32_t g_sys_tick;

typedef enum{
    Chassis_stop = 0,   // 停止
    A_to_B,             // A点到B点
    One_Lap,            // 一圈
} ChassisTask_e;

typedef enum{
    Balance_stop = 0, // 停止
    Balance_0,      // 中心到+5，+5到-5，停在-5
    Balance_1,      // 锁定中心
    Balance_2,      // 锁定任意位置
} BalanceTask_e;

extern ChassisTask_e g_chassis_task;
extern BalanceTask_e g_balance_task;


void Balance_Task(BalanceTask_e task);
void Chassis_Task(ChassisTask_e task);

/* 根据 g_task_id 设置 g_chassis_task / g_balance_task (按钮切题时调用) */
void Task_ApplyTaskId(void);

/* 任务启动时调用 (按 START 时复位里程计/巡线/时间戳) */
void Task_OnStart(void);

#endif /* TASK_H */
