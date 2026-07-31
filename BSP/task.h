#ifndef TASK_H
#define TASK_H

#include <stdint.h>

typedef enum{
    Chassis_stop = 0,   // 停止
    CAR1,               // 分段PD跑一圈, 识别终止线停
    CAR2,               // 占位 (等数据)
    CAR3,               // A_to_B: B200参数跑1500mm停
    CAR4,               // 单PD跑一圈, 识别终止线停
    CAR5,               // 单PD跑一圈, 识别终止线停 (同CAR4)
    ChassisTaskNum,     // 任务总数 (用于循环边界)
} ChassisTask_e;

typedef enum{
    Balance_stop = 0, // 停止
    Balance_0,      // 中心到+5，+5到-5，停在-5
    Balance_1,      // 锁定中心
    Balance_2,      // 锁定任意位置
    BalanceTaskNum,
} BalanceTask_e;

extern volatile bool g_running;          // 小车运行标志位

extern volatile ChassisTask_e g_chassis_task;
extern volatile BalanceTask_e g_balance_task;


void Balance_Task();
void Chassis_Task();
void OLED_Task();
#endif /* TASK_H */
