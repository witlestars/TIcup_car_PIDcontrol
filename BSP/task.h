#ifndef TASK_H
#define TASK_H

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

#endif /* TASK_H */
