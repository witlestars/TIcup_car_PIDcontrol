/*
    用于任务切换

*/

#include "task.h"
#include "balance.h"

ChassisTask_e g_chassis_task = Chassis_stop;  // 小车任务标志位
BalanceTask_e g_balance_task = Balance_stop;     // 平衡任务标志位

void Chassis_Task(ChassisTask_e task)
{
    switch (task)
    {
    case Chassis_stop:
        /* code */
        break;
    case A_to_B:
        /* code */
        break;
    case One_Lap:
        /* code */
    default:
        break;
    }
}

void Balance_Task(BalanceTask_e task)
{
    switch (task)
    {
    case Balance_stop:
        /* code */
        break;
    case Balance_0:
        /* code */
        break;
    case Balance_1:
        /* code */
        break;
    case Balance_2:
        /* code */
    default:
        break;
    }

}