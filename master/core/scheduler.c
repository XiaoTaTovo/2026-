#include "core/scheduler.h"

void CarPeriodicTask_Init(CarPeriodicTask *task,
                          uint32_t period_ms,
                          uint32_t now_ms)
{
    if ((task == 0) || (period_ms == 0U)) {
        return;
    }//防御性编程，不用管 0u细节
    *task = (CarPeriodicTask){0};//初始化结构体
    task->period_ms = period_ms;//设置周期
    task->next_run_ms = now_ms + period_ms;//设置下一次运行时间
    task->initialized = true;//初始化成功
}//这个函数是用来初始化车子时钟片的，它传入一个CarPeriodicTask的结构体指针，你要的周期数，现在的时间，
//然后把结构体先全部清零，然后设置周期，然后设置下一次运行时间为现在的时间加上周期，最后把初始化标志设置为true

bool CarPeriodicTask_Due(CarPeriodicTask *task, uint32_t now_ms)
{
    uint32_t late_periods;

    if ((task == 0) || !task->initialized ||
        ((int32_t)(now_ms - task->next_run_ms) < 0)) {
        return false;
    }//如果任务为空，初始化了但是取反，因为或运算，还是false，

    late_periods = (uint32_t)(now_ms - task->next_run_ms) / task->period_ms;//丢失的周期数
    task->missed_count += late_periods;//用来看一共漏了几拍，方便排查
    task->next_run_ms += (late_periods + 1U) * task->period_ms;
    task->run_count++;
    return true;
}//这个函数是用来车子时钟片到了吗（due），它传入一个CarPeriodicTask的结构体指针和现在的时间，如果到了要执行就是true
//然后计算丢失的次数；把丢失的次数加一，乘以周期作为下一个跑的时刻