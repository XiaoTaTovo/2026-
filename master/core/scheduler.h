#ifndef H2024_SCHEDULER_H
#define H2024_SCHEDULER_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t period_ms;
    uint32_t next_run_ms;
    uint32_t run_count;
    uint32_t missed_count;
    bool initialized;
} CarPeriodicTask;

void CarPeriodicTask_Init(CarPeriodicTask *task,
                          uint32_t period_ms,
                          uint32_t now_ms);
bool CarPeriodicTask_Due(CarPeriodicTask *task, uint32_t now_ms);

#endif
