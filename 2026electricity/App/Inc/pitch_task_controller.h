#ifndef PITCH_TASK_CONTROLLER_H
#define PITCH_TASK_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

#include "pitch_axis_velocity_test.h"
#include "pitch_axis_vision_control.h"

typedef enum
{
    PITCH_TASK_2 = 2,
    PITCH_TASK_3 = 3,
    PITCH_TASK_4 = 4,
    PITCH_TASK_5 = 5,
    PITCH_TASK_6 = 6
} PitchTaskId;

typedef enum
{
    PITCH_TASK_STATE_UNINITIALIZED = 0,
    PITCH_TASK_STATE_IDLE,
    PITCH_TASK_STATE_WAIT_CAPTURE,
    PITCH_TASK_STATE_STARTING,
    PITCH_TASK_STATE_RUNNING_POSITIVE,
    PITCH_TASK_STATE_RUNNING_NEGATIVE,
    PITCH_TASK_STATE_HOLDING,
    PITCH_TASK_STATE_FAULT
} PitchTaskState;

typedef struct
{
    int16_t center_position_0_1mm;
    uint16_t task3_offset_0_1mm;
    uint16_t task3_tolerance_0_1mm;
    uint16_t task3_velocity_limit_0_1mm_s;
    uint32_t task3_turnaround_dwell_ms;
    uint32_t button_debounce_ms;
} PitchTaskControllerConfig;

typedef struct
{
    PitchTaskId selected_task;
    PitchTaskState state;
    int16_t target_position_0_1mm;
    int16_t captured_position_0_1mm;
    bool captured_position_valid;
    bool automatic_armed;
    bool motor_enabled;
    bool fault_latched;
    uint8_t last_key;
    uint32_t rejected_key_count;
    uint32_t transition_count;
    uint32_t task_elapsed_ms;
    uint32_t target_reached_count;
} PitchTaskControllerReport;

typedef struct
{
    bool candidate_pressed;
    bool stable_pressed;
    uint32_t candidate_since_ms;
} PitchTaskButtonDebouncer;

typedef struct
{
    PitchAxisVisionControl *vision;
    PitchAxisVelocityTest *velocity;
    PitchTaskControllerConfig config;
    PitchTaskControllerReport report;
    PitchTaskButtonDebouncer buttons[3];
    uint32_t state_since_ms;
    uint32_t target_window_since_ms;
    bool initialized;
} PitchTaskController;

bool PitchTaskController_Init(
    PitchTaskController *controller,
    PitchAxisVisionControl *vision,
    PitchAxisVelocityTest *velocity,
    const PitchTaskControllerConfig *config,
    uint32_t now_ms);

void PitchTaskController_Service(
    PitchTaskController *controller,
    PitchAxisVelocityTestButtons buttons,
    uint32_t now_ms);

bool PitchTaskController_GetReport(
    const PitchTaskController *controller,
    PitchTaskControllerReport *report);

bool PitchTaskController_GetConfig(
    const PitchTaskController *controller,
    PitchTaskControllerConfig *config);

bool PitchTaskController_UpdateConfig(
    PitchTaskController *controller,
    const PitchTaskControllerConfig *config,
    uint32_t now_ms);

const char *PitchTaskController_StateName(PitchTaskState state);

#endif
