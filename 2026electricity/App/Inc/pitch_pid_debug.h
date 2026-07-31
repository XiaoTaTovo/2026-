#ifndef PITCH_PID_DEBUG_H
#define PITCH_PID_DEBUG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "bsp_bluetooth.h"
#include "pitch_axis_velocity_test.h"
#include "pitch_axis_vision_control.h"
#include "pitch_task_controller.h"

#define PITCH_PID_DEBUG_LINE_SIZE 96U
#define PITCH_PID_DEBUG_DEFAULT_SAMPLE_PERIOD_MS 100U
#define PITCH_PID_DEBUG_MIN_SAMPLE_PERIOD_MS 50U
#define PITCH_PID_DEBUG_MAX_SAMPLE_PERIOD_MS 1000U
#define PITCH_PID_DEBUG_SINGLE_COMMAND_TIMEOUT_MS 30U

typedef struct
{
    BspBluetooth *bluetooth;
    PitchAxisVisionControl *vision;
    PitchAxisVelocityTest *velocity;
    PitchTaskController *tasks;
    char line[PITCH_PID_DEBUG_LINE_SIZE];
    size_t line_length;
    uint32_t next_sample_ms;
    uint32_t sample_period_ms;
    uint32_t accepted_command_count;
    uint32_t rejected_command_count;
    uint32_t pending_single_since_ms;
    uint32_t last_task_transition_count;
    bool boot_report_pending;
    uint8_t boot_line;
    char pending_single_command;
    bool pending_single_active;
    bool enabled;
    bool initialized;
} PitchPidDebug;

bool PitchPidDebug_Init(
    PitchPidDebug *debug,
    BspBluetooth *bluetooth,
    PitchAxisVisionControl *vision,
    PitchAxisVelocityTest *velocity,
    PitchTaskController *tasks,
    uint32_t now_ms);

void PitchPidDebug_Service(
    PitchPidDebug *debug,
    uint32_t now_ms);

bool PitchPidDebug_IsEnabled(const PitchPidDebug *debug);

#endif
