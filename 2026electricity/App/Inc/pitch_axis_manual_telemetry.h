#ifndef PITCH_AXIS_MANUAL_TELEMETRY_H
#define PITCH_AXIS_MANUAL_TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>

#include "bsp_bluetooth.h"
#include "pitch_axis_manual_control.h"

typedef struct
{
    BspBluetooth *output;
    PitchAxisManualControl *control;
    uint32_t next_status_ms;
    uint8_t boot_line;
    bool initialized;
} PitchAxisManualTelemetry;

bool PitchAxisManualTelemetry_Init(
    PitchAxisManualTelemetry *telemetry,
    PitchAxisManualControl *control,
    BspBluetooth *output,
    uint32_t now_ms);

void PitchAxisManualTelemetry_Service(
    PitchAxisManualTelemetry *telemetry,
    uint32_t now_ms);

#endif
