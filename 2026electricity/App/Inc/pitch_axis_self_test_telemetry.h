#ifndef PITCH_AXIS_SELF_TEST_TELEMETRY_H
#define PITCH_AXIS_SELF_TEST_TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>

#include "bsp_bluetooth.h"
#include "pitch_axis_self_test.h"

typedef struct
{
    BspBluetooth *output;
    const PitchAxisSelfTest *self_test;
    uint32_t progress_reported;
    uint8_t summary_line;
    bool start_announced;
    bool summary_started;
    bool summary_active;
    bool initialized;
} PitchAxisSelfTestTelemetry;

bool PitchAxisSelfTestTelemetry_Init(
    PitchAxisSelfTestTelemetry *telemetry,
    const PitchAxisSelfTest *self_test,
    BspBluetooth *output);

void PitchAxisSelfTestTelemetry_Service(
    PitchAxisSelfTestTelemetry *telemetry);

#endif
