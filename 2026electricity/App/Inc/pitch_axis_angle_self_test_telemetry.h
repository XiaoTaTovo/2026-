#ifndef PITCH_AXIS_ANGLE_SELF_TEST_TELEMETRY_H
#define PITCH_AXIS_ANGLE_SELF_TEST_TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>

#include "bsp_bluetooth.h"
#include "pitch_axis_angle_self_test.h"

typedef struct
{
    BspBluetooth *output;
    const PitchAxisAngleSelfTest *self_test;
    uint8_t summary_line;
    bool start_announced;
    bool summary_started;
    bool summary_active;
    bool initialized;
} PitchAxisAngleSelfTestTelemetry;

bool PitchAxisAngleSelfTestTelemetry_Init(
    PitchAxisAngleSelfTestTelemetry *telemetry,
    const PitchAxisAngleSelfTest *self_test,
    BspBluetooth *output);

void PitchAxisAngleSelfTestTelemetry_Service(
    PitchAxisAngleSelfTestTelemetry *telemetry);

#endif
