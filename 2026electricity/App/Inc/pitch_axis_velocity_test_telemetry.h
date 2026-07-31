#ifndef PITCH_AXIS_VELOCITY_TEST_TELEMETRY_H
#define PITCH_AXIS_VELOCITY_TEST_TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>

#include "bsp_bluetooth.h"
#include "pitch_axis_velocity_test.h"

typedef struct
{
    BspBluetooth *output;
    PitchAxisVelocityTest *test;
    uint32_t next_status_ms;
    uint8_t boot_line;
    bool initialized;
} PitchAxisVelocityTestTelemetry;

bool PitchAxisVelocityTestTelemetry_Init(
    PitchAxisVelocityTestTelemetry *telemetry,
    PitchAxisVelocityTest *test,
    BspBluetooth *output,
    uint32_t now_ms);

void PitchAxisVelocityTestTelemetry_Service(
    PitchAxisVelocityTestTelemetry *telemetry,
    uint32_t now_ms);

#endif
