#ifndef PITCH_AXIS_VISION_TELEMETRY_H
#define PITCH_AXIS_VISION_TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>

#include "ball_observation_protocol.h"
#include "bsp_bluetooth.h"
#include "bsp_uart_dma.h"
#include "pitch_axis_vision_control.h"

typedef struct
{
    BspBluetooth *output;
    const BspUartDmaPort *vision_port;
    const BallObservationParser *parser;
    const PitchAxisVisionControl *control;
    uint32_t next_status_ms;
    uint8_t boot_line;
    bool initialized;
} PitchAxisVisionTelemetry;

bool PitchAxisVisionTelemetry_Init(
    PitchAxisVisionTelemetry *telemetry,
    BspBluetooth *output,
    const BspUartDmaPort *vision_port,
    const BallObservationParser *parser,
    const PitchAxisVisionControl *control,
    uint32_t now_ms);

void PitchAxisVisionTelemetry_Service(
    PitchAxisVisionTelemetry *telemetry,
    uint32_t now_ms);

#endif
