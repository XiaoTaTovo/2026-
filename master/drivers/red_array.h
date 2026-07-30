#ifndef H2026_RED_ARRAY_H
#define H2026_RED_ARRAY_H

#include <stdbool.h>
#include <stdint.h>

#include "car_types.h"

#define RED_ARRAY_CHANNELS (8U)
#define RED_ARRAY_FULL_SCALE (1000U)
/* RED is currently a weak-signal diagnostic path; keep this independent of GRAY. */
#define RED_ARRAY_MIN_CALIBRATION_SPAN (50)

/* The platform adapter supplies one complete eight-channel raw frame. */
typedef bool (*RedArrayReadFrameFn)(
    uint16_t values[RED_ARRAY_CHANNELS], void *context);

typedef struct {
    RedArrayReadFrameFn read_frame;
    void *context;
    uint8_t frames_to_average;
} RedArrayPort;

typedef struct {
    RedArrayPort port;
    uint16_t black[RED_ARRAY_CHANNELS];
    uint16_t white[RED_ARRAY_CHANNELS];
    uint16_t raw[RED_ARRAY_CHANNELS];
    CarGraySample latest;
    uint32_t read_errors;
    bool calibrated;
} RedArray;

void RedArray_Init(RedArray *array, const RedArrayPort *port);
bool RedArray_SetCalibration(
    RedArray *array,
    const uint16_t black[RED_ARRAY_CHANNELS],
    const uint16_t white[RED_ARRAY_CHANNELS]);
bool RedArray_Read(RedArray *array, uint32_t now_ms);
bool RedArray_GetLatest(const RedArray *array, CarGraySample *sample);

#endif
