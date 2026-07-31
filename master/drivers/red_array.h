#ifndef H2026_RED_ARRAY_H
#define H2026_RED_ARRAY_H

#include <stdbool.h>
#include <stdint.h>

#include "car_types.h"

#define RED_ARRAY_CHANNELS (8U)
#define RED_ARRAY_FULL_SCALE (1000U)
/* RED is currently a weak-signal diagnostic path; keep this independent of GRAY. */
#define RED_ARRAY_MIN_CALIBRATION_SPAN (50)

/*
 * The legacy RED board is analog. The vendor LINE8 board exposes eight
 * independent GPIO levels instead: low means the black line is detected.
 */
typedef enum {
    RED_ARRAY_SIGNAL_ANALOG = 0,
    RED_ARRAY_SIGNAL_DIGITAL
} RedArraySignalType;

/* The platform adapter supplies one complete eight-channel raw frame. */
typedef bool (*RedArrayReadFrameFn)(
    uint16_t values[RED_ARRAY_CHANNELS], void *context);

typedef struct {
    RedArrayReadFrameFn read_frame;
    void *context;
    /* Digital mode uses an odd count for a strict-majority debounce. */
    uint8_t frames_to_average;
    RedArraySignalType signal_type;
    /* Bit n: X(n + 1) is asserted when its electrical level is low. */
    uint8_t digital_line_active_low_mask;
    /* Reverse X1..X8 when mapping the sensor into the vehicle frame. */
    bool digital_reverse_order;
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
