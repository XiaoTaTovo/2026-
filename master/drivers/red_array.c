#include "drivers/red_array.h"

#include <string.h>

static uint16_t RedArray_NormalizeLine(uint16_t raw,
                                       uint16_t black,
                                       uint16_t white)
{
    int32_t numerator;
    int32_t span;
    int32_t value;

    span = (int32_t)white - (int32_t)black;
    if (span == 0) {
        return 0U;
    }
    if (span > 0) {
        numerator = (int32_t)white - (int32_t)raw;
    } else {
        span = -span;
        numerator = (int32_t)raw - (int32_t)white;
    }
    value = (numerator * (int32_t)RED_ARRAY_FULL_SCALE) / span;
    if (value < 0) {
        return 0U;
    }
    if (value > (int32_t)RED_ARRAY_FULL_SCALE) {
        return RED_ARRAY_FULL_SCALE;
    }
    return (uint16_t)value;
}

void RedArray_Init(RedArray *array, const RedArrayPort *port)
{
    if ((array == 0) || (port == 0)) {
        return;
    }
    *array = (RedArray){0};
    array->port = *port;
}

bool RedArray_SetCalibration(
    RedArray *array,
    const uint16_t black[RED_ARRAY_CHANNELS],
    const uint16_t white[RED_ARRAY_CHANNELS])
{
    if ((array == 0) || (black == 0) || (white == 0)) {
        return false;
    }
    if (array->port.signal_type == RED_ARRAY_SIGNAL_DIGITAL) {
        array->calibrated = true;
        return true;
    }
    for (uint8_t i = 0U; i < RED_ARRAY_CHANNELS; i++) {
        int32_t span = (int32_t)white[i] - (int32_t)black[i];

        if ((span > -RED_ARRAY_MIN_CALIBRATION_SPAN) &&
            (span < RED_ARRAY_MIN_CALIBRATION_SPAN)) {
            array->calibrated = false;
            return false;
        }
    }
    memcpy(array->black, black, sizeof(array->black));
    memcpy(array->white, white, sizeof(array->white));
    array->calibrated = true;
    return true;
}

bool RedArray_Read(RedArray *array, uint32_t now_ms)
{
    uint32_t sums[RED_ARRAY_CHANNELS] = {0U};
    uint8_t frame_count;

    if ((array == 0) || (array->port.read_frame == 0)) {
        if (array != 0) {
            array->read_errors++;
            array->latest.valid = false;
        }
        return false;
    }
    frame_count = array->port.frames_to_average;
    if (frame_count == 0U) {
        frame_count = 1U;
    }

    for (uint8_t frame_index = 0U;
         frame_index < frame_count;
         frame_index++) {
        uint16_t frame[RED_ARRAY_CHANNELS];

        if (!array->port.read_frame(frame, array->port.context)) {
            array->read_errors++;
            array->latest.valid = false;
            return false;
        }
        for (uint8_t channel = 0U;
             channel < RED_ARRAY_CHANNELS;
             channel++) {
            uint8_t target = array->port.digital_reverse_order ?
                (uint8_t)((RED_ARRAY_CHANNELS - 1U) - channel) : channel;

            if (array->port.signal_type == RED_ARRAY_SIGNAL_DIGITAL) {
                bool electrical_high = frame[channel] != 0U;
                bool line_active = electrical_high;

                if ((array->port.digital_line_active_low_mask &
                     (uint8_t)(1U << channel)) != 0U) {
                    line_active = !electrical_high;
                }
                sums[target] += line_active ? 1U : 0U;
            } else {
                sums[target] += frame[channel];
            }
        }
    }

    for (uint8_t channel = 0U; channel < RED_ARRAY_CHANNELS; channel++) {
        if (array->port.signal_type == RED_ARRAY_SIGNAL_DIGITAL) {
            bool line_active = (sums[channel] * 2U) > frame_count;

            array->raw[channel] = line_active ? 0U : RED_ARRAY_FULL_SCALE;
            array->latest.normalized[channel] = line_active ?
                RED_ARRAY_FULL_SCALE : 0U;
        } else {
            array->raw[channel] = (uint16_t)(sums[channel] / frame_count);
            array->latest.normalized[channel] = array->calibrated ?
                RedArray_NormalizeLine(array->raw[channel],
                                       array->black[channel],
                                       array->white[channel]) : 0U;
        }
    }
    array->latest.timestamp_ms = now_ms;
    array->latest.valid = (array->port.signal_type == RED_ARRAY_SIGNAL_DIGITAL) ||
                          array->calibrated;
    return true;
}

bool RedArray_GetLatest(const RedArray *array, CarGraySample *sample)
{
    if ((array == 0) || (sample == 0) || !array->latest.valid) {
        return false;
    }
    *sample = array->latest;
    return true;
}
