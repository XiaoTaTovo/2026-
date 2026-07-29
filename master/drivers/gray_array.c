#include "drivers/gray_array.h"

#include <string.h>

static uint16_t GrayArray_NormalizeLine(uint16_t raw,
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
    value = (numerator * (int32_t)GRAY_ARRAY_FULL_SCALE) / span;
    if (value < 0) {
        return 0U;
    }
    if (value > (int32_t)GRAY_ARRAY_FULL_SCALE) {
        return GRAY_ARRAY_FULL_SCALE;
    }
    return (uint16_t)value;
}

void GrayArray_Init(GrayArray *array, const GrayArrayPort *port)
{
    if ((array == 0) || (port == 0)) {
        return;
    }
    *array = (GrayArray){0};
    array->port = *port;
}

bool GrayArray_SetCalibration(GrayArray *array,
                              const uint16_t black[GRAY_ARRAY_CHANNELS],
                              const uint16_t white[GRAY_ARRAY_CHANNELS])
{
    if ((array == 0) || (black == 0) || (white == 0)) {
        return false;
    }
    for (uint8_t i = 0U; i < GRAY_ARRAY_CHANNELS; i++) {
        int32_t span = (int32_t)white[i] - (int32_t)black[i];
        if ((span > -20) && (span < 20)) {
            array->calibrated = false;
            return false;
        }
    }
    memcpy(array->black, black, sizeof(array->black));
    memcpy(array->white, white, sizeof(array->white));
    array->calibrated = true;
    return true;
}

bool GrayArray_Read(GrayArray *array, uint32_t now_ms)
{
    uint8_t sample_count;

    if ((array == 0) || (array->port.select_channel == 0) ||
        (array->port.read_adc == 0)) {
        return false;
    }
    sample_count = array->port.samples_per_channel;
    if (sample_count == 0U) {
        sample_count = 1U;
    }

    for (uint8_t channel = 0U; channel < GRAY_ARRAY_CHANNELS; channel++) {
        uint32_t sum = 0U;

        if (!array->port.select_channel(channel, array->port.context)) {
            array->read_errors++;
            array->latest.valid = false;
            return false;
        }
        if ((array->port.delay_us != 0) && (array->port.settle_us > 0U)) {
            array->port.delay_us(array->port.settle_us, array->port.context);
        }
        for (uint8_t sample = 0U; sample < sample_count; sample++) {
            uint16_t value;
            if (!array->port.read_adc(&value, array->port.context)) {
                array->read_errors++;
                array->latest.valid = false;
                return false;
            }
            sum += value;
        }
        array->raw[channel] = (uint16_t)(sum / sample_count);
    }

    for (uint8_t channel = 0U; channel < GRAY_ARRAY_CHANNELS; channel++) {
        array->latest.normalized[channel] = array->calibrated ?
            GrayArray_NormalizeLine(array->raw[channel],
                                    array->black[channel],
                                    array->white[channel]) : 0U;
    }
    array->latest.timestamp_ms = now_ms;
    array->latest.valid = array->calibrated;
    return true;
}

bool GrayArray_GetLatest(const GrayArray *array, CarGraySample *sample)
{
    if ((array == 0) || (sample == 0) || !array->latest.valid) {
        return false;
    }
    *sample = array->latest;
    return true;
}

bool GrayArray_ClassifyLatest(const GrayArray *array,
                              uint16_t white_threshold,
                              uint16_t black_threshold,
                              GrayArrayClassification *classification)
{
    if ((array == 0) || (classification == 0) ||
        !array->calibrated || !array->latest.valid ||
        (white_threshold > black_threshold)) {
        return false;
    }

    *classification = (GrayArrayClassification){0};
    for (uint8_t channel = 0U; channel < GRAY_ARRAY_CHANNELS; channel++) {
        uint8_t bit = (uint8_t)(1U << channel);
        uint16_t value = array->latest.normalized[channel];

        if (value <= white_threshold) {
            classification->white_mask |= bit;
        } else if (value >= black_threshold) {
            classification->black_mask |= bit;
        } else {
            classification->unknown_mask |= bit;
        }
    }
    return true;
}
