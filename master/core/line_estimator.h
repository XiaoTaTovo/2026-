#ifndef H2024_LINE_ESTIMATOR_H
#define H2024_LINE_ESTIMATOR_H

#include "car_config.h"
#include "car_types.h"

typedef enum {
    CAR_LINE_PATTERN_NONE = 0,
    CAR_LINE_PATTERN_NARROW_TRACK,
    CAR_LINE_PATTERN_WIDE_AREA,
    CAR_LINE_PATTERN_SPLIT_NOISE
} CarLinePattern;

typedef struct {
    int16_t position;//线的位置，左边为负，右边为正，中间为0，记忆方法是想象数轴，以后也这样
    uint16_t confidence;
    uint8_t active_count;
    /* Bit i is set when gray channel i exceeds gray_min_signal.  Keeping the
     * mask lets endpoint detection distinguish two adjacent probes on a thin
     * line from two unrelated noisy probes. */
    uint8_t active_mask;
    /* Corner diagnostics derived from the same normalized frame. */
    uint16_t left_confidence;
    uint16_t right_confidence;
    uint16_t right_ratio_permille;
    uint8_t active_span;
    /* Adaptive diagnostics are independent of the legacy fields above. */
    int16_t track_position;
    uint16_t adaptive_background;
    uint16_t adaptive_contrast;
    uint16_t track_confidence;
    uint8_t track_active_count;
    uint8_t track_active_mask;
    uint8_t track_active_span;
    uint8_t track_cluster_count;
    CarLinePattern pattern;
    uint32_t timestamp_ms;
    bool valid;
} CarLineEstimate;

CarStatus CarLineEstimator_Update(const CarConfig *config,
                                  const CarGraySample *sample,
                                  CarLineEstimate *estimate);

#endif
