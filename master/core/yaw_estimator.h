#ifndef H2024_YAW_ESTIMATOR_H
#define H2024_YAW_ESTIMATOR_H

#include <stdbool.h>
#include <stdint.h>

#include "car_types.h"

typedef struct {
    float bias_dps;
    float yaw_deg;
    float calibration_sum_dps;
    uint16_t calibration_target;
    uint16_t calibration_count;
    uint32_t previous_timestamp_ms;
    uint32_t max_step_ms;
    uint32_t delayed_step_count;
    uint32_t rejected_step_count;
    /* Fixed-bias mode is retained for repeatable bench tests. */
    bool fixed_bias;
    bool calibrated;
    bool timestamp_valid;
} CarYawEstimator;

void CarYawEstimator_Init(CarYawEstimator *estimator,
                          uint16_t calibration_samples,
                          uint32_t max_step_ms,
                          float initial_bias_dps,
                          bool fixed_bias);
void CarYawEstimator_ResetYaw(CarYawEstimator *estimator, float yaw_deg);
bool CarYawEstimator_Update(CarYawEstimator *estimator,
                            float gyro_z_dps,
                            uint32_t timestamp_ms);
bool CarYawEstimator_GetSample(const CarYawEstimator *estimator,
                               CarImuSample *sample);

#endif
