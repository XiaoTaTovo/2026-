#include "core/yaw_estimator.h"

void CarYawEstimator_Init(CarYawEstimator *estimator,
                          uint16_t calibration_samples,
                          uint32_t max_step_ms,
                          float initial_bias_dps,
                          bool fixed_bias)
{
    if (estimator == 0) {
        return;
    }
    *estimator = (CarYawEstimator){0};
    estimator->bias_dps = initial_bias_dps;
    estimator->fixed_bias = fixed_bias;
    estimator->calibration_target = fixed_bias ? 0U : calibration_samples;
    estimator->max_step_ms = max_step_ms;
    estimator->calibrated = fixed_bias || (calibration_samples == 0U);
}

void CarYawEstimator_ResetYaw(CarYawEstimator *estimator, float yaw_deg)
{
    if (estimator == 0) {
        return;
    }
    estimator->yaw_deg = yaw_deg;
}

bool CarYawEstimator_Update(CarYawEstimator *estimator,
                            float gyro_z_dps,
                            uint32_t timestamp_ms)
{
    uint32_t delta_ms;
    uint32_t reject_step_ms;

    if (estimator == 0) {
        return false;
    }

    if (!estimator->calibrated) {
        estimator->calibration_sum_dps += gyro_z_dps;
        estimator->calibration_count++;
        estimator->previous_timestamp_ms = timestamp_ms;
        estimator->timestamp_valid = true;
        if (estimator->calibration_count >= estimator->calibration_target) {
            estimator->bias_dps = estimator->calibration_sum_dps /
                                  (float)estimator->calibration_count;
            estimator->calibrated = true;
            estimator->yaw_deg = 0.0f;
        }
        return estimator->calibrated;
    }

    if (!estimator->timestamp_valid) {
        estimator->previous_timestamp_ms = timestamp_ms;
        estimator->timestamp_valid = true;
        return true;
    }

    delta_ms = (uint32_t)(timestamp_ms - estimator->previous_timestamp_ms);
    estimator->previous_timestamp_ms = timestamp_ms;
    if (delta_ms == 0U) {
        return false;
    }
    /* UI and telemetry must not erase real rotation. max_step_ms now marks a
     * delayed sample for diagnostics; only a true long outage is rejected. */
    if ((estimator->max_step_ms > 0U) &&
        (delta_ms > estimator->max_step_ms)) {
        estimator->delayed_step_count++;
    }
    reject_step_ms = estimator->max_step_ms * 10U;
    if (reject_step_ms < 200U) {
        reject_step_ms = 200U;
    }
    if (delta_ms > reject_step_ms) {
        estimator->rejected_step_count++;
        return false;
    }
    estimator->yaw_deg += (gyro_z_dps - estimator->bias_dps) *
                          ((float)delta_ms * 0.001f);
    return true;
}

bool CarYawEstimator_GetSample(const CarYawEstimator *estimator,
                               CarImuSample *sample)
{
    if ((estimator == 0) || (sample == 0) || !estimator->calibrated ||
        !estimator->timestamp_valid) {
        return false;
    }
    sample->yaw_deg = estimator->yaw_deg;
    sample->yaw_rate_dps = 0.0f;
    sample->timestamp_ms = estimator->previous_timestamp_ms;
    sample->valid = true;
    return true;
}
