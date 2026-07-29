#include "core/safety_supervisor.h"

static bool CarSafety_IsStale(uint32_t now_ms,
                              uint32_t timestamp_ms,
                              uint32_t timeout_ms)
{
    /* A live sampler can timestamp one tick after the caller snapshots now. */
    return (int32_t)(now_ms - timestamp_ms) > (int32_t)timeout_ms;
}

uint32_t CarSafety_Evaluate(const CarConfig *config,
                            uint32_t now_ms,
                            const CarInputSnapshot *input,
                            bool motion_requested,
                            bool gray_required)
{
    uint32_t faults = CAR_FAULT_NONE;

    if ((config == 0) || (input == 0)) {
        return CAR_FAULT_ROUTE_INVALID;
    }

    if (input->emergency_stop) {
        faults |= CAR_FAULT_EMERGENCY_STOP;
    }

    if (!motion_requested) {
        return faults;
    }

    if (!input->encoder.valid ||
        CarSafety_IsStale(now_ms, input->encoder.timestamp_ms,
                          config->encoder_timeout_ms)) {
        faults |= CAR_FAULT_ENCODER_STALE;
    }
    if (!input->imu.valid ||
        CarSafety_IsStale(now_ms, input->imu.timestamp_ms,
                          config->imu_timeout_ms)) {
        faults |= CAR_FAULT_IMU_STALE;
    }
    if (gray_required &&
        (!input->gray.valid ||
         CarSafety_IsStale(now_ms, input->gray.timestamp_ms,
                           config->gray_timeout_ms))) {
        faults |= CAR_FAULT_GRAY_STALE;
    }

    return faults;
}
