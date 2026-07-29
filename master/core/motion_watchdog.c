#include "core/motion_watchdog.h"

#include <limits.h>

static int8_t CarMotionWatchdog_TargetDirection(
    int32_t target_rpm,
    uint32_t minimum_target_rpm)
{
    uint32_t magnitude = (target_rpm < 0) ?
        (uint32_t)(-(int64_t)target_rpm) : (uint32_t)target_rpm;

    if (magnitude < minimum_target_rpm) {
        return 0;
    }
    return (target_rpm < 0) ? -1 : 1;
}

static bool CarMotionWatchdog_UpdateWheel(
    CarWheelMotionWatchdog *wheel,
    const CarMotionWatchdogConfig *config,
    int32_t target_rpm,
    int32_t encoder_count,
    uint32_t now_ms)
{
    int8_t direction = CarMotionWatchdog_TargetDirection(
        target_rpm, config->minimum_target_rpm);
    bool encoder_moved = wheel->encoder_count_valid &&
        (encoder_count != wheel->previous_encoder_count);

    wheel->previous_encoder_count = encoder_count;
    wheel->encoder_count_valid = true;

    if (direction == 0) {
        wheel->last_motion_ms = now_ms;
        wheel->target_direction = 0;
        wheel->armed = false;
        wheel->timed_out = false;
        return true;
    }

    if (!wheel->armed || (wheel->target_direction != direction)) {
        wheel->last_motion_ms = now_ms;
        wheel->target_direction = direction;
        wheel->armed = true;
        wheel->timed_out = false;
        return true;
    }

    if (wheel->timed_out) {
        return false;
    }
    if (encoder_moved) {
        wheel->last_motion_ms = now_ms;
        return true;
    }
    if ((uint32_t)(now_ms - wheel->last_motion_ms) >=
        config->no_motion_timeout_ms) {
        wheel->timed_out = true;
        return false;
    }
    return true;
}

bool CarMotionWatchdog_Init(CarMotionWatchdog *watchdog,
                            const CarMotionWatchdogConfig *config)
{
    if ((watchdog == 0) || (config == 0) ||
        (config->minimum_target_rpm == 0U) ||
        (config->minimum_target_rpm > (uint32_t)INT32_MAX) ||
        (config->no_motion_timeout_ms == 0U)) {
        return false;
    }

    *watchdog = (CarMotionWatchdog){0};
    watchdog->config = *config;
    watchdog->initialized = true;
    return true;
}

void CarMotionWatchdog_Reset(CarMotionWatchdog *watchdog)
{
    if ((watchdog == 0) || !watchdog->initialized) {
        return;
    }
    watchdog->left = (CarWheelMotionWatchdog){0};
    watchdog->right = (CarWheelMotionWatchdog){0};
}

CarMotionWatchdogResult CarMotionWatchdog_Update(
    CarMotionWatchdog *watchdog,
    int32_t left_target_rpm,
    int32_t right_target_rpm,
    int32_t left_encoder_count,
    int32_t right_encoder_count,
    uint32_t now_ms)
{
    CarMotionWatchdogResult result = {false, false};

    if ((watchdog == 0) || !watchdog->initialized) {
        return result;
    }
    result.left_valid = CarMotionWatchdog_UpdateWheel(
        &watchdog->left, &watchdog->config, left_target_rpm,
        left_encoder_count, now_ms);
    result.right_valid = CarMotionWatchdog_UpdateWheel(
        &watchdog->right, &watchdog->config, right_target_rpm,
        right_encoder_count, now_ms);
    return result;
}
