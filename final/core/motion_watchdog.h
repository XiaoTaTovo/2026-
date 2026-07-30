#ifndef H2026_MOTION_WATCHDOG_H
#define H2026_MOTION_WATCHDOG_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t minimum_target_rpm;
    uint32_t no_motion_timeout_ms;
} CarMotionWatchdogConfig;

typedef struct {
    int32_t previous_encoder_count;
    uint32_t last_motion_ms;
    int8_t target_direction;
    bool encoder_count_valid;
    bool armed;
    bool timed_out;
} CarWheelMotionWatchdog;

typedef struct {
    CarMotionWatchdogConfig config;
    CarWheelMotionWatchdog left;
    CarWheelMotionWatchdog right;
    bool initialized;
} CarMotionWatchdog;

typedef struct {
    bool left_valid;
    bool right_valid;
} CarMotionWatchdogResult;

bool CarMotionWatchdog_Init(CarMotionWatchdog *watchdog,
                            const CarMotionWatchdogConfig *config);
void CarMotionWatchdog_Reset(CarMotionWatchdog *watchdog);
CarMotionWatchdogResult CarMotionWatchdog_Update(
    CarMotionWatchdog *watchdog,
    int32_t left_target_rpm,
    int32_t right_target_rpm,
    int32_t left_encoder_count,
    int32_t right_encoder_count,
    uint32_t now_ms);

#endif
