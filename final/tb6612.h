#ifndef TB6612_H
#define TB6612_H

#include <stdbool.h>
#include <stdint.h>

#include "car_types.h"
#include "core/motion_watchdog.h"

#define TB6612_PWM_PERIOD_TICKS (1600U)
#define TB6612_MAX_DUTY_PERCENT (80U)//最大占空比

/* Set to 1 when IN1=High is physical forward, otherwise set to 0. */
#define TB6612_LEFT_FORWARD_IN1_HIGH (0)
#define TB6612_RIGHT_FORWARD_IN1_HIGH (0)

typedef uint32_t (*TB6612NowFn)(void *context);

typedef struct {
    float wheel_diameter_mm;
    uint32_t left_counts_per_rev;
    uint32_t right_counts_per_rev;
    uint32_t control_period_ms;
    uint32_t kp_milli;
    uint32_t ki_milli;
    uint32_t kd_milli;
    int32_t feedforward_static_milli;
    uint32_t feedforward_rpm_milli;
    int32_t error_deadband_rpm;
    uint8_t output_limit_percent;
    uint32_t motion_watchdog_min_target_rpm;
    uint32_t motion_watchdog_timeout_ms;
} TB6612SpeedLoopConfig;

typedef struct {
    bool enabled;
    int32_t target_left_rpm;
    int32_t target_right_rpm;
    int32_t measured_left_rpm;
    int32_t measured_right_rpm;
    int32_t left_delta_count;
    int32_t right_delta_count;
    int8_t left_output_percent;
    int8_t right_output_percent;
    uint32_t update_count;
    uint32_t sample_elapsed_ms;
} TB6612SpeedLoopStatus;

typedef struct {
    TB6612NowFn now_ms;
    void *now_context;
    TB6612SpeedLoopConfig speed_loop;
    CarMotionWatchdog motion_watchdog;
    int32_t target_left_rpm;
    int32_t target_right_rpm;
    int32_t measured_left_rpm;
    int32_t measured_right_rpm;
    int32_t left_delta_count;
    int32_t right_delta_count;
    int32_t previous_left_count;
    int32_t previous_right_count;
    int32_t left_previous_error;
    int32_t right_previous_error;
    int64_t left_integral_milli;
    int64_t right_integral_milli;
    uint32_t previous_control_ms;
    uint32_t update_count;
    uint32_t last_sample_elapsed_ms;
    bool speed_loop_enabled;
    bool motion_watchdog_enabled;
    bool encoder_sample_ready;
    bool speed_filter_ready;
} TB6612Drive;

void TB6612_Init(void);
void TB6612_Stop(void);
void TB6612_SetMotors(int8_t left_percent, int8_t right_percent);
int8_t TB6612_GetLeftCommand(void);
int8_t TB6612_GetRightCommand(void);

void TB6612_DriveInit(TB6612Drive *drive,
                      TB6612NowFn now_ms,
                      void *now_context);
bool TB6612_DriveConfigureSpeedLoop(TB6612Drive *drive,
                                    const TB6612SpeedLoopConfig *config);
bool TB6612_DriveGetSpeedLoopConfig(const TB6612Drive *drive,
                                    TB6612SpeedLoopConfig *config);
bool TB6612_DriveUpdateSpeedLoopTuning(TB6612Drive *drive,
                                       uint32_t kp_milli,
                                       uint32_t ki_milli,
                                       uint32_t kd_milli,
                                       uint8_t output_limit_percent);
void TB6612_DriveGetSpeedLoopStatus(const TB6612Drive *drive,
                                    TB6612SpeedLoopStatus *status);
void TB6612_DriveService(TB6612Drive *drive);
bool TB6612_DriveSetWheelSpeeds(int16_t left_mm_s,
                                int16_t right_mm_s,
                                void *context);
bool TB6612_DriveReadEncoder(CarEncoderSample *sample, void *context);

#endif
