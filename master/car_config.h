#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H

#include <stdint.h>

typedef struct {
    /* Measured chassis geometry. */
    float wheel_diameter_mm;
    float track_width_mm;
    float encoder_counts_per_wheel_rev;
    float arc_effective_track_width_mm;

    /* Official H track command limits. */
    float straight_speed_mm_s;
    float arc_speed_mm_s;
    float max_wheel_speed_mm_s;
    float track_wheel_accel_limit_mm_s2;
    float distance_tolerance_mm;

    /* Sensor freshness and segment deadlines. */
    uint32_t encoder_timeout_ms;
    uint32_t imu_timeout_ms;
    uint32_t gray_timeout_ms;
    uint32_t straight_timeout_ms;
    uint32_t arc_timeout_ms;

    /* Eight-channel line perception. */
    uint16_t gray_min_signal;
    uint16_t gray_min_confidence;
    int16_t gray_center_offset;
    uint16_t gray_relative_delta;
    uint16_t gray_track_min_confidence;
    uint8_t gray_track_enter_frames;
    uint8_t gray_track_lost_frames;
    uint8_t gray_track_max_active;
    uint8_t gray_track_max_span;
    uint8_t gray_wide_min_active;
    uint16_t gray_wide_min_background;

    /* Final A marker uses a stronger rule than ordinary line following. */
    uint16_t gray_finish_min_confidence;
    uint8_t gray_finish_min_active;
    uint8_t gray_finish_consecutive_frames;

    /* Outer line loop. Output unit is differential wheel speed in mm/s. */
    float line_kp;
    float line_ki;
    float line_kd;
    float line_integral_limit;
    float straight_line_max_correction_mm_s;
    float arc_line_max_correction_mm_s;

    /* Loss containment and final sensor-to-test-point compensation. */
    float required_line_search_mm;
    float required_line_search_speed_mm_s;
    float finish_sensor_to_test_point_mm;
    float finish_approach_speed_mm_s;
} CarConfig;

static inline CarConfig CarConfig_MakeDefault(void)
{
    return (CarConfig){
        .wheel_diameter_mm = 65.0f,
        .track_width_mm = 115.0f,
        .encoder_counts_per_wheel_rev = 724.0f,
        .arc_effective_track_width_mm = 160.0f,
        .straight_speed_mm_s = 350.0f,
        .arc_speed_mm_s = 300.0f,
        .max_wheel_speed_mm_s = 350.0f,
        .track_wheel_accel_limit_mm_s2 = 1500.0f,
        .distance_tolerance_mm = 3.0f,
        .encoder_timeout_ms = 150U,
        .imu_timeout_ms = 150U,
        .gray_timeout_ms = 100U,
        .straight_timeout_ms = 8000U,
        .arc_timeout_ms = 8000U,
        .gray_min_signal = 80U,
        .gray_min_confidence = 200U,
        .gray_center_offset = 0,
        .gray_relative_delta = 120U,
        .gray_track_min_confidence = 250U,
        .gray_track_enter_frames = 2U,
        .gray_track_lost_frames = 3U,
        .gray_track_max_active = 4U,
        .gray_track_max_span = 4U,
        .gray_wide_min_active = 6U,
        .gray_wide_min_background = 600U,
        .gray_finish_min_confidence = 1200U,
        .gray_finish_min_active = 2U,
        .gray_finish_consecutive_frames = 2U,
        .line_kp = 0.025f,
        .line_ki = 0.0f,
        .line_kd = 0.0f,
        .line_integral_limit = 5000.0f,
        .straight_line_max_correction_mm_s = 87.5f,
        .arc_line_max_correction_mm_s = 20.0f,
        .required_line_search_mm = 80.0f,
        .required_line_search_speed_mm_s = 60.0f,
        .finish_sensor_to_test_point_mm = 150.0f,
        .finish_approach_speed_mm_s = 180.0f
    };
}

#endif
