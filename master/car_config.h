#ifndef H2024_CAR_CONFIG_H
#define H2024_CAR_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float wheel_diameter_mm;
    float track_width_mm;
    float encoder_counts_per_wheel_rev;

    float straight_speed_mm_s;//直线基础速度，注意和现在已知的这个基准做转换，比如现在是350，所以用
    float arc_speed_mm_s;//弧线段速度上限，包括直角弯
    float turn_wheel_speed_mm_s;//转向速度上限
    float max_wheel_speed_mm_s;

    float distance_tolerance_mm;//车子有惯性，为了稳稳停到1m处，预留的这个距离
    float angle_tolerance_deg;//允许的误差最小角度
    float straight_heading_kp;//直线yaw的kp
    float arc_line_kp;
    float turn_heading_kp;//yaw转一定方向的kp
    float turn_min_speed_mm_s;//克服静摩擦力加机械死区

    uint32_t encoder_timeout_ms;
    uint32_t imu_timeout_ms;
    uint32_t gray_timeout_ms;

    uint32_t straight_timeout_ms;
    uint32_t arc_timeout_ms;
    uint32_t turn_timeout_ms;

    uint16_t gray_min_signal;
    uint16_t gray_min_confidence;
    float arc_line_ki;
    float arc_line_kd;
    float arc_line_integral_limit;
    /* 改造3：灰度中心偏移标定。理想情况下线在阵列正中时 position==0，
     * 但传感器安装/焊接误差会让零点漂移。这里存一个有符号偏移量，
     * line_estimator 算出 position 后统一减去它，把零点标定回真正的车体中线。
     * 默认 0 表示不做修正，保持旧行为不变。 */
    int16_t gray_center_offset;
    /* 终点横线使用独立强判定，不能复用灵敏的普通巡线 valid 条件。 */
    float gray_finish_arm_ratio;
    uint16_t gray_finish_min_confidence;
    uint8_t gray_finish_min_active;
    uint8_t gray_finish_consecutive_frames;
    int16_t line_corner_min_position;
    uint8_t line_corner_consecutive_frames;
    uint8_t line_lost_consecutive_frames;
    /* Rectangle straight-line tracking and right-corner handoff. */
    float line_heading_kp;
    float line_heading_max_correction_mm_s;
    uint16_t line_corner_min_right_ratio_permille;
    uint8_t line_corner_min_active;
    uint8_t line_corner_min_span;
    float line_sensor_to_axle_mm;
    float line_corner_approach_speed_mm_s;
    float turn_line_reacquire_min_angle_deg;
    int16_t turn_line_reacquire_max_position;
    uint8_t turn_line_reacquire_frames;
    /* Differential-drive geometry on a real skid-steer chassis differs from
     * the ruler-measured wheel-center spacing. ARC uses this empirical width;
     * odometry continues to use track_width_mm. A non-positive value keeps the
     * legacy behavior and falls back to track_width_mm. */
    float arc_effective_track_width_mm;
    /* Independent outer-loop limits keep heading/line corrections slower than
     * the 50 ms wheel-speed loop. Non-positive values retain legacy limits. */
    float straight_heading_max_correction_mm_s;
    float arc_line_max_correction_mm_s;
    /* H2026 ARC line-tail handoff. A non-positive angle or zero frames keeps
     * the legacy angle-only exit, so all H2024 routes remain unchanged. */
    float arc_line_exit_min_angle_deg;
    uint8_t arc_line_exit_lost_frames;
    /* Rectangle LINE_FOLLOW uses independent gains so runtime tuning cannot
     * disturb the already-validated semicircle ARC controller. */
    float line_follow_kp;
    float line_follow_ki;
    float line_follow_kd;
    float line_follow_integral_limit;
    /* Weak line evidence may steer only when this explicit safety limit is
     * positive; zero keeps the previous yaw-only approach bit-for-bit. */
    float line_seek_max_correction_mm_s;
    /* ARC keeps line_sensor_to_axle_mm as its physical gray-row offset. A
     * one-wheel pivot has different geometry and needs an independent value. */
    float line_corner_pivot_approach_mm;
    /* H2026 adaptive gray classification. Defaults keep the H2024 estimator
     * consumers on the legacy absolute-threshold fields. */
    bool gray_shape_filter_enable;
    uint16_t gray_relative_delta;
    uint16_t gray_track_min_confidence;
    uint8_t gray_track_enter_frames;
    uint8_t gray_track_lost_frames;
    uint8_t gray_track_max_active;
    uint8_t gray_track_max_span;
    uint8_t gray_wide_min_active;
    uint16_t gray_wide_min_background;
    /* ARC line acquisition is gated and blended to avoid an entry-frame jerk. */
    float arc_line_entry_min_angle_deg;
    uint8_t arc_line_entry_frames;
    uint16_t arc_line_blend_ms;
    /* Oblique marker capture and sensor-to-axle handoff. A non-positive arm
     * ratio falls back to gray_finish_arm_ratio for legacy configurations. */
    float diagonal_line_arm_ratio;
    float diagonal_line_approach_mm;
    int16_t line_cross_center_position;
    float line_cross_capture_window_mm;
    float line_cross_capture_speed_mm_s;
    float line_cross_capture_kp;
    /* A non-positive limit disables the capture guide. */
    float line_cross_capture_max_correction_mm_s;
    /* Legal forward-only turn geometry and post-turn stabilization. */
    float turn_inner_speed_ratio;
    uint16_t turn_settle_ms;
    float turn_settle_speed_mm_s;
    /* Required-line routes search briefly, then stop with LINE_MISSED. */
    float required_line_search_mm;
    float required_line_search_speed_mm_s;
    /* Front sensor to the official unique test point; measured along travel. */
    float finish_sensor_to_test_point_mm;
    float finish_approach_speed_mm_s;
    /* Per-wheel command slew. Non-positive keeps immediate legacy commands. */
    float track_wheel_accel_limit_mm_s2;
} CarConfig;

static inline CarConfig CarConfig_MakeDefault(void)
{
    CarConfig config = {
        65.0f,
        140.0f,
        1000.0f,
        180.0f,//直线基础速度，注意和现在已知的这个基准做转换，比如现在是350，所以用
        140.0f,
        90.0f,
        350.0f,//转向速度上限
        3.0f,//车子有惯性，为了稳稳停到1m处，预留的这个距离
        1.0f,//允许的误差最小角度，arc段也有
        2.0f,
        0.0f,
        5.0f,//yaw转向的kp，转速等于误差乘这个值
        40.0f,//克服静摩擦力加机械死区，刚好能让车从静止启动即可，不能太大
        150U,
        150U,
        100U,
        15000U,
        15000U,
        6000U,
        80U,
        200U,
        0.0f,
        0.0f,
        0.0f,
        0, /* 改造3：gray_center_offset 默认 0，标定完成后再填实测零点漂移值 */
        0.8f,  /* 只在目标距离最后20%开放终点检测，过滤途中污点 */
        1200U, /* 1cm细线约由两个探头命中，要求两路合计有足够黑度 */
        2U,    /* 两个相邻探头即可识别细线 */
        2U,    /* 连续2个控制帧：180mm/s下1cm线约可见55ms */
        2000,  /* 右侧线位置超过该值，可作为右直角到达证据 */
        2U,    /* 右侧拐角连续2帧，避免单帧位置毛刺 */
        3U,    /* 连续丢线3帧，才确认直角或A点线尾 */
        1.0f,  /* 直线巡线时yaw只做小幅辅助，灰度仍是主控制 */
        15.0f, /* yaw辅助修正限幅，避免与灰度抢方向 */
        650U,  /* 右半区灰度能量至少占65%，作为直角形态证据 */
        3U,    /* 直角形态至少覆盖3个有效探头 */
        3U,    /* 最左到最右有效探头跨度至少3路 */
        150.0f,/* 前置灰度阵列到轮轴转向支点的实测初值 */
        80.0f, /* 灰度确认拐角后，轮轴补偿阶段低速前进 */
        80.0f, /* yaw右转至少80度后才允许灰度重捕接管 */
        900,   /* 重捕时黑线必须接近中间 */
        2U,    /* 连续2帧居中才确认重捕 */
        0.0f,  /* <=0: ARC继续使用物理track_width_mm，保持旧路线行为 */
        0.0f,  /* <=0: 直线yaw修正继续使用旧的40%轮速限幅 */
        0.0f,  /* <=0: 弧线灰度修正继续使用旧的25%轮速限幅 */
        0.0f,  /* <=0: ARC只按角度退出，保持H2024旧路线行为 */
        0U,    /* 0: 不启用ARC线尾连续丢线判定 */
        0.0f,  /* LINE_FOLLOW gains are supplied by the H2026 platform. */
        0.0f,
        0.0f,
        0.0f,
        0.0f,  /* <=0: weak-line SEEK keeps the legacy yaw-only command. */
        70.0f, /* Real-car right-pivot approach baseline. */
        false, /* H2024 keeps legacy line.valid behavior. */
        120U,
        250U,
        2U,
        3U,
        4U,
        4U,
        6U,
        600U,
        8.0f,
        2U,
        200U,
        0.0f,  /* <=0: reuse gray_finish_arm_ratio. */
        130.0f,
        1200,
        80.0f,
        80.0f,
        0.0f,
        0.0f,  /* <=0: diagonal CAPTURE keeps the legacy yaw-only command. */
        0.0f,
        300U,
        80.0f,
        80.0f,
        60.0f,
        0.0f,
        0.0f,
        0.0f
    };
    return config;
}

#endif
