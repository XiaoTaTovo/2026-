#ifndef H2024_ROUTE_EXECUTOR_H
#define H2024_ROUTE_EXECUTOR_H

#include "car_config.h"
#include "car_types.h"
#include "core/line_estimator.h"
#include "core/odometry.h"

#define CAR_ROUTE_MAX_SEGMENTS (64U)

typedef enum {
    CAR_SEGMENT_STRAIGHT = 0,
    CAR_SEGMENT_TURN,
    CAR_SEGMENT_ARC,
    CAR_SEGMENT_CUE,
    CAR_SEGMENT_STOP,
    /* 2026 rectangle path: follow the line and exit on a guarded line event. */
    CAR_SEGMENT_LINE_FOLLOW,
    /* Official H2026 continuous track: center speed + signed curvature. */
    CAR_SEGMENT_TRACK
} CarSegmentType;

typedef enum {
    CAR_LINE_FOLLOW_EXIT_DISTANCE = 0,
    CAR_LINE_FOLLOW_EXIT_RIGHT_CORNER,
    CAR_LINE_FOLLOW_EXIT_LOST,
    /* STRAIGHT-only H2026 handoffs; the existing segment layout is retained. */
    CAR_LINE_FOLLOW_EXIT_SENSOR_AXLE,
    CAR_LINE_FOLLOW_EXIT_DIAGONAL_AXLE,
    /* Final A marker: require a late wide-line event, then apply offset. */
    CAR_LINE_FOLLOW_EXIT_WIDE_MARKER
} CarLineFollowExit;

typedef enum {
    CAR_LINE_FOLLOW_PHASE_TRACK = 0,
    CAR_LINE_FOLLOW_PHASE_AXLE_APPROACH,
    CAR_LINE_FOLLOW_PHASE_CROSSING_CAPTURE,
    CAR_LINE_FOLLOW_PHASE_SEARCH
} CarLineFollowPhase;

/* 只锁存运动段的退出原因，CUE/STOP 不覆盖它，便于停车后判断
 * 上一段到底是见线、到距离还是到角度退出。 */
typedef enum {
    CAR_ROUTE_EXIT_NONE = 0,
    CAR_ROUTE_EXIT_LINE,
    CAR_ROUTE_EXIT_DISTANCE,
    CAR_ROUTE_EXIT_ANGLE,
    CAR_ROUTE_EXIT_REACQUIRE,
    CAR_ROUTE_EXIT_LINE_MISS
} CarRouteExitReason;

typedef struct {
    CarSegmentType type;
    float value;
    float speed;
    uint32_t timeout_ms;
    bool use_line;
    CarCue cue;
    /* 改造1：ARC 段退出角度参数化。value 仍存半径（带符号表方向），
     * 这里单独存该弧要转过多少度才退出。之前 route_executor 里写死 180 度，
     * 现在改成读这个字段，可支持 90/180/任意角度的弧。
     * 注意 H2024_Add 会把它初始化成 180，保证 2024 老路线行为完全不变。 */
    /* ARC: angle in deg. TRACK: signed curvature in 1/mm. */
    float arc_angle_deg;
    CarLineFollowExit line_follow_exit;
    float line_event_arm_ratio;
    bool handoff_arc_heading;
} CarRouteSegment;

typedef struct {
    CarRouteSegment segments[CAR_ROUTE_MAX_SEGMENTS];
    uint16_t count;
} CarRoute;

typedef struct {
    const CarRoute *route;
    uint16_t index;
    uint32_t segment_start_ms;
    float segment_start_distance_mm;
    float segment_start_yaw_deg;//起始的yaw
    float line_integral;
    float line_previous_error;
    uint32_t line_previous_ms;
    uint8_t finish_line_streak;
    uint8_t line_event_streak;
    uint8_t turn_line_reacquire_streak;
    uint8_t track_enter_streak;
    uint8_t track_lost_streak;
    bool track_locked;
    bool line_follow_seen;
    bool line_corner_candidate;
    CarLineFollowPhase line_follow_phase;
    float line_corner_trigger_progress_mm;
    float line_corner_trigger_yaw_deg;
    float line_corner_approach_mm;
    float line_search_start_progress_mm;
    bool line_search_budget_active;
    int16_t line_capture_previous_position;
    bool line_capture_position_valid;
    float line_capture_guide_mm_s;
    float line_seek_correction_mm_s;
    float line_heading_reference_yaw_deg;
    bool line_heading_reference_valid;
    bool arc_line_locked;
    uint8_t arc_line_entry_streak;
    uint32_t arc_line_lock_ms;
    uint16_t arc_line_blend_permille;
    uint32_t post_turn_settle_until_ms;
    uint32_t semantic_line_miss_count;
    float pending_straight_yaw_deg;
    bool pending_straight_yaw_valid;
    float track_last_left_mm_s;
    float track_last_right_mm_s;
    uint32_t track_last_command_ms;
    float track_center_speed_mm_s;
    float track_curvature_inv_mm;
    float track_last_line_distance_mm;
    float track_line_gap_mm;
    bool track_command_valid;
    bool track_has_seen_line;
    uint16_t last_line_exit_confidence;
    uint8_t last_line_exit_active_count;
    uint8_t last_line_exit_active_mask;
    float last_line_exit_progress_mm;
    CarRouteExitReason last_motion_exit_reason;
    bool running;
    bool finished;
} CarRouteExecutor;

void CarRouteExecutor_Init(CarRouteExecutor *executor);
void CarRouteExecutor_ResetSegmentState(CarRouteExecutor *executor,
                                        uint32_t now_ms,
                                        float yaw_deg);
CarStatus CarRouteExecutor_Start(CarRouteExecutor *executor,
                                 const CarRoute *route,
                                 uint32_t now_ms,
                                 const CarOdometry *odometry,
                                 float yaw_deg);
CarStatus CarRouteExecutor_Update(CarRouteExecutor *executor,
                                  const CarConfig *config,
                                  uint32_t now_ms,
                                  const CarOdometry *odometry,
                                  float yaw_deg,
                                  const CarLineEstimate *line,
                                  CarMotorCommand *motor,
                                  CarCue *cue,
                                  uint32_t *faults);
bool CarRouteExecutor_GrayRequired(const CarRouteExecutor *executor);

#endif
