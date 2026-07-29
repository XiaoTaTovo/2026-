#ifndef ROUTE_EXECUTOR_H
#define ROUTE_EXECUTOR_H

#include "car_config.h"
#include "car_types.h"
#include "core/line_estimator.h"
#include "core/odometry.h"

#define CAR_ROUTE_MAX_SEGMENTS (8U)

typedef enum {
    CAR_SEGMENT_TRACK = 0,
    CAR_SEGMENT_CUE,
    CAR_SEGMENT_STOP
} CarSegmentType;

typedef enum {
    CAR_TRACK_EXIT_DISTANCE = 0,
    CAR_TRACK_EXIT_FINISH_MARKER
} CarTrackExit;

typedef enum {
    CAR_TRACK_PHASE_FOLLOW = 0,
    CAR_TRACK_PHASE_FINISH_OFFSET
} CarTrackPhase;

typedef enum {
    CAR_ROUTE_EXIT_NONE = 0,
    CAR_ROUTE_EXIT_DISTANCE,
    CAR_ROUTE_EXIT_FINISH_MARKER,
    CAR_ROUTE_EXIT_LINE_MISS,
    CAR_ROUTE_EXIT_TIMEOUT
} CarRouteExitReason;

typedef struct {
    CarSegmentType type;
    float distance_mm;
    float center_speed_mm_s;
    uint32_t timeout_ms;
    bool use_line;
    CarCue cue;
    float curvature_inv_mm;
    CarTrackExit track_exit;
    float marker_arm_ratio;
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
    float segment_progress_mm;

    float line_integral;
    float line_previous_error;
    uint32_t line_previous_ms;
    float line_correction_mm_s;

    uint8_t track_enter_streak;
    uint8_t track_lost_streak;
    bool track_locked;
    bool track_has_seen_line;
    float track_last_line_distance_mm;
    float track_line_gap_mm;

    CarTrackPhase track_phase;
    uint8_t marker_streak;
    float finish_trigger_progress_mm;
    float finish_offset_progress_mm;
    uint16_t last_marker_confidence;
    uint8_t last_marker_active_count;
    uint8_t last_marker_active_mask;

    float track_last_left_mm_s;
    float track_last_right_mm_s;
    uint32_t track_last_command_ms;
    float track_center_speed_mm_s;
    float track_curvature_inv_mm;
    bool track_command_valid;

    uint32_t line_miss_count;
    CarRouteExitReason last_exit_reason;
    bool running;
    bool finished;
} CarRouteExecutor;

void CarRouteExecutor_Init(CarRouteExecutor *executor);
CarStatus CarRouteExecutor_Start(CarRouteExecutor *executor,
                                 const CarRoute *route,
                                 uint32_t now_ms,
                                 const CarOdometry *odometry);
CarStatus CarRouteExecutor_Update(CarRouteExecutor *executor,
                                  const CarConfig *config,
                                  uint32_t now_ms,
                                  const CarOdometry *odometry,
                                  const CarLineEstimate *line,
                                  CarMotorCommand *motor,
                                  CarCue *cue,
                                  uint32_t *faults);
bool CarRouteExecutor_GrayRequired(const CarRouteExecutor *executor);

#endif
