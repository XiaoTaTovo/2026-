#include "core/route_executor.h"

static float CarRoute_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float CarRoute_Clamp(float value, float limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

static float CarRoute_MoveToward(float current,
                                 float target,
                                 float max_delta)
{
    float delta = target - current;

    if (max_delta <= 0.0f) {
        return current;
    }
    if (CarRoute_Abs(delta) <= max_delta) {
        return target;
    }
    return current + ((delta > 0.0f) ? max_delta : -max_delta);
}

static float CarRoute_LimitAcceleration(float current,
                                        float target,
                                        float max_delta)
{
    /* Slowing down is immediate; only increasing wheel authority is ramped. */
    if (CarRoute_Abs(target) <= CarRoute_Abs(current)) {
        return target;
    }
    return CarRoute_MoveToward(current, target, max_delta);
}

static void CarRoute_Stop(CarMotorCommand *motor)
{
    *motor = (CarMotorCommand){0};
}

static bool CarRoute_IsNarrowTrack(const CarLineEstimate *line)
{
    return (line != 0) &&
           (line->pattern == CAR_LINE_PATTERN_NARROW_TRACK);
}

static bool CarRoute_HasAdjacentActive(uint8_t active_mask)
{
    return (active_mask & (uint8_t)(active_mask >> 1U)) != 0U;
}

static bool CarRoute_IsFinishMarker(const CarConfig *config,
                                    const CarLineEstimate *line)
{
    bool coherent_wide;

    if ((config == 0) || (line == 0) || !line->valid ||
        (line->confidence < config->gray_finish_min_confidence) ||
        (line->active_count < config->gray_finish_min_active)) {
        return false;
    }
    coherent_wide =
        (config->gray_wide_min_active > 0U) &&
        (line->active_count >= config->gray_wide_min_active) &&
        (line->active_span == line->active_count) &&
        (line->adaptive_background >=
         config->gray_wide_min_background) &&
        CarRoute_HasAdjacentActive(line->active_mask);
    return (line->pattern == CAR_LINE_PATTERN_WIDE_AREA) || coherent_wide;
}

static void CarRoute_UpdateTrackLock(CarRouteExecutor *executor,
                                     const CarConfig *config,
                                     const CarLineEstimate *line)
{
    uint8_t enter_frames = (config->gray_track_enter_frames == 0U) ? 1U :
                           config->gray_track_enter_frames;
    uint8_t lost_frames = (config->gray_track_lost_frames == 0U) ? 1U :
                          config->gray_track_lost_frames;
    CarLinePattern pattern = (line == 0) ? CAR_LINE_PATTERN_NONE :
                             line->pattern;

    if (pattern == CAR_LINE_PATTERN_NARROW_TRACK) {
        executor->track_lost_streak = 0U;
        if (executor->track_enter_streak < UINT8_MAX) {
            executor->track_enter_streak++;
        }
        if (executor->track_enter_streak >= enter_frames) {
            executor->track_locked = true;
        }
    } else if (pattern == CAR_LINE_PATTERN_NONE) {
        executor->track_enter_streak = 0U;
        if (executor->track_lost_streak < UINT8_MAX) {
            executor->track_lost_streak++;
        }
        if (executor->track_lost_streak >= lost_frames) {
            executor->track_locked = false;
        }
    } else {
        /* A broad marker or split footprint is ambiguous, not a new track. */
        executor->track_enter_streak = 0U;
        executor->track_lost_streak = 0U;
    }
}

static float CarRoute_LinePid(CarRouteExecutor *executor,
                              const CarConfig *config,
                              int16_t position,
                              uint32_t now_ms)
{
    float error = (float)position;
    float dt_s = (float)(now_ms - executor->line_previous_ms) / 1000.0f;
    float derivative = 0.0f;

    if ((dt_s > 0.0f) && (dt_s <= 0.25f)) {
        derivative = (error - executor->line_previous_error) / dt_s;
        executor->line_integral += error * dt_s;
    }
    if (config->line_integral_limit > 0.0f) {
        executor->line_integral = CarRoute_Clamp(
            executor->line_integral, config->line_integral_limit);
    } else {
        executor->line_integral = 0.0f;
    }
    executor->line_previous_error = error;
    executor->line_previous_ms = now_ms;
    return error * config->line_kp +
           executor->line_integral * config->line_ki +
           derivative * config->line_kd;
}

static void CarRoute_ResetLineState(CarRouteExecutor *executor,
                                    uint32_t now_ms,
                                    float distance_mm)
{
    executor->line_integral = 0.0f;
    executor->line_previous_error = 0.0f;
    executor->line_previous_ms = now_ms;
    executor->line_correction_mm_s = 0.0f;
    executor->track_enter_streak = 0U;
    executor->track_lost_streak = 0U;
    executor->track_locked = false;
    executor->track_has_seen_line = false;
    executor->track_last_line_distance_mm = distance_mm;
    executor->track_line_gap_mm = 0.0f;
}

static void CarRoute_ResetSegmentState(CarRouteExecutor *executor)
{
    executor->segment_progress_mm = 0.0f;
    executor->track_phase = CAR_TRACK_PHASE_FOLLOW;
    executor->marker_streak = 0U;
    executor->finish_trigger_progress_mm = 0.0f;
    executor->finish_offset_progress_mm = 0.0f;
    executor->track_center_speed_mm_s = 0.0f;
    executor->track_curvature_inv_mm = 0.0f;
    executor->line_correction_mm_s = 0.0f;
}

static void CarRoute_Advance(CarRouteExecutor *executor,
                             uint32_t now_ms,
                             const CarOdometry *odometry,
                             CarRouteExitReason reason)
{
    bool continuous_track = false;

    if (reason != CAR_ROUTE_EXIT_NONE) {
        executor->last_exit_reason = reason;
    }
    if ((executor->route != 0) &&
        (executor->index + 1U < executor->route->count)) {
        continuous_track =
            (executor->route->segments[executor->index].type ==
             CAR_SEGMENT_TRACK) &&
            (executor->route->segments[executor->index + 1U].type ==
             CAR_SEGMENT_TRACK);
    }
    executor->index++;
    executor->segment_start_ms = now_ms;
    executor->segment_start_distance_mm = odometry->center_distance_mm;
    CarRoute_ResetSegmentState(executor);
    if (!continuous_track) {
        CarRoute_ResetLineState(executor, now_ms,
                                odometry->center_distance_mm);
        executor->track_command_valid = false;
    }
    if ((executor->route == 0) ||
        (executor->index >= executor->route->count)) {
        executor->running = false;
        executor->finished = true;
    }
}

static CarStatus CarRoute_Fail(CarRouteExecutor *executor,
                               CarMotorCommand *motor,
                               uint32_t *faults,
                               uint32_t fault,
                               CarRouteExitReason reason)
{
    *faults |= fault;
    executor->last_exit_reason = reason;
    if (fault == CAR_FAULT_LINE_MISSED) {
        executor->line_miss_count++;
    }
    executor->running = false;
    CarRoute_Stop(motor);
    return CAR_ERROR_STATE;
}

static CarStatus CarRoute_UpdateTrack(CarRouteExecutor *executor,
                                      const CarRouteSegment *segment,
                                      const CarConfig *config,
                                      uint32_t now_ms,
                                      const CarOdometry *odometry,
                                      const CarLineEstimate *line,
                                      CarMotorCommand *motor,
                                      uint32_t *faults)
{
    float track_width = config->arc_effective_track_width_mm;
    float center_speed = segment->center_speed_mm_s;
    float desired_left;
    float desired_right;
    float correction_limit;
    float progress;
    bool narrow = CarRoute_IsNarrowTrack(line);

    if (track_width <= 0.0f) {
        track_width = config->track_width_mm;
    }
    progress = CarRoute_Abs(odometry->center_distance_mm -
                            executor->segment_start_distance_mm);
    executor->segment_progress_mm = progress;
    CarRoute_UpdateTrackLock(executor, config, line);

    if (executor->track_phase == CAR_TRACK_PHASE_FINISH_OFFSET) {
        executor->track_line_gap_mm = 0.0f;
        center_speed = config->finish_approach_speed_mm_s;
        if (center_speed <= 0.0f) {
            center_speed = segment->center_speed_mm_s;
        }
    } else if (narrow) {
        executor->track_has_seen_line = true;
        executor->track_last_line_distance_mm = odometry->center_distance_mm;
        executor->track_line_gap_mm = 0.0f;
    } else {
        float reference_mm = executor->track_has_seen_line ?
            executor->track_last_line_distance_mm :
            executor->segment_start_distance_mm;

        executor->track_line_gap_mm = CarRoute_Abs(
            odometry->center_distance_mm - reference_mm);
        if ((config->required_line_search_mm > 0.0f) &&
            (executor->track_line_gap_mm + config->distance_tolerance_mm >=
             config->required_line_search_mm)) {
            return CarRoute_Fail(executor, motor, faults,
                                 CAR_FAULT_LINE_MISSED,
                                 CAR_ROUTE_EXIT_LINE_MISS);
        }
        if ((config->required_line_search_speed_mm_s > 0.0f) &&
            (center_speed > config->required_line_search_speed_mm_s)) {
            center_speed = config->required_line_search_speed_mm_s;
        }
    }

    desired_left = center_speed *
        (1.0f - segment->curvature_inv_mm * track_width * 0.5f);
    desired_right = center_speed *
        (1.0f + segment->curvature_inv_mm * track_width * 0.5f);

    executor->line_correction_mm_s = 0.0f;
    if (narrow && executor->track_locked) {
        correction_limit = (segment->curvature_inv_mm == 0.0f) ?
            config->straight_line_max_correction_mm_s :
            config->arc_line_max_correction_mm_s;
        executor->line_correction_mm_s = CarRoute_Clamp(
            CarRoute_LinePid(executor, config, line->track_position, now_ms),
            correction_limit);
        desired_left += executor->line_correction_mm_s;
        desired_right -= executor->line_correction_mm_s;
    } else {
        executor->line_integral = 0.0f;
        executor->line_previous_error = 0.0f;
        executor->line_previous_ms = now_ms;
    }

    desired_left = CarRoute_Clamp(desired_left,
                                  config->max_wheel_speed_mm_s);
    desired_right = CarRoute_Clamp(desired_right,
                                   config->max_wheel_speed_mm_s);
    if (!executor->track_command_valid) {
        executor->track_last_left_mm_s = 0.0f;
        executor->track_last_right_mm_s = 0.0f;
        executor->track_last_command_ms = executor->segment_start_ms;
        executor->track_command_valid = true;
    }
    if (config->track_wheel_accel_limit_mm_s2 > 0.0f) {
        float max_delta = config->track_wheel_accel_limit_mm_s2 *
            (float)(now_ms - executor->track_last_command_ms) / 1000.0f;

        desired_left = CarRoute_LimitAcceleration(
            executor->track_last_left_mm_s, desired_left, max_delta);
        desired_right = CarRoute_LimitAcceleration(
            executor->track_last_right_mm_s, desired_right, max_delta);
    }
    executor->track_last_left_mm_s = desired_left;
    executor->track_last_right_mm_s = desired_right;
    executor->track_last_command_ms = now_ms;
    executor->track_center_speed_mm_s = center_speed;
    executor->track_curvature_inv_mm = segment->curvature_inv_mm;
    motor->left_mm_s = desired_left;
    motor->right_mm_s = desired_right;
    motor->enable = true;

    if (executor->track_phase == CAR_TRACK_PHASE_FINISH_OFFSET) {
        executor->finish_offset_progress_mm = CarRoute_Abs(
            progress - executor->finish_trigger_progress_mm);
        if (executor->finish_offset_progress_mm +
            config->distance_tolerance_mm >=
            config->finish_sensor_to_test_point_mm) {
            CarRoute_Stop(motor);
            executor->track_command_valid = false;
            CarRoute_Advance(executor, now_ms, odometry,
                             CAR_ROUTE_EXIT_FINISH_MARKER);
        }
        return CAR_OK;
    }

    if (segment->track_exit == CAR_TRACK_EXIT_FINISH_MARKER) {
        float expected_mm = CarRoute_Abs(segment->distance_mm) -
                            config->finish_sensor_to_test_point_mm;
        float latest_mm = expected_mm + config->required_line_search_mm;
        float earliest_mm = CarRoute_Abs(segment->distance_mm) *
                            CarRoute_Clamp(segment->marker_arm_ratio, 1.0f);
        uint8_t required_frames =
            (config->gray_finish_consecutive_frames == 0U) ? 1U :
            config->gray_finish_consecutive_frames;

        if (executor->track_locked && (progress >= earliest_mm) &&
            ((progress <= latest_mm) || (executor->marker_streak > 0U))) {
            if (CarRoute_IsFinishMarker(config, line)) {
                if (executor->marker_streak < UINT8_MAX) {
                    executor->marker_streak++;
                }
                if (executor->marker_streak >= required_frames) {
                    executor->last_marker_confidence = line->confidence;
                    executor->last_marker_active_count = line->active_count;
                    executor->last_marker_active_mask = line->active_mask;
                    executor->finish_trigger_progress_mm = progress;
                    executor->finish_offset_progress_mm = 0.0f;
                    executor->track_phase =
                        CAR_TRACK_PHASE_FINISH_OFFSET;
                    executor->marker_streak = 0U;
                }
            } else {
                executor->marker_streak = 0U;
            }
        } else if (progress < earliest_mm) {
            executor->marker_streak = 0U;
        }

        if (executor->track_phase == CAR_TRACK_PHASE_FINISH_OFFSET) {
            return CAR_OK;
        }
        if ((progress >= latest_mm) && (executor->marker_streak == 0U)) {
            return CarRoute_Fail(executor, motor, faults,
                                 CAR_FAULT_LINE_MISSED,
                                 CAR_ROUTE_EXIT_LINE_MISS);
        }
    } else if (progress + config->distance_tolerance_mm >=
               CarRoute_Abs(segment->distance_mm)) {
        CarRoute_Advance(executor, now_ms, odometry,
                         CAR_ROUTE_EXIT_DISTANCE);
    }
    return CAR_OK;
}

void CarRouteExecutor_Init(CarRouteExecutor *executor)
{
    if (executor != 0) {
        *executor = (CarRouteExecutor){0};
    }
}

CarStatus CarRouteExecutor_Start(CarRouteExecutor *executor,
                                 const CarRoute *route,
                                 uint32_t now_ms,
                                 const CarOdometry *odometry)
{
    if ((executor == 0) || (route == 0) || (odometry == 0) ||
        (route->count == 0U) || (route->count > CAR_ROUTE_MAX_SEGMENTS)) {
        return CAR_ERROR_ARG;
    }
    *executor = (CarRouteExecutor){0};
    executor->route = route;
    executor->segment_start_ms = now_ms;
    executor->segment_start_distance_mm = odometry->center_distance_mm;
    executor->track_last_command_ms = now_ms;
    executor->track_last_line_distance_mm = odometry->center_distance_mm;
    executor->line_previous_ms = now_ms;
    executor->running = true;
    return CAR_OK;
}

CarStatus CarRouteExecutor_Update(CarRouteExecutor *executor,
                                  const CarConfig *config,
                                  uint32_t now_ms,
                                  const CarOdometry *odometry,
                                  const CarLineEstimate *line,
                                  CarMotorCommand *motor,
                                  CarCue *cue,
                                  uint32_t *faults)
{
    const CarRouteSegment *segment;

    if ((executor == 0) || (config == 0) || (odometry == 0) ||
        (motor == 0) || (cue == 0) || (faults == 0)) {
        return CAR_ERROR_ARG;
    }
    CarRoute_Stop(motor);
    *cue = CAR_CUE_NONE;
    if (!executor->running) {
        return CAR_OK;
    }
    if ((executor->route == 0) ||
        (executor->index >= executor->route->count)) {
        return CarRoute_Fail(executor, motor, faults,
                             CAR_FAULT_ROUTE_INVALID,
                             CAR_ROUTE_EXIT_NONE);
    }
    segment = &executor->route->segments[executor->index];
    if ((segment->type == CAR_SEGMENT_TRACK) &&
        (segment->timeout_ms > 0U) &&
        ((uint32_t)(now_ms - executor->segment_start_ms) >
         segment->timeout_ms)) {
        return CarRoute_Fail(executor, motor, faults,
                             CAR_FAULT_SEGMENT_TIMEOUT,
                             CAR_ROUTE_EXIT_TIMEOUT);
    }

    switch (segment->type) {
        case CAR_SEGMENT_TRACK:
            if (line == 0) {
                return CAR_ERROR_ARG;
            }
            return CarRoute_UpdateTrack(executor, segment, config, now_ms,
                                        odometry, line, motor, faults);
        case CAR_SEGMENT_CUE:
            *cue = segment->cue;
            CarRoute_Advance(executor, now_ms, odometry,
                             CAR_ROUTE_EXIT_NONE);
            return CAR_OK;
        case CAR_SEGMENT_STOP:
            *cue = segment->cue;
            CarRoute_Advance(executor, now_ms, odometry,
                             CAR_ROUTE_EXIT_NONE);
            return CAR_OK;
        default:
            return CarRoute_Fail(executor, motor, faults,
                                 CAR_FAULT_ROUTE_INVALID,
                                 CAR_ROUTE_EXIT_NONE);
    }
}

bool CarRouteExecutor_GrayRequired(const CarRouteExecutor *executor)
{
    if ((executor == 0) || !executor->running ||
        (executor->route == 0) ||
        (executor->index >= executor->route->count)) {
        return false;
    }
    return executor->route->segments[executor->index].use_line;
}
