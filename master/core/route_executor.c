#include "core/route_executor.h"

static float CarRoute_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}
//abs 绝对值
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

    if ((max_delta <= 0.0f) || (CarRoute_Abs(delta) <= max_delta)) {
        return target;
    }
    return current + ((delta > 0.0f) ? max_delta : -max_delta);
}
//clamp 限幅

static float CarRoute_ConfiguredLimit(float configured_limit,
                                      float legacy_limit)
{
    return (configured_limit > 0.0f) ? configured_limit : legacy_limit;
}

static void CarRoute_Stop(CarMotorCommand *motor)
{
    *motor = (CarMotorCommand){0.0f, 0.0f, false};
}
//stop 停车

static bool CarRoute_HasAdjacentActive(uint8_t active_mask)
{
    return (active_mask & (uint8_t)(active_mask >> 1U)) != 0U;
}

static bool CarRoute_IsNarrowTrack(const CarConfig *config,
                                   const CarLineEstimate *line)
{
    if (line == 0) {
        return false;
    }
    if (!config->gray_shape_filter_enable) {
        return line->valid;
    }
    return line->pattern == CAR_LINE_PATTERN_NARROW_TRACK;
}

static bool CarRoute_HasSmallLegacyFootprint(const CarConfig *config,
                                             const CarLineEstimate *line)
{
    if ((line == 0) || !line->valid || (line->active_count == 0U)) {
        return false;
    }
    return (config->gray_track_max_active == 0U) ||
           (line->active_count <= config->gray_track_max_active);
}

static int16_t CarRoute_TrackPosition(const CarConfig *config,
                                      const CarLineEstimate *line)
{
    if (config->gray_shape_filter_enable &&
        (line->track_active_count > 0U)) {
        return line->track_position;
    }
    return line->position;
}

static CarLinePattern CarRoute_LinePattern(const CarConfig *config,
                                           const CarLineEstimate *line)
{
    if (line == 0) {
        return CAR_LINE_PATTERN_NONE;
    }
    if (!config->gray_shape_filter_enable) {
        return line->valid ? CAR_LINE_PATTERN_NARROW_TRACK :
                             CAR_LINE_PATTERN_NONE;
    }
    return line->pattern;
}

static bool CarRoute_HasDirectionalTrackEvidence(
    const CarConfig *config,
    const CarLineEstimate *line)
{
    if (line == 0) {
        return false;
    }
    if (!config->gray_shape_filter_enable) {
        return line->valid;
    }
    if (line->track_active_count > 0U) {
        return (line->pattern != CAR_LINE_PATTERN_NONE) &&
               (line->pattern != CAR_LINE_PATTERN_WIDE_AREA);
    }
    return CarRoute_HasSmallLegacyFootprint(config, line);
}

static bool CarRoute_HasAnyLineEvidence(const CarConfig *config,
                                        const CarLineEstimate *line)
{
    if (line == 0) {
        return false;
    }
    if (!config->gray_shape_filter_enable) {
        return line->valid;
    }

    /* The diagonal has already cleared its start-line distance guard. At that
     * point one adaptive probe or one legacy-valid black footprint is more
     * useful than a shape label that can change while the line sweeps across. */
    if (line->track_active_count > 0U) {
        return (config->gray_track_max_active == 0U) ||
               (line->track_active_count <= config->gray_track_max_active);
    }
    return CarRoute_HasSmallLegacyFootprint(config, line);
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
        (line->active_count >= config->gray_wide_min_active) &&
        (line->active_span == line->active_count) &&
        CarRoute_HasAdjacentActive(line->active_mask);
    return (CarRoute_LinePattern(config, line) ==
            CAR_LINE_PATTERN_WIDE_AREA) || coherent_wide;
}

static void CarRoute_UpdateTrackLock(CarRouteExecutor *executor,
                                     const CarConfig *config,
                                     const CarLineEstimate *line)
{
    CarLinePattern pattern = CarRoute_LinePattern(config, line);
    uint8_t enter_frames = (config->gray_track_enter_frames == 0U) ? 1U :
                           config->gray_track_enter_frames;
    uint8_t lost_frames = (config->gray_track_lost_frames == 0U) ? 1U :
                          config->gray_track_lost_frames;

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
        /* WIDE/SPLIT is ambiguous: it is neither centered track nor proof
         * that the track disappeared. A following narrow/NONE frame decides. */
        executor->track_enter_streak = 0U;
        executor->track_lost_streak = 0U;
    }
}

static bool CarRoute_TimeBefore(uint32_t now_ms, uint32_t deadline_ms)
{
    return (int32_t)(deadline_ms - now_ms) > 0;
}

static float CarRoute_SettledSpeed(const CarRouteExecutor *executor,
                                   const CarConfig *config,
                                   uint32_t now_ms,
                                   float requested_speed)
{
    if ((config->turn_settle_speed_mm_s > 0.0f) &&
        CarRoute_TimeBefore(now_ms, executor->post_turn_settle_until_ms) &&
        (CarRoute_Abs(requested_speed) > config->turn_settle_speed_mm_s)) {
        return (requested_speed < 0.0f) ? -config->turn_settle_speed_mm_s :
                                         config->turn_settle_speed_mm_s;
    }
    return requested_speed;
}

static CarStatus CarRoute_FailLineMiss(CarRouteExecutor *executor,
                                       CarMotorCommand *motor,
                                       uint32_t *faults)
{
    *faults |= CAR_FAULT_LINE_MISSED;
    executor->last_motion_exit_reason = CAR_ROUTE_EXIT_LINE_MISS;
    executor->semantic_line_miss_count++;
    executor->running = false;
    CarRoute_Stop(motor);
    return CAR_ERROR_STATE;
}

static void CarRoute_ResetSegmentState(CarRouteExecutor *executor,
                                       uint32_t now_ms,
                                       float yaw_deg)
{
    executor->line_integral = 0.0f;
    executor->line_previous_error = 0.0f;
    executor->line_previous_ms = now_ms;
    executor->finish_line_streak = 0U;
    executor->line_event_streak = 0U;
    executor->turn_line_reacquire_streak = 0U;
    executor->track_enter_streak = 0U;
    executor->track_lost_streak = 0U;
    executor->track_locked = false;
    executor->track_line_gap_mm = 0.0f;
    executor->track_has_seen_line = false;
    executor->line_follow_seen = false;
    executor->line_corner_candidate = false;
    executor->line_follow_phase = CAR_LINE_FOLLOW_PHASE_TRACK;
    executor->line_corner_trigger_progress_mm = 0.0f;
    executor->line_corner_trigger_yaw_deg = yaw_deg;
    executor->line_corner_approach_mm = 0.0f;
    executor->line_search_start_progress_mm = 0.0f;
    executor->line_search_budget_active = false;
    executor->line_capture_previous_position = 0;
    executor->line_capture_position_valid = false;
    executor->line_capture_guide_mm_s = 0.0f;
    executor->line_seek_correction_mm_s = 0.0f;
    executor->line_heading_reference_yaw_deg = yaw_deg;
    executor->line_heading_reference_valid = false;
    executor->arc_line_locked = false;
    executor->arc_line_entry_streak = 0U;
    executor->arc_line_lock_ms = now_ms;
    executor->arc_line_blend_permille = 0U;
}

void CarRouteExecutor_ResetSegmentState(CarRouteExecutor *executor,
                                        uint32_t now_ms,
                                        float yaw_deg)
{
    if (executor != 0) {
        CarRoute_ResetSegmentState(executor, now_ms, yaw_deg);
    }
}

static float CarRoute_LinePid(CarRouteExecutor *executor,
                              const CarConfig *config,
                              int16_t position,
                              uint32_t now_ms,
                              float kp,
                              float ki,
                              float kd,
                              float integral_limit)
{
    float line_error;
    float dt_s;
    float derivative = 0.0f;
    float correction;

    if ((executor == 0) || (config == 0)) {
        return 0.0f;
    }

    line_error = (float)position;
    dt_s = (float)(now_ms - executor->line_previous_ms) / 1000.0f;
    if ((dt_s > 0.0f) && (dt_s <= 0.25f)) {
        derivative = (line_error - executor->line_previous_error) / dt_s;
        executor->line_integral += line_error * dt_s;
    } else {
        dt_s = 0.01f;
    }
    if (integral_limit > 0.0f) {
        executor->line_integral = CarRoute_Clamp(
            executor->line_integral, integral_limit);
    } else {
        executor->line_integral = 0.0f;
    }
    correction = line_error * kp +
                 executor->line_integral * ki +
                 derivative * kd;
    executor->line_previous_error = line_error;
    executor->line_previous_ms = now_ms;
    return correction;
}

static bool CarRoute_CaptureHasCenter(CarRouteExecutor *executor,
                                      const CarConfig *config,
                                      const CarLineEstimate *line)
{
    int16_t position;
    bool center_reached;
    bool sign_crossed = false;

    if ((executor == 0) || (config == 0) || (line == 0)) {
        return false;
    }
    if (config->gray_shape_filter_enable) {
        /* An armed diagonal may start capture from a single probe. Once
         * guarded, any non-empty adaptive footprint may carry the centroid
         * across the array even if its shape label changes between frames. */
        if (line->track_active_count > 0U) {
            position = line->track_position;
        } else {
            if (!CarRoute_HasSmallLegacyFootprint(config, line)) {
                return false;
            }
            position = line->position;
        }
    } else {
        if (!line->valid) {
            return false;
        }
        position = line->position;
    }

    center_reached = CarRoute_Abs((float)position) <=
        (float)config->line_cross_center_position;
    if (executor->line_capture_position_valid) {
        sign_crossed =
            ((executor->line_capture_previous_position < 0) &&
             (position >= 0)) ||
            ((executor->line_capture_previous_position > 0) &&
             (position <= 0));
    }
    executor->line_capture_previous_position = position;
    executor->line_capture_position_valid = true;
    return center_reached || sign_crossed;
}

static void CarRoute_BeginLineSearch(CarRouteExecutor *executor,
                                     float origin_progress_mm)
{
    if (!executor->line_search_budget_active) {
        executor->line_search_start_progress_mm = origin_progress_mm;
        executor->line_search_budget_active = true;
    }
    executor->line_follow_phase = CAR_LINE_FOLLOW_PHASE_SEARCH;
}

static CarStatus CarRoute_ContinueLineSearch(
    CarRouteExecutor *executor,
    const CarConfig *config,
    float progress_mm,
    float fallback_speed_mm_s,
    float *base_speed_mm_s,
    CarMotorCommand *motor,
    uint32_t *faults)
{
    if (CarRoute_Abs(progress_mm -
                     executor->line_search_start_progress_mm) +
        config->distance_tolerance_mm >= config->required_line_search_mm) {
        return CarRoute_FailLineMiss(executor, motor, faults);
    }
    *base_speed_mm_s = config->required_line_search_speed_mm_s;
    if (*base_speed_mm_s <= 0.0f) {
        *base_speed_mm_s = fallback_speed_mm_s;
    }
    return CAR_OK;
}

static void CarRoute_SetCrossCaptureMotor(
    CarRouteExecutor *executor,
    const CarConfig *config,
    const CarLineEstimate *line,
    bool directional_track,
    float base_speed_mm_s,
    float target_yaw_deg,
    float yaw_deg,
    CarMotorCommand *motor)
{
    float yaw_error = target_yaw_deg - yaw_deg;
    float yaw_correction = CarRoute_Clamp(
        yaw_error * config->straight_heading_kp,
        CarRoute_ConfiguredLimit(
            config->straight_heading_max_correction_mm_s,
            config->max_wheel_speed_mm_s * 0.25f));

    executor->line_capture_guide_mm_s = 0.0f;
    if (config->gray_shape_filter_enable &&
        (config->line_cross_capture_max_correction_mm_s > 0.0f) &&
        directional_track && (line != 0) && (base_speed_mm_s > 0.0f)) {
        float guide_limit =
            config->line_cross_capture_max_correction_mm_s;
        float forward_margin =
            base_speed_mm_s - CarRoute_Abs(yaw_correction);

        if (forward_margin < 0.0f) {
            forward_margin = 0.0f;
        }
        if (guide_limit > forward_margin) {
            guide_limit = forward_margin;
        }
        executor->line_capture_guide_mm_s = CarRoute_Clamp(
            (float)CarRoute_TrackPosition(config, line) *
                config->line_cross_capture_kp,
            guide_limit);
        motor->left_mm_s = base_speed_mm_s - yaw_correction +
                           executor->line_capture_guide_mm_s;
        motor->right_mm_s = base_speed_mm_s + yaw_correction -
                            executor->line_capture_guide_mm_s;
    } else {
        /* Keep these exact legacy assignments in the disabled branch; doing
         * an unconditional +0/-0 would weaken bitwise compatibility checks. */
        motor->left_mm_s = base_speed_mm_s - yaw_correction;
        motor->right_mm_s = base_speed_mm_s + yaw_correction;
    }
    motor->enable = true;
}

static void CarRoute_Advance(CarRouteExecutor *executor,
                             uint32_t now_ms,
                             const CarOdometry *odometry,
                             float yaw_deg,
                             CarRouteExitReason exit_reason)
{
    bool continuous_track = false;
    bool track_locked = executor->track_locked;
    uint8_t track_enter_streak = executor->track_enter_streak;
    float line_integral = executor->line_integral;
    float line_previous_error = executor->line_previous_error;
    uint32_t line_previous_ms = executor->line_previous_ms;
    float track_last_line_distance_mm =
        executor->track_last_line_distance_mm;
    float track_line_gap_mm = executor->track_line_gap_mm;
    bool track_has_seen_line = executor->track_has_seen_line;

    if (exit_reason != CAR_ROUTE_EXIT_NONE) {
        executor->last_motion_exit_reason = exit_reason;
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
    executor->segment_start_yaw_deg = yaw_deg;
    CarRoute_ResetSegmentState(executor, now_ms, yaw_deg);
    executor->track_last_line_distance_mm = odometry->center_distance_mm;
    if (continuous_track) {
        executor->track_locked = track_locked;
        executor->track_enter_streak = track_enter_streak;
        executor->line_integral = line_integral;
        executor->line_previous_error = line_previous_error;
        executor->line_previous_ms = line_previous_ms;
        executor->track_last_line_distance_mm =
            track_last_line_distance_mm;
        executor->track_line_gap_mm = track_line_gap_mm;
        executor->track_has_seen_line = track_has_seen_line;
    }
    if (executor->index >= executor->route->count) {
        executor->running = false;
        executor->finished = true;
    }
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
                                 const CarOdometry *odometry,
                                 float yaw_deg)
{
    if ((executor == 0) || (route == 0) || (odometry == 0) ||
        (route->count == 0U) || (route->count > CAR_ROUTE_MAX_SEGMENTS)) {
        return CAR_ERROR_ARG;
    }

    executor->route = route;
    executor->index = 0U;
    executor->segment_start_ms = now_ms;
    executor->segment_start_distance_mm = odometry->center_distance_mm;
    executor->segment_start_yaw_deg = yaw_deg;
    CarRoute_ResetSegmentState(executor, now_ms, yaw_deg);
    executor->post_turn_settle_until_ms = 0U;
    executor->semantic_line_miss_count = 0U;
    executor->pending_straight_yaw_deg = 0.0f;
    executor->pending_straight_yaw_valid = false;
    executor->track_last_left_mm_s = 0.0f;
    executor->track_last_right_mm_s = 0.0f;
    executor->track_last_command_ms = now_ms;
    executor->track_center_speed_mm_s = 0.0f;
    executor->track_curvature_inv_mm = 0.0f;
    executor->track_last_line_distance_mm = odometry->center_distance_mm;
    executor->track_line_gap_mm = 0.0f;
    executor->track_command_valid = false;
    executor->track_has_seen_line = false;
    executor->last_line_exit_confidence = 0U;
    executor->last_line_exit_active_count = 0U;
    executor->last_line_exit_active_mask = 0U;
    executor->last_line_exit_progress_mm = 0.0f;
    executor->last_motion_exit_reason = CAR_ROUTE_EXIT_NONE;
    executor->running = true;
    executor->finished = false;
    return CAR_OK;
}

CarStatus CarRouteExecutor_Update(CarRouteExecutor *executor,
                                  const CarConfig *config,
                                  uint32_t now_ms,
                                  const CarOdometry *odometry,
                                  float yaw_deg,
                                  const CarLineEstimate *line,
                                  CarMotorCommand *motor,
                                  CarCue *cue,
                                  uint32_t *faults)
{
    const CarRouteSegment *segment;
    float progress;
    float error;
    float correction;

    if ((executor == 0) || (config == 0) || (odometry == 0) ||
        (motor == 0) || (cue == 0) || (faults == 0)) {
        return CAR_ERROR_ARG;
    }

    *cue = CAR_CUE_NONE;
    CarRoute_Stop(motor);
    //默认停车
    if (!executor->running) {
        return CAR_OK;
    }
    //如果没有跑，直接返回
    //后续根据状态去覆盖
    if ((executor->route == 0) || (executor->index >= executor->route->count)) {
        *faults |= CAR_FAULT_ROUTE_INVALID;
        executor->running = false;
        return CAR_ERROR_STATE;
    }

    segment = &executor->route->segments[executor->index];
    if ((segment->timeout_ms > 0U) &&
        ((uint32_t)(now_ms - executor->segment_start_ms) > segment->timeout_ms)) {
        *faults |= CAR_FAULT_SEGMENT_TIMEOUT;
        executor->running = false;
        return CAR_ERROR_STATE;
    }

    CarRoute_UpdateTrackLock(executor, config, line);

    switch (segment->type) {
        case CAR_SEGMENT_STRAIGHT:
        {
            bool required_handoff =
                (segment->line_follow_exit ==
                 CAR_LINE_FOLLOW_EXIT_SENSOR_AXLE) ||
                (segment->line_follow_exit ==
                 CAR_LINE_FOLLOW_EXIT_DIAGONAL_AXLE);
            bool diagonal_handoff =
                segment->line_follow_exit ==
                CAR_LINE_FOLLOW_EXIT_DIAGONAL_AXLE;
            bool permissive_handoff = config->gray_shape_filter_enable &&
                                      diagonal_handoff;
            bool capture_timed_out = false;
            bool narrow_track = CarRoute_IsNarrowTrack(config, line);
            bool directional_track =
                CarRoute_HasDirectionalTrackEvidence(config, line);
            bool handoff_track = permissive_handoff ?
                CarRoute_HasAnyLineEvidence(config, line) :
                directional_track;
            float handoff_arm_ratio = config->gray_finish_arm_ratio;
            float handoff_distance_mm =
                diagonal_handoff ?
                config->diagonal_line_approach_mm :
                config->line_sensor_to_axle_mm;
            float base_speed = segment->speed;

            if (permissive_handoff &&
                (config->diagonal_line_arm_ratio > 0.0f)) {
                handoff_arm_ratio = config->diagonal_line_arm_ratio;
            }

            progress = odometry->center_distance_mm -
                       executor->segment_start_distance_mm;
            /* The axle handoff is authorized only by an observed centroid
             * crossing.  A bounded capture distance is a search budget, not
             * evidence that the marker was centered. */
            if (required_handoff &&
                (executor->line_follow_phase ==
                 CAR_LINE_FOLLOW_PHASE_AXLE_APPROACH)) {
                executor->line_corner_approach_mm = CarRoute_Abs(
                    progress - executor->line_corner_trigger_progress_mm);
                if (executor->line_corner_approach_mm +
                    config->distance_tolerance_mm >= handoff_distance_mm) {
                    executor->pending_straight_yaw_valid = false;
                    CarRoute_Advance(executor, now_ms, odometry, yaw_deg,
                                     CAR_ROUTE_EXIT_LINE);
                    break;
                }
                base_speed = config->line_cross_capture_speed_mm_s;
                if (base_speed <= 0.0f) {
                    base_speed = segment->speed;
                }
                error = executor->line_corner_trigger_yaw_deg - yaw_deg;
                correction = CarRoute_Clamp(
                    error * config->straight_heading_kp,
                    CarRoute_ConfiguredLimit(
                        config->straight_heading_max_correction_mm_s,
                        config->max_wheel_speed_mm_s * 0.25f));
                motor->left_mm_s = base_speed - correction;
                motor->right_mm_s = base_speed + correction;
                motor->enable = true;
                break;
            }
            if (required_handoff &&
                (executor->line_follow_phase ==
                 CAR_LINE_FOLLOW_PHASE_CROSSING_CAPTURE)) {
                float capture_mm = CarRoute_Abs(
                    progress - executor->line_corner_trigger_progress_mm);
                bool capture_window_exhausted =
                    capture_mm >= config->line_cross_capture_window_mm;
                bool capture_search_exhausted =
                    executor->line_search_budget_active &&
                    (CarRoute_Abs(
                        progress - executor->line_search_start_progress_mm) +
                     config->distance_tolerance_mm >=
                     config->required_line_search_mm);
                bool capture_centered = CarRoute_CaptureHasCenter(
                    executor, config, line);

                base_speed = config->line_cross_capture_speed_mm_s;
                if (base_speed <= 0.0f) {
                    base_speed = segment->speed;
                }
                if (capture_centered ||
                    (!config->gray_shape_filter_enable &&
                     capture_window_exhausted)) {
                    executor->line_follow_phase =
                        CAR_LINE_FOLLOW_PHASE_AXLE_APPROACH;
                    executor->line_corner_trigger_progress_mm = progress;
                    executor->line_corner_trigger_yaw_deg = yaw_deg;
                    executor->line_corner_approach_mm = 0.0f;
                    CarRoute_SetCrossCaptureMotor(
                        executor, config, line, false, base_speed,
                        executor->line_corner_trigger_yaw_deg, yaw_deg,
                        motor);
                    break;
                }
                if (config->gray_shape_filter_enable &&
                    (capture_window_exhausted || capture_search_exhausted)) {
                    /* Only an exhausted window without a center crossing is a
                     * miss; a center sample on the boundary remains evidence. */
                    CarRoute_BeginLineSearch(
                        executor,
                        executor->line_corner_trigger_progress_mm);
                    executor->line_capture_guide_mm_s = 0.0f;
                    capture_timed_out = true;
                } else {
                    CarRoute_SetCrossCaptureMotor(
                        executor, config, line, directional_track,
                        base_speed,
                        executor->line_corner_trigger_yaw_deg, yaw_deg,
                        motor);
                    break;
                }
            }
            if (required_handoff && handoff_track && !capture_timed_out &&
                ((executor->line_follow_phase !=
                  CAR_LINE_FOLLOW_PHASE_SEARCH) || directional_track) &&
                (CarRoute_Abs(progress) >=
                 CarRoute_Abs(segment->value) *
                 CarRoute_Clamp(handoff_arm_ratio, 1.0f))) {
                executor->line_follow_phase =
                    CAR_LINE_FOLLOW_PHASE_CROSSING_CAPTURE;
                executor->line_corner_candidate = true;
                executor->line_corner_trigger_progress_mm = progress;
                executor->line_corner_trigger_yaw_deg = yaw_deg;
                executor->line_capture_previous_position =
                    CarRoute_TrackPosition(config, line);
                executor->line_capture_position_valid = true;
                executor->last_line_exit_confidence =
                    line->track_confidence;
                executor->last_line_exit_active_count =
                    line->track_active_count;
                executor->last_line_exit_active_mask =
                    line->track_active_mask;
                executor->last_line_exit_progress_mm = progress;
                base_speed = config->line_cross_capture_speed_mm_s;
                if (base_speed <= 0.0f) {
                    base_speed = segment->speed;
                }
                CarRoute_SetCrossCaptureMotor(
                    executor, config, line, directional_track, base_speed,
                    yaw_deg, yaw_deg, motor);
                break;
            }
            if (required_handoff &&
                ((executor->line_follow_phase ==
                  CAR_LINE_FOLLOW_PHASE_SEARCH) ||
                 (CarRoute_Abs(progress) +
                      config->distance_tolerance_mm >=
                  CarRoute_Abs(segment->value)))) {
                CarStatus search_status;

                if (executor->line_follow_phase !=
                    CAR_LINE_FOLLOW_PHASE_SEARCH) {
                    CarRoute_BeginLineSearch(executor, progress);
                }
                search_status = CarRoute_ContinueLineSearch(
                    executor, config, progress, segment->speed,
                    &base_speed, motor, faults);
                if (search_status != CAR_OK) {
                    return search_status;
                }
            }
            /* 1cm终点细线实测通常只有两个探头同时命中，因此不能要求6/8路。
             * 这里改为“末段里程护栏 + 两个相邻探头 + 足够黑度 + 连续数帧”：
             * 相邻掩码排除两个离散噪点，里程护栏排除途中污点；弧线巡线仍
             * 保留原来的灵敏 line.valid 判定，不受这里影响。
             *
             * 增加“进入末段才布防”的护栏：因为车往往起步就压在上一条线上，
             * 若一进段就判断“线有效+置信度够”会在第 1 帧误退出。所以要求本段
             * 已经走过设定距离的 arm_ratio 后，灰度触发才生效，
             * 确保是在段尾附近真正压到目标横线时才提前退出。
             * 当前实测值0.8表示1000mm段走到800mm后才开始寻找终点线。 */
            if (!required_handoff && segment->use_line &&
                (line != 0) &&
                (config->gray_shape_filter_enable ? narrow_track :
                                                    line->valid) &&
                (line->confidence >= config->gray_finish_min_confidence) &&
                (line->active_count >= config->gray_finish_min_active) &&
                CarRoute_HasAdjacentActive(line->active_mask) &&
                (CarRoute_Abs(progress) >=
                 CarRoute_Abs(segment->value) *
                 CarRoute_Clamp(config->gray_finish_arm_ratio, 1.0f))) {
                uint8_t required_frames =
                    (config->gray_finish_consecutive_frames == 0U) ? 1U :
                    config->gray_finish_consecutive_frames;

                if (executor->finish_line_streak < UINT8_MAX) {
                    executor->finish_line_streak++;
                }
                if (executor->finish_line_streak >= required_frames) {
                    /* OLED refresh is slower than a 1cm marker crossing, so
                     * retain the exact frame that caused the line exit. */
                    executor->last_line_exit_confidence = line->confidence;
                    executor->last_line_exit_active_count = line->active_count;
                    executor->last_line_exit_active_mask = line->active_mask;
                    executor->last_line_exit_progress_mm = progress;
                    executor->pending_straight_yaw_valid = false;
                    CarRoute_Advance(executor, now_ms, odometry, yaw_deg,
                                     CAR_ROUTE_EXIT_LINE);
                    break;
                }
            } else {
                executor->finish_line_streak = 0U;
            }
            /* 改造2：原有的距离退出保留为兜底，防止永远检测不到线而卡死本段
             * （比如线丢失或灰度阵列异常时，跑满距离仍然会正常进入下一段）。 */
            if (!required_handoff &&
                (CarRoute_Abs(progress) + config->distance_tolerance_mm >=
                 CarRoute_Abs(segment->value))) {
                executor->pending_straight_yaw_valid = false;
                CarRoute_Advance(executor, now_ms, odometry, yaw_deg,
                                 CAR_ROUTE_EXIT_DISTANCE);
                break;
            }
            error = (executor->pending_straight_yaw_valid ?
                     executor->pending_straight_yaw_deg :
                     executor->segment_start_yaw_deg) - yaw_deg;
            correction = CarRoute_Clamp(error * config->straight_heading_kp,
                CarRoute_ConfiguredLimit(
                    config->straight_heading_max_correction_mm_s,
                    config->max_wheel_speed_mm_s * 0.4f));
            motor->left_mm_s = base_speed - correction;
            motor->right_mm_s = base_speed + correction;
            motor->enable = true;
            break;
        }
        //直线段行驶逻辑，现在的假设是车头向左偏，yaw增大
        //error为原来0-现在，为负，pid计算correction为负
        //左轮减一个负数就是加速，所以向哪边偏说明哪边慢了，要加速


        case CAR_SEGMENT_LINE_FOLLOW:
        {
            CarLinePattern pattern = CarRoute_LinePattern(config, line);
            bool narrow_frame = CarRoute_IsNarrowTrack(config, line);
            bool line_control_valid = narrow_frame &&
                (!config->gray_shape_filter_enable || executor->track_locked);
            bool line_event = false;
            uint8_t required_frames = 1U;
            float arm_ratio = CarRoute_Clamp(segment->line_event_arm_ratio,
                                             1.0f);
            float fallback_distance_mm = CarRoute_Abs(segment->value);
            float base_speed = CarRoute_SettledSpeed(
                executor, config, now_ms, segment->speed);

            progress = odometry->center_distance_mm -
                       executor->segment_start_distance_mm;
            executor->line_seek_correction_mm_s = 0.0f;

            /* The front gray bar sees a corner before the wheel axle reaches
             * the pivot. Once a corner is confirmed, ignore the outgoing
             * branch and drive the independently tuned pivot-approach distance
             * while holding yaw. Encoder distance is used only for this
             * mechanical handoff, not to decide where the corner begins. */
            if (executor->line_follow_phase ==
                CAR_LINE_FOLLOW_PHASE_AXLE_APPROACH) {
                float approach_speed =
                    config->line_corner_approach_speed_mm_s;

                executor->line_corner_candidate = true;
                executor->line_corner_approach_mm = CarRoute_Abs(
                    progress - executor->line_corner_trigger_progress_mm);
                if (executor->line_corner_approach_mm +
                    config->distance_tolerance_mm >=
                    config->line_corner_pivot_approach_mm) {
                    CarRoute_Advance(executor, now_ms, odometry, yaw_deg,
                                     CAR_ROUTE_EXIT_LINE);
                    break;
                }

                if (approach_speed <= 0.0f) {
                    approach_speed = segment->speed;
                }
                error = executor->line_corner_trigger_yaw_deg - yaw_deg;
                correction = CarRoute_Clamp(
                    error * config->straight_heading_kp,
                    CarRoute_ConfiguredLimit(
                        config->straight_heading_max_correction_mm_s,
                        config->max_wheel_speed_mm_s * 0.25f));
                motor->left_mm_s = approach_speed - correction;
                motor->right_mm_s = approach_speed + correction;
                motor->enable = true;
                break;
            }

            executor->line_corner_candidate = false;
            executor->line_corner_approach_mm = 0.0f;
            if (line_control_valid) {
                executor->line_follow_seen = true;
                executor->line_heading_reference_yaw_deg = yaw_deg;
                executor->line_heading_reference_valid = true;
            }

            /* A real corner/end is accepted only near the expected end of the
             * current black-line edge.  This prevents an early offset or a
             * short dirty patch from changing route state. */
            if (CarRoute_Abs(progress) >=
                CarRoute_Abs(segment->value) * arm_ratio) {
                bool line_lost = executor->line_follow_seen &&
                    (pattern == CAR_LINE_PATTERN_NONE) &&
                    (!config->gray_shape_filter_enable ||
                     (executor->track_lost_streak >=
                      ((config->gray_track_lost_frames == 0U) ? 1U :
                       config->gray_track_lost_frames)));
                bool right_position = line_control_valid &&
                    (CarRoute_TrackPosition(config, line) >=
                     config->line_corner_min_position);
                bool right_shape;
                bool wide_corner = config->gray_shape_filter_enable &&
                    executor->line_follow_seen &&
                    (pattern == CAR_LINE_PATTERN_WIDE_AREA) &&
                    (line->right_ratio_permille >=
                     config->line_corner_min_right_ratio_permille);

                if (config->gray_shape_filter_enable) {
                    right_shape = wide_corner;
                } else {
                    right_shape = narrow_frame &&
                        (line->right_ratio_permille >=
                         config->line_corner_min_right_ratio_permille) &&
                        (line->active_count >=
                         config->line_corner_min_active) &&
                        (line->active_span >= config->line_corner_min_span);
                }
                executor->line_corner_candidate =
                    right_position || right_shape || line_lost;

                if ((segment->line_follow_exit ==
                     CAR_LINE_FOLLOW_EXIT_RIGHT_CORNER) &&
                    (right_position || right_shape)) {
                    line_event = true;
                    required_frames =
                        (config->line_corner_consecutive_frames == 0U) ? 1U :
                        config->line_corner_consecutive_frames;
                } else if (((segment->line_follow_exit ==
                             CAR_LINE_FOLLOW_EXIT_RIGHT_CORNER) ||
                            (segment->line_follow_exit ==
                             CAR_LINE_FOLLOW_EXIT_LOST)) && line_lost) {
                    line_event = true;
                    /* Adaptive hysteresis already consumed the configured
                     * consecutive NONE frames; do not count them twice. */
                    required_frames = config->gray_shape_filter_enable ? 1U :
                        ((config->line_lost_consecutive_frames == 0U) ? 1U :
                         config->line_lost_consecutive_frames);
                }
            }

            if (line_event) {
                if (executor->line_event_streak < UINT8_MAX) {
                    executor->line_event_streak++;
                }
                if (executor->line_event_streak >= required_frames) {
                    bool has_line_observation =
                        pattern != CAR_LINE_PATTERN_NONE;
                    bool use_track_observation =
                        config->gray_shape_filter_enable &&
                        (pattern == CAR_LINE_PATTERN_NARROW_TRACK);

                    executor->last_line_exit_confidence =
                        has_line_observation ?
                        (use_track_observation ?
                         line->track_confidence : line->confidence) : 0U;
                    executor->last_line_exit_active_count =
                        has_line_observation ?
                        (use_track_observation ?
                         line->track_active_count : line->active_count) : 0U;
                    executor->last_line_exit_active_mask =
                        has_line_observation ?
                        (use_track_observation ?
                         line->track_active_mask : line->active_mask) : 0U;
                    executor->last_line_exit_progress_mm = progress;
                    if ((segment->line_follow_exit ==
                         CAR_LINE_FOLLOW_EXIT_RIGHT_CORNER) &&
                        (config->line_corner_pivot_approach_mm >
                         config->distance_tolerance_mm)) {
                        executor->line_follow_phase =
                            CAR_LINE_FOLLOW_PHASE_AXLE_APPROACH;
                        executor->line_corner_trigger_progress_mm = progress;
                        executor->line_corner_trigger_yaw_deg = yaw_deg;
                        executor->line_corner_approach_mm = 0.0f;
                        executor->line_event_streak = 0U;
                        motor->left_mm_s =
                            config->line_corner_approach_speed_mm_s;
                        motor->right_mm_s =
                            config->line_corner_approach_speed_mm_s;
                        motor->enable = true;
                    } else {
                        CarRoute_Advance(executor, now_ms, odometry, yaw_deg,
                                         CAR_ROUTE_EXIT_LINE);
                    }
                    break;
                }
            } else {
                executor->line_event_streak = 0U;
            }

            /* Continuous official-track straights change curvature at the
             * mapped tangent point. Distance is a normal transition here,
             * not evidence that a required marker was missed. */
            if ((segment->line_follow_exit ==
                 CAR_LINE_FOLLOW_EXIT_DISTANCE) &&
                (CarRoute_Abs(progress) + config->distance_tolerance_mm >=
                 fallback_distance_mm)) {
                CarRoute_Advance(executor, now_ms, odometry, yaw_deg,
                                 CAR_ROUTE_EXIT_DISTANCE);
                break;
            }

            if (config->gray_shape_filter_enable) {
                /* Map distance only opens a bounded low-speed search. It must
                 * never authorize a blind turn on a required-line segment. */
                if ((executor->line_follow_phase ==
                     CAR_LINE_FOLLOW_PHASE_SEARCH) ||
                    (CarRoute_Abs(progress) +
                         config->distance_tolerance_mm >=
                     fallback_distance_mm)) {
                    CarStatus search_status;

                    if (executor->line_follow_phase !=
                        CAR_LINE_FOLLOW_PHASE_SEARCH) {
                        CarRoute_BeginLineSearch(executor, progress);
                    }
                    search_status = CarRoute_ContinueLineSearch(
                        executor, config, progress, segment->speed,
                        &base_speed, motor, faults);
                    if (search_status != CAR_OK) {
                        return search_status;
                    }
                }
            } else {
                /* Legacy H2024-compatible odometry fallback. */
                if (segment->line_follow_exit ==
                    CAR_LINE_FOLLOW_EXIT_RIGHT_CORNER) {
                    fallback_distance_mm +=
                        config->line_corner_pivot_approach_mm;
                }
                if (CarRoute_Abs(progress) +
                    config->distance_tolerance_mm >= fallback_distance_mm) {
                    CarRoute_Advance(executor, now_ms, odometry, yaw_deg,
                                     CAR_ROUTE_EXIT_DISTANCE);
                    break;
                }
            }

            /* During a brief NONE gap, hold the last reliable line heading;
             * while a narrow track is visible, gray PID is the primary loop. */
            error = (executor->line_heading_reference_valid ?
                     executor->line_heading_reference_yaw_deg :
                     executor->segment_start_yaw_deg) - yaw_deg;
            correction = CarRoute_Clamp(error * config->straight_heading_kp,
                CarRoute_ConfiguredLimit(
                    config->straight_heading_max_correction_mm_s,
                    config->max_wheel_speed_mm_s * 0.4f));
            motor->left_mm_s = base_speed - correction;
            motor->right_mm_s = base_speed + correction;

            if (line_control_valid) {
                correction = CarRoute_LinePid(
                    executor, config,
                    CarRoute_TrackPosition(config, line), now_ms,
                    config->line_follow_kp, config->line_follow_ki,
                    config->line_follow_kd,
                    config->line_follow_integral_limit);
                if (!config->gray_shape_filter_enable &&
                    (config->line_heading_max_correction_mm_s > 0.0f)) {
                    float yaw_correction = CarRoute_Clamp(
                        (yaw_deg - executor->segment_start_yaw_deg) *
                            config->line_heading_kp,
                        config->line_heading_max_correction_mm_s);
                    correction += yaw_correction;
                }
                correction = CarRoute_Clamp(
                    correction, config->max_wheel_speed_mm_s * 0.25f);
                motor->left_mm_s = base_speed + correction;
                motor->right_mm_s = base_speed - correction;
            } else if (config->gray_shape_filter_enable &&
                       (config->line_seek_max_correction_mm_s > 0.0f) &&
                       CarRoute_HasDirectionalTrackEvidence(config, line)) {
                float seek_limit =
                    config->line_seek_max_correction_mm_s;
                float forward_margin =
                    (motor->left_mm_s < motor->right_mm_s) ?
                    motor->left_mm_s : motor->right_mm_s;

                if (forward_margin < 0.0f) {
                    forward_margin = 0.0f;
                }
                if (seek_limit > forward_margin) {
                    seek_limit = forward_margin;
                }
                correction = CarRoute_LinePid(
                    executor, config, line->track_position, now_ms,
                    config->line_follow_kp, config->line_follow_ki,
                    config->line_follow_kd,
                    config->line_follow_integral_limit);
                executor->line_seek_correction_mm_s =
                    CarRoute_Clamp(correction, seek_limit);
                motor->left_mm_s +=
                    executor->line_seek_correction_mm_s;
                motor->right_mm_s -=
                    executor->line_seek_correction_mm_s;
            } else {
                executor->line_integral = 0.0f;
                executor->line_previous_error = 0.0f;
                executor->line_previous_ms = now_ms;
            }
            motor->enable = true;
            break;
        }

        case CAR_SEGMENT_TURN:
        {
            float target_yaw = executor->segment_start_yaw_deg + segment->value;
            float turn_direction = (segment->value >= 0.0f) ? 1.0f : -1.0f;//控制方向，我们现在的逻辑是左转为正，右转为负
            float turn_error = (target_yaw - yaw_deg) * turn_direction;
            float turned_angle = CarRoute_Abs(
                yaw_deg - executor->segment_start_yaw_deg);
            float turn_speed;
            float inner_ratio = config->turn_inner_speed_ratio;

            if (inner_ratio < 0.0f) {
                inner_ratio = 0.0f;
            } else if (inner_ratio > 0.95f) {
                inner_ratio = 0.95f;
            }

            /* H2026 right angles use yaw for the coarse turn, then allow a
             * centered outgoing line to finish alignment. The angle guard
             * prevents the old line at the corner from ending the turn early. */
            if (segment->use_line &&
                CarRoute_IsNarrowTrack(config, line) &&
                (turned_angle >=
                 config->turn_line_reacquire_min_angle_deg) &&
                (CarRoute_Abs((float)CarRoute_TrackPosition(config, line)) <=
                 (float)config->turn_line_reacquire_max_position)) {
                uint8_t required_frames =
                    (config->turn_line_reacquire_frames == 0U) ? 1U :
                    config->turn_line_reacquire_frames;

                if (executor->turn_line_reacquire_streak < UINT8_MAX) {
                    executor->turn_line_reacquire_streak++;
                }
                if (executor->turn_line_reacquire_streak >= required_frames) {
                    executor->post_turn_settle_until_ms = now_ms +
                        (uint32_t)config->turn_settle_ms;
                    CarRoute_Advance(executor, now_ms, odometry, yaw_deg,
                                     CAR_ROUTE_EXIT_REACQUIRE);
                    break;
                }
            } else {
                executor->turn_line_reacquire_streak = 0U;
            }

            if (turn_error <= config->angle_tolerance_deg) {
                executor->post_turn_settle_until_ms = now_ms +
                    (uint32_t)config->turn_settle_ms;
                CarRoute_Advance(executor, now_ms, odometry, yaw_deg,
                                 CAR_ROUTE_EXIT_ANGLE);
                break;
            }//最小的这个角度误差，就不用动了

            /* Proportional yaw loop with a small floor to overcome stiction. */
            turn_speed = turn_error * config->turn_heading_kp;
            if (turn_speed < config->turn_min_speed_mm_s) {
                turn_speed = config->turn_min_speed_mm_s;
            }//克服静摩擦力加机械死区
            turn_speed = CarRoute_Clamp(
                turn_speed, CarRoute_Abs(segment->speed));
            if (segment->value > 0.0f) {
                motor->left_mm_s = turn_speed * inner_ratio;
                motor->right_mm_s = turn_speed;
            } else {
                motor->left_mm_s = turn_speed;
                motor->right_mm_s = turn_speed * inner_ratio;
            }//左转为正，所以左轮速度为0
            motor->enable = true;
            break;
        }
        //转弯段逻辑：先算目标值，就是起点加要转到的角度
        //error 差的角度
        case CAR_SEGMENT_ARC:
        {
            float radius = CarRoute_Abs(segment->value);
            float direction = (segment->value >= 0.0f) ? 1.0f : -1.0f;
            float arc_distance_progress = odometry->center_distance_mm -
                                          executor->segment_start_distance_mm;
            float effective_track_width =
                (config->arc_effective_track_width_mm > 0.0f) ?
                config->arc_effective_track_width_mm : config->track_width_mm;
            CarLinePattern pattern = CarRoute_LinePattern(config, line);
            bool narrow_frame = CarRoute_IsNarrowTrack(config, line);
            float arc_angle_progress;
            float inner_ratio;
            /* 改造1：退出角度参数化。之前写死 180，现在读 segment->arc_angle_deg。
             * 用局部变量做兜底：万一某个段忘了填（==0），退回旧的 180 度，
             * 保证不会因为 0 度阈值导致刚进弧就立刻退出。 */
            float exit_angle_deg =
                (segment->arc_angle_deg > 0.0f) ? segment->arc_angle_deg : 180.0f;

            progress = yaw_deg - executor->segment_start_yaw_deg;
            arc_angle_progress = CarRoute_Abs(progress);
            if (radius <= effective_track_width * 0.5f) {
                *faults |= CAR_FAULT_ROUTE_INVALID;
                executor->running = false;
                return CAR_ERROR_STATE;
            }

            /* The measured 115 mm wheel-center spacing is still correct for
             * odometry, but the latest three arcs imply a 153-161 mm effective
             * turning width because of tire scrub. Keeping the two values
             * separate corrects ARC curvature without corrupting distance. */
            inner_ratio = (radius - effective_track_width * 0.5f) /
                          (radius + effective_track_width * 0.5f);
            if (direction > 0.0f) {
                motor->left_mm_s = segment->speed * inner_ratio;
                motor->right_mm_s = segment->speed;
            } else {
                motor->left_mm_s = segment->speed;
                motor->right_mm_s = segment->speed * inner_ratio;
            }

            /* The gray bar is in front of the axle. Once the end of the arc
             * has been confirmed, keep the same geometric arc only for the
             * measured sensor-to-axle distance. This removes the observed
             * extra ~430mm loop without turning roughly 150mm too early. */
            if (executor->line_follow_phase ==
                CAR_LINE_FOLLOW_PHASE_AXLE_APPROACH) {
                float approach_distance =
                    (segment->line_follow_exit ==
                     CAR_LINE_FOLLOW_EXIT_WIDE_MARKER) ?
                    config->finish_sensor_to_test_point_mm :
                    config->line_sensor_to_axle_mm;

                executor->line_corner_candidate = true;
                executor->line_corner_approach_mm = CarRoute_Abs(
                    arc_distance_progress -
                    executor->line_corner_trigger_progress_mm);
                if (executor->line_corner_approach_mm +
                    config->distance_tolerance_mm >=
                    approach_distance) {
                    if (segment->handoff_arc_heading) {
                        /* A line-tail exit is earlier than the yaw fallback.
                         * Hold the actual tangent at the axle handoff instead
                         * of commanding the missing angle up to 180deg. */
                        executor->pending_straight_yaw_deg = yaw_deg;
                        executor->pending_straight_yaw_valid = true;
                    }
                    CarRoute_Advance(executor, now_ms, odometry, yaw_deg,
                                     CAR_ROUTE_EXIT_LINE);
                    if (segment->line_follow_exit ==
                        CAR_LINE_FOLLOW_EXIT_WIDE_MARKER) {
                        CarRoute_Stop(motor);
                    }
                    break;
                }
                motor->enable = true;
                break;
            }

            /* Acquire the narrow arc only after leaving the start marker.
             * Two frames plus a short blend prevent the first off-center
             * sample from producing the entry jerk seen on the real car. */
            if (!config->gray_shape_filter_enable) {
                executor->arc_line_locked = narrow_frame;
                executor->arc_line_blend_permille = narrow_frame ? 1000U : 0U;
            } else if ((arc_angle_progress >=
                        config->arc_line_entry_min_angle_deg) &&
                       narrow_frame) {
                uint8_t entry_frames =
                    (config->arc_line_entry_frames == 0U) ? 1U :
                    config->arc_line_entry_frames;

                if (executor->arc_line_entry_streak < UINT8_MAX) {
                    executor->arc_line_entry_streak++;
                }
                if (!executor->arc_line_locked &&
                    (executor->arc_line_entry_streak >= entry_frames)) {
                    executor->arc_line_locked = true;
                    executor->arc_line_lock_ms = now_ms;
                }
            } else if (pattern == CAR_LINE_PATTERN_NONE) {
                executor->arc_line_entry_streak = 0U;
                if (executor->track_lost_streak >=
                    ((config->gray_track_lost_frames == 0U) ? 1U :
                     config->gray_track_lost_frames)) {
                    executor->arc_line_locked = false;
                }
            } else {
                executor->arc_line_entry_streak = 0U;
            }

            if (config->gray_shape_filter_enable) {
                if (executor->arc_line_locked) {
                    uint32_t blend_ms = config->arc_line_blend_ms;
                    uint32_t elapsed_ms = now_ms - executor->arc_line_lock_ms;

                    executor->arc_line_blend_permille =
                        (blend_ms == 0U || elapsed_ms >= blend_ms) ? 1000U :
                        (uint16_t)((elapsed_ms * 1000U) / blend_ms);
                } else {
                    executor->arc_line_blend_permille = 0U;
                }
            }

            if ((segment->line_follow_exit ==
                 CAR_LINE_FOLLOW_EXIT_WIDE_MARKER) &&
                (arc_angle_progress >=
                 exit_angle_deg *
                     CarRoute_Clamp(segment->line_event_arm_ratio, 1.0f))) {
                bool wide_marker =
                    (pattern == CAR_LINE_PATTERN_WIDE_AREA) &&
                    (line->confidence >= config->gray_finish_min_confidence) &&
                    (line->active_count >= config->gray_finish_min_active);
                uint8_t required_frames =
                    (config->gray_finish_consecutive_frames == 0U) ? 1U :
                    config->gray_finish_consecutive_frames;

                if (wide_marker) {
                    if (executor->line_event_streak < UINT8_MAX) {
                        executor->line_event_streak++;
                    }
                    if (executor->line_event_streak >= required_frames) {
                        executor->last_line_exit_confidence = line->confidence;
                        executor->last_line_exit_active_count =
                            line->active_count;
                        executor->last_line_exit_active_mask =
                            line->active_mask;
                        executor->last_line_exit_progress_mm =
                            arc_distance_progress;
                        executor->line_follow_phase =
                            CAR_LINE_FOLLOW_PHASE_AXLE_APPROACH;
                        executor->line_corner_trigger_progress_mm =
                            arc_distance_progress;
                        executor->line_corner_trigger_yaw_deg = yaw_deg;
                        executor->line_corner_approach_mm = 0.0f;
                        executor->line_event_streak = 0U;
                        motor->enable = true;
                        break;
                    }
                } else {
                    executor->line_event_streak = 0U;
                }
            }

            /* H2026 task2 uses disappearance of the black arc as a guarded
             * end event. The feature is disabled by zero-valued defaults, so
             * H2024 ARC segments still exit only by their configured angle. */
            if (segment->use_line &&
                (config->arc_line_exit_min_angle_deg > 0.0f) &&
                (config->arc_line_exit_lost_frames > 0U)) {
                if (!executor->line_follow_seen) {
                    /* Do not arm from the B-point line under the sensor at
                     * segment entry; require a valid arc sample after 110deg. */
                    if ((arc_angle_progress >=
                         config->arc_line_exit_min_angle_deg) &&
                        executor->arc_line_locked && narrow_frame) {
                        executor->line_follow_seen = true;
                    }
                } else if (narrow_frame) {
                    executor->line_event_streak = 0U;
                    executor->line_corner_candidate = false;
                } else if (pattern == CAR_LINE_PATTERN_NONE) {
                    executor->line_corner_candidate = true;
                    if (executor->line_event_streak < UINT8_MAX) {
                        executor->line_event_streak++;
                    }
                    if (executor->line_event_streak >=
                        config->arc_line_exit_lost_frames) {
                        executor->last_line_exit_confidence = 0U;
                        executor->last_line_exit_active_count = 0U;
                        executor->last_line_exit_active_mask = 0U;
                        executor->last_line_exit_progress_mm =
                            arc_distance_progress;
                        executor->line_follow_phase =
                            CAR_LINE_FOLLOW_PHASE_AXLE_APPROACH;
                        executor->line_corner_trigger_progress_mm =
                            arc_distance_progress;
                        executor->line_corner_trigger_yaw_deg = yaw_deg;
                        executor->line_corner_approach_mm = 0.0f;
                        executor->line_event_streak = 0U;
                        motor->enable = true;
                        break;
                    }
                } else {
                    /* WIDE/SPLIT is not proof that the narrow arc ended. */
                    executor->line_event_streak = 0U;
                    executor->line_corner_candidate = false;
                }
            }

            /* Angle is a fallback only. If line-tail logic already entered
             * AXLE_APPROACH above, the next update takes the precedence path
             * and completes the full physical offset even at 179..180deg. */
            if (arc_angle_progress + config->angle_tolerance_deg >=
                exit_angle_deg) {
                if (segment->line_follow_exit ==
                    CAR_LINE_FOLLOW_EXIT_WIDE_MARKER) {
                    return CarRoute_FailLineMiss(executor, motor, faults);
                }
                if (segment->handoff_arc_heading) {
                    executor->pending_straight_yaw_deg =
                        executor->segment_start_yaw_deg +
                        direction * exit_angle_deg;
                    executor->pending_straight_yaw_valid = true;
                }
                CarRoute_Advance(executor, now_ms, odometry, yaw_deg,
                                 CAR_ROUTE_EXIT_ANGLE);
                break;
            }

            if (segment->use_line && executor->arc_line_locked &&
                narrow_frame) {
                correction = CarRoute_LinePid(
                    executor, config,
                    CarRoute_TrackPosition(config, line), now_ms,
                    config->arc_line_kp, config->arc_line_ki,
                    config->arc_line_kd,
                    config->arc_line_integral_limit);
                /* A one-frame edge centroid previously produced +/-80 mm/s
                 * and could reverse the intended inner/outer-wheel ordering.
                 * H2026 applies a smaller independent limit; zero keeps the
                 * original 25% limit for the untouched H2024 behavior. */
                correction = CarRoute_Clamp(correction,
                    CarRoute_ConfiguredLimit(
                        config->arc_line_max_correction_mm_s,
                        config->max_wheel_speed_mm_s * 0.25f));
                correction *=
                    (float)executor->arc_line_blend_permille / 1000.0f;
                /* 改造1：退出前把最后一帧的灰度修正衰减，避免冲过头。
                 * 越接近退出角度，剩余角度 remaining 越小；在最后一个
                 * angle_tolerance 窗口内按比例把修正量线性衰减到 0，
                 * 这样弧段收尾时不会再猛打方向造成过冲/画龙。 */
                {
                    float remaining =
                        exit_angle_deg - CarRoute_Abs(progress);
                    float decay_window = config->angle_tolerance_deg;
                    if (decay_window > 0.0f) {
                        if (remaining < 0.0f) {
                            remaining = 0.0f;
                        }
                        if (remaining < decay_window) {
                            correction *= (remaining / decay_window);
                        }
                    }
                }
                /* Positive line position means the line is on the right.
                 * Increase the left wheel and slow the right wheel to steer
                 * toward it; this matches the TB6612 forward convention. */
                motor->left_mm_s += correction;
                motor->right_mm_s -= correction;
            }
            motor->enable = true;
            break;
        }
        //圆弧段
        case CAR_SEGMENT_TRACK:
        {
            float track_width =
                (config->arc_effective_track_width_mm > 0.0f) ?
                config->arc_effective_track_width_mm : config->track_width_mm;
            float curvature = segment->arc_angle_deg;
            float center_speed = segment->speed;
            float desired_left;
            float desired_right;
            float finish_marker_latest_mm = 0.0f;
            bool narrow_frame = CarRoute_IsNarrowTrack(config, line);
            bool line_control_valid = narrow_frame &&
                (!config->gray_shape_filter_enable || executor->track_locked);

            progress = odometry->center_distance_mm -
                       executor->segment_start_distance_mm;
            if (segment->line_follow_exit ==
                CAR_LINE_FOLLOW_EXIT_WIDE_MARKER) {
                float expected_trigger_mm = CarRoute_Abs(segment->value) -
                    config->finish_sensor_to_test_point_mm;

                finish_marker_latest_mm = expected_trigger_mm +
                    config->required_line_search_mm;
            }

            if (executor->line_follow_phase ==
                CAR_LINE_FOLLOW_PHASE_AXLE_APPROACH) {
                /* The marker is already confirmed. The remaining travel is
                 * purely the measured sensor-to-test-point offset. */
                executor->track_line_gap_mm = 0.0f;
            } else if (narrow_frame) {
                executor->track_has_seen_line = true;
                executor->track_last_line_distance_mm =
                    odometry->center_distance_mm;
                executor->track_line_gap_mm = 0.0f;
            } else {
                float line_reference_mm = executor->track_has_seen_line ?
                    executor->track_last_line_distance_mm :
                    executor->segment_start_distance_mm;

                executor->track_line_gap_mm = CarRoute_Abs(
                    odometry->center_distance_mm - line_reference_mm);
                if ((config->required_line_search_mm > 0.0f) &&
                    (executor->track_line_gap_mm +
                     config->distance_tolerance_mm >=
                     config->required_line_search_mm)) {
                    return CarRoute_FailLineMiss(executor, motor, faults);
                }
            }

            if (executor->line_follow_phase ==
                CAR_LINE_FOLLOW_PHASE_AXLE_APPROACH) {
                center_speed = config->finish_approach_speed_mm_s;
                if (center_speed <= 0.0f) {
                    center_speed = segment->speed;
                }
            } else if (!narrow_frame &&
                       (config->required_line_search_speed_mm_s > 0.0f) &&
                       (center_speed >
                        config->required_line_search_speed_mm_s)) {
                center_speed = config->required_line_search_speed_mm_s;
            }

            desired_left = center_speed *
                (1.0f - curvature * track_width * 0.5f);
            desired_right = center_speed *
                (1.0f + curvature * track_width * 0.5f);

            if (line_control_valid) {
                float correction_limit =
                    (curvature == 0.0f) ?
                    config->max_wheel_speed_mm_s * 0.25f :
                    CarRoute_ConfiguredLimit(
                        config->arc_line_max_correction_mm_s,
                        config->max_wheel_speed_mm_s * 0.25f);

                correction = CarRoute_LinePid(
                    executor, config, CarRoute_TrackPosition(config, line),
                    now_ms, config->line_follow_kp, config->line_follow_ki,
                    config->line_follow_kd,
                    config->line_follow_integral_limit);
                correction = CarRoute_Clamp(correction, correction_limit);
                desired_left += correction;
                desired_right -= correction;
            } else {
                executor->line_integral = 0.0f;
                executor->line_previous_error = 0.0f;
                executor->line_previous_ms = now_ms;
            }

            desired_left = CarRoute_Clamp(
                desired_left, config->max_wheel_speed_mm_s);
            desired_right = CarRoute_Clamp(
                desired_right, config->max_wheel_speed_mm_s);

            if (!executor->track_command_valid) {
                executor->track_last_left_mm_s = 0.0f;
                executor->track_last_right_mm_s = 0.0f;
                executor->track_last_command_ms = executor->segment_start_ms;
                executor->track_command_valid = true;
            }
            if (config->track_wheel_accel_limit_mm_s2 > 0.0f) {
                float max_delta = config->track_wheel_accel_limit_mm_s2 *
                    (float)(now_ms - executor->track_last_command_ms) /
                    1000.0f;

                desired_left = CarRoute_MoveToward(
                    executor->track_last_left_mm_s, desired_left, max_delta);
                desired_right = CarRoute_MoveToward(
                    executor->track_last_right_mm_s, desired_right, max_delta);
            }
            executor->track_last_left_mm_s = desired_left;
            executor->track_last_right_mm_s = desired_right;
            executor->track_last_command_ms = now_ms;
            executor->track_center_speed_mm_s = center_speed;
            executor->track_curvature_inv_mm = curvature;
            motor->left_mm_s = desired_left;
            motor->right_mm_s = desired_right;
            motor->enable = true;

            if (executor->line_follow_phase ==
                CAR_LINE_FOLLOW_PHASE_AXLE_APPROACH) {
                executor->line_corner_approach_mm = CarRoute_Abs(
                    progress - executor->line_corner_trigger_progress_mm);
                if (executor->line_corner_approach_mm +
                    config->distance_tolerance_mm >=
                    config->finish_sensor_to_test_point_mm) {
                    CarRoute_Stop(motor);
                    executor->track_command_valid = false;
                    CarRoute_Advance(executor, now_ms, odometry, yaw_deg,
                                     CAR_ROUTE_EXIT_LINE);
                }
                break;
            }

            if ((segment->line_follow_exit ==
                 CAR_LINE_FOLLOW_EXIT_WIDE_MARKER) &&
                executor->track_locked &&
                (CarRoute_Abs(progress) >=
                  CarRoute_Abs(segment->value) *
                    CarRoute_Clamp(segment->line_event_arm_ratio, 1.0f)) &&
                ((CarRoute_Abs(progress) <= finish_marker_latest_mm) ||
                 (executor->line_event_streak > 0U))) {
                uint8_t required_frames =
                    (config->gray_finish_consecutive_frames == 0U) ? 1U :
                    config->gray_finish_consecutive_frames;

                if (CarRoute_IsFinishMarker(config, line)) {
                    if (executor->line_event_streak < UINT8_MAX) {
                        executor->line_event_streak++;
                    }
                    if (executor->line_event_streak >= required_frames) {
                        executor->last_line_exit_confidence = line->confidence;
                        executor->last_line_exit_active_count =
                            line->active_count;
                        executor->last_line_exit_active_mask =
                            line->active_mask;
                        executor->last_line_exit_progress_mm = progress;
                        executor->line_corner_trigger_progress_mm = progress;
                        executor->line_corner_trigger_yaw_deg = yaw_deg;
                        executor->line_corner_approach_mm = 0.0f;
                        executor->line_follow_phase =
                            CAR_LINE_FOLLOW_PHASE_AXLE_APPROACH;
                        executor->line_event_streak = 0U;
                    }
                } else {
                    executor->line_event_streak = 0U;
                }
            }

            if (executor->line_follow_phase ==
                CAR_LINE_FOLLOW_PHASE_AXLE_APPROACH) {
                break;
            }

            if (segment->line_follow_exit ==
                CAR_LINE_FOLLOW_EXIT_WIDE_MARKER) {
                if (CarRoute_Abs(progress) >= finish_marker_latest_mm) {
                    /* A first marker frame at the search boundary still gets
                     * one control period to satisfy consecutive confirmation.
                     * A following non-marker frame clears the streak and
                     * faults on that same update. */
                    if (executor->line_event_streak == 0U) {
                        return CarRoute_FailLineMiss(executor, motor, faults);
                    }
                    break;
                }
            } else if (CarRoute_Abs(progress) +
                       config->distance_tolerance_mm >=
                       CarRoute_Abs(segment->value)) {
                CarRoute_Advance(executor, now_ms, odometry, yaw_deg,
                                 CAR_ROUTE_EXIT_DISTANCE);
            }
            break;
        }
        case CAR_SEGMENT_CUE:
            *cue = segment->cue;
            CarRoute_Advance(executor, now_ms, odometry, yaw_deg,
                             CAR_ROUTE_EXIT_NONE);
            break;
        //触发声光提示段
        case CAR_SEGMENT_STOP:
            *cue = segment->cue;
            CarRoute_Advance(executor, now_ms, odometry, yaw_deg,
                             CAR_ROUTE_EXIT_NONE);
            break;
        //停止段
        default:
            *faults |= CAR_FAULT_ROUTE_INVALID;
            executor->running = false;
            return CAR_ERROR_STATE;
    }

    motor->left_mm_s = CarRoute_Clamp(motor->left_mm_s,
                                      config->max_wheel_speed_mm_s);
    motor->right_mm_s = CarRoute_Clamp(motor->right_mm_s,
                                       config->max_wheel_speed_mm_s);
    return CAR_OK;
}

bool CarRouteExecutor_GrayRequired(const CarRouteExecutor *executor)
{
    if ((executor == 0) || !executor->running ||
        (executor->route == 0) || (executor->index >= executor->route->count)) {
        return false;
    }
    return executor->route->segments[executor->index].use_line;
}
