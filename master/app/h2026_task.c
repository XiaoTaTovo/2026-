#include "app/h2026_task.h"

static CarStatus H2026_Add(CarRoute *route,
                           CarSegmentType type,
                           float value,
                           float speed,
                           uint32_t timeout_ms,
                           bool use_line,
                           CarCue cue,
                           float arc_angle_deg,
                           CarLineFollowExit line_exit,
                           float line_arm_ratio,
                           bool handoff_arc_heading)
{
    CarRouteSegment *segment;

    if ((route == 0) || (route->count >= CAR_ROUTE_MAX_SEGMENTS)) {
        return CAR_ERROR_CAPACITY;
    }
    segment = &route->segments[route->count++];
    *segment = (CarRouteSegment){
        type,
        value,
        speed,
        timeout_ms,
        use_line,
        cue,
        arc_angle_deg,
        line_exit,
        line_arm_ratio,
        handoff_arc_heading
    };
    return CAR_OK;
}

static float H2026_FinishMarkerArmRatio(const CarConfig *config,
                                        float arc_length_mm)
{
    float expected_trigger_mm;
    float earliest_trigger_mm;

    if ((config == 0) || (arc_length_mm <= 0.0f)) {
        return 1.0f;
    }
    expected_trigger_mm = arc_length_mm -
                          config->finish_sensor_to_test_point_mm;
    earliest_trigger_mm = expected_trigger_mm -
                          config->required_line_search_mm;
    if (earliest_trigger_mm <= 0.0f) {
        return 0.0f;
    }
    if (earliest_trigger_mm >= arc_length_mm) {
        return 1.0f;
    }
    return earliest_trigger_mm / arc_length_mm;
}

static CarStatus H2026_AddCue(CarRoute *route, CarCue cue)
{
    return H2026_Add(route, CAR_SEGMENT_CUE, 0.0f, 0.0f, 0U, false,
                     cue, 0.0f, CAR_LINE_FOLLOW_EXIT_DISTANCE, 0.0f,
                     false);
}

static CarStatus H2026_AddTrack(CarRoute *route,
                                const CarConfig *config,
                                float distance_mm,
                                float curvature_inv_mm,
                                float speed_mm_s,
                                uint32_t timeout_ms,
                                bool finish_at_a)
{
    return H2026_Add(
        route, CAR_SEGMENT_TRACK, distance_mm, speed_mm_s, timeout_ms,
        true, CAR_CUE_NONE, curvature_inv_mm,
        finish_at_a ? CAR_LINE_FOLLOW_EXIT_WIDE_MARKER :
                      CAR_LINE_FOLLOW_EXIT_DISTANCE,
        finish_at_a ?
            H2026_FinishMarkerArmRatio(config, distance_mm) : 0.0f,
        false);
}

static CarStatus H2026_AddStraight(CarRoute *route,
                                   const CarConfig *config)
{
    return H2026_AddTrack(route, config, H2026_TRACK_STRAIGHT_MM, 0.0f,
                          config->straight_speed_mm_s,
                          config->straight_timeout_ms, false);
}

static CarStatus H2026_AddArc(CarRoute *route,
                              const CarConfig *config,
                              float distance_mm,
                              bool finish_at_a)
{
    return H2026_AddTrack(route, config, distance_mm,
                          -1.0f / H2026_TRACK_ARC_RADIUS_MM,
                          config->arc_speed_mm_s,
                          config->arc_timeout_ms, finish_at_a);
}

static CarStatus H2026_BuildFullLap(CarRoute *route,
                                    const CarConfig *config)
{
    float arc_distance_mm =
        3.14159265358979323846f * H2026_TRACK_ARC_RADIUS_MM;

    if ((H2026_AddStraight(route, config) != CAR_OK) ||
        (H2026_AddArc(route, config, arc_distance_mm, false) != CAR_OK) ||
        (H2026_AddStraight(route, config) != CAR_OK) ||
        (H2026_AddArc(route, config, arc_distance_mm, true) != CAR_OK)) {
        return CAR_ERROR_CAPACITY;
    }
    return CAR_OK;
}

bool H2026_ModeIsOfficial(H2024Mode mode)
{
    return (mode >= H2026_MODE_B2) && (mode <= H2026_MODE_B6);
}

bool H2026_ModeUsesLine(H2024Mode mode)
{
    return (mode == H2026_MODE_B2) || (mode == H2026_MODE_B4) ||
           (mode == H2026_MODE_B5) || (mode == H2026_MODE_B6);
}

bool H2026_ModeRequiresBallControl(H2024Mode mode)
{
    return (mode == H2026_MODE_B3) || (mode == H2026_MODE_B4) ||
           (mode == H2026_MODE_B5) || (mode == H2026_MODE_B6);
}

CarStatus H2026_BuildRoute(H2024Mode mode,
                           const CarConfig *config,
                           CarRoute *route)
{
    CarStatus status = CAR_OK;

    if ((config == 0) || (route == 0) || !H2026_ModeIsOfficial(mode)) {
        return CAR_ERROR_ARG;
    }
    if (mode != H2026_MODE_B2) {
        /* These modes need the ball controller and mode-specific timing/exit
         * semantics. Refuse placeholders even if a compile-time gate is
         * accidentally changed. */
        return CAR_ERROR_STATE;
    }
    *route = (CarRoute){0};
    if (H2026_AddCue(route, CAR_CUE_START) != CAR_OK) {
        return CAR_ERROR_CAPACITY;
    }

    switch (mode) {
        case H2026_MODE_B2:
            status = H2026_BuildFullLap(route, config);
            break;
        default:
            return CAR_ERROR_ARG;
    }

    if (status != CAR_OK) {
        return status;
    }
    return H2026_Add(route, CAR_SEGMENT_STOP, 0.0f, 0.0f, 0U, false,
                     CAR_CUE_FINISH, 0.0f,
                     CAR_LINE_FOLLOW_EXIT_DISTANCE, 0.0f, false);
}
