#include "app/h2026_task.h"

static CarStatus H2026_AddSegment(CarRoute *route,
                                  const CarRouteSegment *segment)
{
    if ((route == 0) || (segment == 0) ||
        (route->count >= CAR_ROUTE_MAX_SEGMENTS)) {
        return CAR_ERROR_CAPACITY;
    }
    route->segments[route->count++] = *segment;
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

static CarStatus H2026_AddTrack(CarRoute *route,
                                float distance_mm,
                                float center_speed_mm_s,
                                uint32_t timeout_ms,
                                float curvature_inv_mm,
                                CarTrackExit exit,
                                float marker_arm_ratio)
{
    const CarRouteSegment segment = {
        .type = CAR_SEGMENT_TRACK,
        .distance_mm = distance_mm,
        .center_speed_mm_s = center_speed_mm_s,
        .timeout_ms = timeout_ms,
        .use_line = true,
        .cue = CAR_CUE_NONE,
        .curvature_inv_mm = curvature_inv_mm,
        .track_exit = exit,
        .marker_arm_ratio = marker_arm_ratio
    };

    return H2026_AddSegment(route, &segment);
}

static CarStatus H2026_BuildB2(const CarConfig *config, CarRoute *route)
{
    const float arc_length_mm =
        3.14159265358979323846f * H2026_TRACK_ARC_RADIUS_MM;
    const CarRouteSegment start = {
        .type = CAR_SEGMENT_CUE,
        .cue = CAR_CUE_START
    };
    const CarRouteSegment stop = {
        .type = CAR_SEGMENT_STOP,
        .cue = CAR_CUE_FINISH
    };

    if ((config->finish_sensor_to_test_point_mm <= 0.0f) ||
        (config->finish_sensor_to_test_point_mm >= arc_length_mm) ||
        (config->required_line_search_mm <= 0.0f)) {
        return CAR_ERROR_ARG;
    }
    if ((H2026_AddSegment(route, &start) != CAR_OK) ||
        (H2026_AddTrack(route, H2026_TRACK_STRAIGHT_MM,
                        config->straight_speed_mm_s,
                        config->straight_timeout_ms, 0.0f,
                        CAR_TRACK_EXIT_DISTANCE, 0.0f) != CAR_OK) ||
        (H2026_AddTrack(route, arc_length_mm, config->arc_speed_mm_s,
                        config->arc_timeout_ms,
                        -1.0f / H2026_TRACK_ARC_RADIUS_MM,
                        CAR_TRACK_EXIT_DISTANCE, 0.0f) != CAR_OK) ||
        (H2026_AddTrack(route, H2026_TRACK_STRAIGHT_MM,
                        config->straight_speed_mm_s,
                        config->straight_timeout_ms, 0.0f,
                        CAR_TRACK_EXIT_DISTANCE, 0.0f) != CAR_OK) ||
        (H2026_AddTrack(route, arc_length_mm, config->arc_speed_mm_s,
                        config->arc_timeout_ms,
                        -1.0f / H2026_TRACK_ARC_RADIUS_MM,
                        CAR_TRACK_EXIT_FINISH_MARKER,
                        H2026_FinishMarkerArmRatio(config,
                                                  arc_length_mm)) != CAR_OK) ||
        (H2026_AddSegment(route, &stop) != CAR_OK)) {
        return CAR_ERROR_CAPACITY;
    }
    return CAR_OK;
}

const char *H2026_ModeName(H2026Mode mode)
{
    switch (mode) {
        case H2026_MODE_B2:
            return "B2";
        case H2026_MODE_B3:
            return "B3";
        case H2026_MODE_B4:
            return "B4";
        case H2026_MODE_B5:
            return "B5";
        case H2026_MODE_B6:
            return "B6";
        default:
            return "?";
    }
}

bool H2026_ModeIsOfficial(H2026Mode mode)
{
    return (mode >= H2026_MODE_B2) && (mode <= H2026_MODE_B6);
}

bool H2026_ModeUsesLine(H2026Mode mode)
{
    return (mode == H2026_MODE_B2) || (mode == H2026_MODE_B4) ||
           (mode == H2026_MODE_B5) || (mode == H2026_MODE_B6);
}

bool H2026_ModeRequiresBallControl(H2026Mode mode)
{
    return (mode == H2026_MODE_B3) || (mode == H2026_MODE_B4) ||
           (mode == H2026_MODE_B5) || (mode == H2026_MODE_B6);
}

CarStatus H2026_BuildRoute(H2026Mode mode,
                           const CarConfig *config,
                           CarRoute *route)
{
    if ((config == 0) || (route == 0) || !H2026_ModeIsOfficial(mode)) {
        return CAR_ERROR_ARG;
    }
    *route = (CarRoute){0};
    if (mode != H2026_MODE_B2) {
        return CAR_ERROR_STATE;
    }
    return H2026_BuildB2(config, route);
}
