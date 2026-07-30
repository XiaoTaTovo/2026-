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

static CarStatus H2026_BuildLap(const CarConfig *config,
                                float straight_speed_mm_s,
                                float arc_speed_mm_s,
                                CarRoute *route)
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
                        straight_speed_mm_s,
                        config->straight_timeout_ms, 0.0f,
                        CAR_TRACK_EXIT_DISTANCE, 0.0f) != CAR_OK) ||
        (H2026_AddTrack(route, arc_length_mm, arc_speed_mm_s,
                        config->arc_timeout_ms,
                        -1.0f / H2026_TRACK_ARC_RADIUS_MM,
                        CAR_TRACK_EXIT_DISTANCE, 0.0f) != CAR_OK) ||
        (H2026_AddTrack(route, H2026_TRACK_STRAIGHT_MM,
                        straight_speed_mm_s,
                        config->straight_timeout_ms, 0.0f,
                        CAR_TRACK_EXIT_DISTANCE, 0.0f) != CAR_OK) ||
        (H2026_AddTrack(route, arc_length_mm, arc_speed_mm_s,
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

static CarStatus H2026_BuildB4(const CarConfig *config, CarRoute *route)
{
    const CarRouteSegment start = {
        .type = CAR_SEGMENT_CUE,
        .cue = CAR_CUE_START
    };
    const CarRouteSegment stop = {
        .type = CAR_SEGMENT_STOP,
        .cue = CAR_CUE_FINISH
    };

    if ((config->ball_task_track_speed_mm_s <= 0.0f) ||
        (H2026_AddSegment(route, &start) != CAR_OK) ||
        (H2026_AddTrack(route, H2026_TRACK_STRAIGHT_MM,
                        config->ball_task_track_speed_mm_s,
                        config->straight_timeout_ms, 0.0f,
                        CAR_TRACK_EXIT_DISTANCE, 0.0f) != CAR_OK) ||
        (H2026_AddSegment(route, &stop) != CAR_OK)) {
        return CAR_ERROR_ARG;
    }
    return CAR_OK;
}

const char *H2026_ModeName(H2026Mode mode)
{
    switch (mode) {
        case H2026_MODE_B1:
            return "T1";
        case H2026_MODE_B2:
            return "T2";
        case H2026_MODE_B3:
            return "T3";
        case H2026_MODE_B4:
            return "T4";
        case H2026_MODE_B5:
            return "T5";
        case H2026_MODE_B6:
            return "T6";
        default:
            return "?";
    }
}

const char *H2026_TaskStateName(H2026TaskState state)
{
    switch (state) {
        case H2026_TASK_READY:
            return "READY";
        case H2026_TASK_RUNNING:
            return "RUN";
        case H2026_TASK_DONE:
            return "DONE";
        case H2026_TASK_FAULT:
            return "FAULT";
        default:
            return "?";
    }
}

const char *H2026_RouteSegmentName(H2026Mode mode, uint16_t route_index)
{
    if (H2026_ModeWaitsForExternalCompletion(mode)) {
        return "EXTERNAL";
    }
    if (route_index == 0U) {
        return "START";
    }
    if (route_index == 1U) {
        return "AB";
    }
    if (mode == H2026_MODE_B4) {
        return (route_index == 2U) ? "STOP" : "DONE";
    }
    switch (route_index) {
        case 2U:
            return "BC";
        case 3U:
            return "CD";
        case 4U:
            return "DA";
        case 5U:
            return "STOP";
        default:
            return "DONE";
    }
}

bool H2026_ModeIsOfficial(H2026Mode mode)
{
    return (mode >= H2026_MODE_B1) && (mode <= H2026_MODE_B6);
}

bool H2026_ModeUsesLine(H2026Mode mode)
{
    return (mode == H2026_MODE_B2) || (mode == H2026_MODE_B4) ||
           (mode == H2026_MODE_B5) || (mode == H2026_MODE_B6);
}

bool H2026_ModeUsesChassisRoute(H2026Mode mode)
{
    return H2026_ModeUsesLine(mode);
}

bool H2026_ModeWaitsForExternalCompletion(H2026Mode mode)
{
    return (mode == H2026_MODE_B1) || (mode == H2026_MODE_B3);
}

bool H2026_ModeRequiresBallControl(H2026Mode mode)
{
    return (mode == H2026_MODE_B3) || (mode == H2026_MODE_B4) ||
           (mode == H2026_MODE_B5) || (mode == H2026_MODE_B6);
}

H2026Mode H2026_NextMode(H2026Mode mode)
{
    return (mode >= H2026_MODE_B1) && (mode < H2026_MODE_B6) ?
        (H2026Mode)(mode + 1) : H2026_MODE_B1;
}

uint32_t H2026_ModeTimeLimitMs(H2026Mode mode)
{
    switch (mode) {
        case H2026_MODE_B2:
            return 20000U;
        case H2026_MODE_B3:
            return 5000U;
        case H2026_MODE_B4:
            return 8000U;
        case H2026_MODE_B5:
        case H2026_MODE_B6:
            return 30000U;
        case H2026_MODE_B1:
        default:
            return 0U;
    }
}

CarStatus H2026_BuildRoute(H2026Mode mode,
                           const CarConfig *config,
                           CarRoute *route)
{
    if ((config == 0) || (route == 0) || !H2026_ModeIsOfficial(mode)) {
        return CAR_ERROR_ARG;
    }
    *route = (CarRoute){0};
    switch (mode) {
        case H2026_MODE_B2:
            return H2026_BuildLap(config, config->straight_speed_mm_s,
                                  config->arc_speed_mm_s, route);
        case H2026_MODE_B4:
            return H2026_BuildB4(config, route);
        case H2026_MODE_B5:
        case H2026_MODE_B6:
            return H2026_BuildLap(config,
                                  config->ball_task_track_speed_mm_s,
                                  config->ball_task_track_speed_mm_s,
                                  route);
        case H2026_MODE_B1:
        case H2026_MODE_B3:
        default:
            return CAR_ERROR_STATE;
    }
}
