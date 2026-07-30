#ifndef H2026_TASK_H
#define H2026_TASK_H

#include <stdbool.h>

#include "car_config.h"
#include "core/route_executor.h"

typedef enum {
    H2026_MODE_B1 = 1,
    H2026_MODE_B2,
    H2026_MODE_B3,
    H2026_MODE_B4,
    H2026_MODE_B5,
    H2026_MODE_B6
} H2026Mode;

typedef enum {
    H2026_TASK_READY = 0,
    H2026_TASK_RUNNING,
    H2026_TASK_DONE,
    H2026_TASK_FAULT
} H2026TaskState;

#define H2026_TRACK_STRAIGHT_MM (1500.0f)
#define H2026_TRACK_ARC_RADIUS_MM (500.0f)
#define H2026_TRACK_ARC_ANGLE_DEG (180.0f)
#define H2026_TRACK_TOTAL_MM \
    (2.0f * H2026_TRACK_STRAIGHT_MM + \
     2.0f * 3.14159265358979323846f * H2026_TRACK_ARC_RADIUS_MM)

const char *H2026_ModeName(H2026Mode mode);
const char *H2026_TaskStateName(H2026TaskState state);
const char *H2026_RouteSegmentName(H2026Mode mode, uint16_t route_index);
bool H2026_ModeIsOfficial(H2026Mode mode);
bool H2026_ModeUsesLine(H2026Mode mode);
bool H2026_ModeUsesChassisRoute(H2026Mode mode);
bool H2026_ModeWaitsForExternalCompletion(H2026Mode mode);
bool H2026_ModeRequiresBallControl(H2026Mode mode);
H2026Mode H2026_NextMode(H2026Mode mode);
uint32_t H2026_ModeTimeLimitMs(H2026Mode mode);
CarStatus H2026_BuildRoute(H2026Mode mode,
                           const CarConfig *config,
                           CarRoute *route);

#endif
