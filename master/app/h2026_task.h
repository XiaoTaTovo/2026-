#ifndef H2026_TASK_H
#define H2026_TASK_H

#include <stdbool.h>

#include "app/h2024_task.h"

#define H2026_TRACK_STRAIGHT_MM (1500.0f)
#define H2026_TRACK_ARC_RADIUS_MM (500.0f)
#define H2026_TRACK_ARC_ANGLE_DEG (180.0f)
#define H2026_TRACK_TOTAL_MM \
    (2.0f * H2026_TRACK_STRAIGHT_MM + \
     2.0f * 3.14159265358979323846f * H2026_TRACK_ARC_RADIUS_MM)

bool H2026_ModeIsOfficial(H2024Mode mode);
bool H2026_ModeUsesLine(H2024Mode mode);
bool H2026_ModeRequiresBallControl(H2024Mode mode);
CarStatus H2026_BuildRoute(H2024Mode mode,
                           const CarConfig *config,
                           CarRoute *route);

#endif
