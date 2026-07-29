#ifndef H2024_ODOMETRY_H
#define H2024_ODOMETRY_H

#include "car_config.h"
#include "car_types.h"

typedef struct {
    int16_t previous_left_count;
    int16_t previous_right_count;
    float left_distance_mm;
    float right_distance_mm;
    float center_distance_mm;
    float encoder_heading_deg;
    uint32_t last_update_ms;
    bool initialized;
} CarOdometry;

void CarOdometry_Init(CarOdometry *odometry);
CarStatus CarOdometry_Update(CarOdometry *odometry,
                             const CarConfig *config,
                             const CarEncoderSample *sample);

#endif
