#ifndef H2024_CAR_APP_H
#define H2024_CAR_APP_H

#include "app/h2024_task.h"
#include "car_config.h"
#include "car_types.h"
#include "core/line_estimator.h"
#include "core/odometry.h"
#include "core/route_executor.h"

typedef struct {
    CarConfig config;
    CarRoute route;
    CarRouteExecutor executor;
    CarOdometry odometry;
    CarLineEstimate line;
    uint32_t faults;
    uint32_t run_start_ms;
    uint32_t result_time_ms;
    uint32_t stopped_time_ms;
    H2024Mode mode;
    bool result_valid;
    bool stopped_time_valid;
    bool armed;
} CarApp;

CarStatus CarApp_Init(CarApp *app, const CarConfig *config);
CarStatus CarApp_Arm(CarApp *app,
                     H2024Mode mode,
                     uint32_t now_ms,
                     const CarInputSnapshot *input);
CarStatus CarApp_Update(CarApp *app,
                        uint32_t now_ms,
                        const CarInputSnapshot *input,
                        CarOutputSnapshot *output);
void CarApp_Stop(CarApp *app, uint32_t fault, uint32_t now_ms);

#endif
