#include "app/car_app.h"

#include "core/safety_supervisor.h"

static void CarApp_FillTiming(const CarApp *app,
                              uint32_t now_ms,
                              CarOutputSnapshot *output)
{
    output->result_valid = app->result_valid;
    output->result_time_ms = app->result_time_ms;
    output->run_time_ms = app->result_valid ? app->result_time_ms :
        ((app->task_state == H2026_TASK_RUNNING) ?
         (uint32_t)(now_ms - app->run_start_ms) :
         (app->stopped_time_valid ? app->stopped_time_ms : 0U));
}

static void CarApp_LatchStoppedTime(CarApp *app, uint32_t now_ms)
{
    if (!app->result_valid && !app->stopped_time_valid &&
        (app->task_state == H2026_TASK_RUNNING)) {
        app->stopped_time_ms = (uint32_t)(now_ms - app->run_start_ms);
        app->stopped_time_valid = true;
    }
}

static bool CarApp_CurrentSegmentIsStop(const CarApp *app)
{
    return (app->executor.route != 0) &&
           (app->executor.index < app->executor.route->count) &&
           (app->executor.route->segments[app->executor.index].type ==
            CAR_SEGMENT_STOP);
}

CarStatus CarApp_Init(CarApp *app, const CarConfig *config)
{
    if ((app == 0) || (config == 0)) {
        return CAR_ERROR_ARG;
    }
    *app = (CarApp){0};
    app->config = *config;
    CarOdometry_Init(&app->odometry);
    CarRouteExecutor_Init(&app->executor);
    app->mode = H2026_MODE_B1;
    app->task_state = H2026_TASK_READY;
    return CAR_OK;
}

CarStatus CarApp_SelectMode(CarApp *app, H2026Mode mode)
{
    if ((app == 0) || !H2026_ModeIsOfficial(mode)) {
        return CAR_ERROR_ARG;
    }
    if (CarApp_IsRunning(app)) {
        return CAR_ERROR_STATE;
    }
    app->route = (CarRoute){0};
    CarRouteExecutor_Init(&app->executor);
    CarOdometry_Init(&app->odometry);
    app->line = (CarLineEstimate){0};
    app->mode = mode;
    app->run_start_ms = 0U;
    app->result_time_ms = 0U;
    app->stopped_time_ms = 0U;
    app->result_valid = false;
    app->stopped_time_valid = false;
    app->armed = false;
    app->task_state = (app->faults == CAR_FAULT_NONE) ?
        H2026_TASK_READY : H2026_TASK_FAULT;
    return CAR_OK;
}

CarStatus CarApp_Arm(CarApp *app,
                     H2026Mode mode,
                     uint32_t now_ms,
                     const CarInputSnapshot *input)
{
    CarStatus status;

    if ((app == 0) || (input == 0) || !H2026_ModeIsOfficial(mode)) {
        return CAR_ERROR_ARG;
    }
    if (CarApp_IsRunning(app) || (app->faults != CAR_FAULT_NONE)) {
        return CAR_ERROR_STATE;
    }
    CarRouteExecutor_Init(&app->executor);
    app->route = (CarRoute){0};
    CarOdometry_Init(&app->odometry);
    status = CAR_OK;
    if (H2026_ModeUsesChassisRoute(mode)) {
        if (!input->encoder.valid || !input->imu.valid) {
            return CAR_ERROR_ARG;
        }
        status = CarOdometry_Update(&app->odometry, &app->config,
                                    &input->encoder);
        if (status == CAR_OK) {
            status = H2026_BuildRoute(mode, &app->config, &app->route);
        }
        if (status == CAR_OK) {
            status = CarRouteExecutor_Start(&app->executor, &app->route,
                                            now_ms, &app->odometry);
        }
        if (status != CAR_OK) {
            app->faults |= CAR_FAULT_ROUTE_INVALID;
            app->task_state = H2026_TASK_FAULT;
            return status;
        }
    }
    app->mode = mode;
    app->run_start_ms = now_ms;
    app->result_time_ms = 0U;
    app->stopped_time_ms = 0U;
    app->result_valid = false;
    app->stopped_time_valid = false;
    app->armed = true;
    app->task_state = H2026_TASK_RUNNING;
    return CAR_OK;
}

CarStatus CarApp_Update(CarApp *app,
                        uint32_t now_ms,
                        const CarInputSnapshot *input,
                        CarOutputSnapshot *output)
{
    CarStatus status;
    CarMotorCommand motor = {0};
    CarCue cue = CAR_CUE_NONE;
    bool gray_required;

    if ((app == 0) || (input == 0) || (output == 0)) {
        return CAR_ERROR_ARG;
    }
    *output = (CarOutputSnapshot){0};
    if (input->encoder.valid) {
        (void)CarOdometry_Update(&app->odometry, &app->config,
                                 &input->encoder);
    }
    if (input->gray.valid) {
        (void)CarLineEstimator_Update(&app->config, &input->gray,
                                      &app->line);
    }

    gray_required = CarRouteExecutor_GrayRequired(&app->executor);
    app->faults |= CarSafety_Evaluate(&app->config, now_ms, input,
                                      app->executor.running,
                                      gray_required);
    if (app->faults != CAR_FAULT_NONE) {
        CarApp_LatchStoppedTime(app, now_ms);
        app->armed = false;
        app->executor.running = false;
        app->task_state = H2026_TASK_FAULT;
        output->cue = CAR_CUE_FAULT;
        output->faults = app->faults;
        CarApp_FillTiming(app, now_ms, output);
        return CAR_ERROR_STATE;
    }
    if (!app->armed) {
        output->faults = app->faults;
        output->route_index = app->executor.index;
        output->route_running = app->executor.running;
        output->route_finished = app->executor.finished ||
            (H2026_ModeWaitsForExternalCompletion(app->mode) &&
             (app->task_state == H2026_TASK_DONE));
        CarApp_FillTiming(app, now_ms, output);
        return CAR_OK;
    }

    if (H2026_ModeWaitsForExternalCompletion(app->mode)) {
        output->faults = app->faults;
        output->route_running = false;
        output->route_finished = false;
        CarApp_FillTiming(app, now_ms, output);
        return CAR_OK;
    }

    status = CarRouteExecutor_Update(&app->executor, &app->config, now_ms,
                                     &app->odometry, &app->line, &motor,
                                     &cue, &app->faults);
    if (app->faults != CAR_FAULT_NONE) {
        motor = (CarMotorCommand){0};
        cue = CAR_CUE_FAULT;
        CarApp_LatchStoppedTime(app, now_ms);
        app->armed = false;
        app->task_state = H2026_TASK_FAULT;
    }
    if (!app->result_valid && H2026_ModeIsOfficial(app->mode) &&
        ((CarApp_CurrentSegmentIsStop(app) && !motor.enable) ||
         app->executor.finished)) {
        app->result_time_ms = (uint32_t)(now_ms - app->run_start_ms);
        app->result_valid = true;
    }

    output->motor = motor;
    output->cue = cue;
    output->faults = app->faults;
    output->route_index = app->executor.index;
    output->route_running = app->executor.running;
    output->route_finished = app->executor.finished;
    CarApp_FillTiming(app, now_ms, output);
    if (app->executor.finished) {
        app->armed = false;
        app->task_state = H2026_TASK_DONE;
    }
    return status;
}

void CarApp_Stop(CarApp *app, uint32_t fault, uint32_t now_ms)
{
    if (app == 0) {
        return;
    }
    CarApp_LatchStoppedTime(app, now_ms);
    app->faults |= fault;
    app->armed = false;
    app->executor.running = false;
    app->task_state = (fault == CAR_FAULT_NONE) ?
        H2026_TASK_READY : H2026_TASK_FAULT;
}

CarStatus CarApp_CompleteExternalTask(CarApp *app, uint32_t now_ms)
{
    if ((app == 0) || !H2026_ModeWaitsForExternalCompletion(app->mode) ||
        !CarApp_IsRunning(app)) {
        return CAR_ERROR_STATE;
    }
    app->result_time_ms = (uint32_t)(now_ms - app->run_start_ms);
    app->result_valid = true;
    app->stopped_time_ms = app->result_time_ms;
    app->stopped_time_valid = true;
    app->armed = false;
    app->task_state = H2026_TASK_DONE;
    return CAR_OK;
}

bool CarApp_IsRunning(const CarApp *app)
{
    return (app != 0) && (app->task_state == H2026_TASK_RUNNING);
}

bool CarApp_IsDone(const CarApp *app)
{
    return (app != 0) && (app->task_state == H2026_TASK_DONE);
}
