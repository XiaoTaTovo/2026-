#include "firmware.h"

static bool CarFirmware_GrayCalibrationReady(const CarFirmware *firmware)
{
    return !firmware->config.require_runtime_gray_calibration ||
           (firmware->gray_cal_state == CAR_GRAY_CAL_READY);
}

static const uint16_t *CarFirmware_TrackRaw(const CarFirmware *firmware)
{
    return firmware->gray.raw;
}

static int32_t CarFirmware_GrayMinimumCalibrationSpan(void)
{
    return GRAY_ARRAY_MIN_CALIBRATION_SPAN;
}

static bool CarFirmware_SetTrackCalibration(
    CarFirmware *firmware,
    const uint16_t black[GRAY_ARRAY_CHANNELS],
    const uint16_t white[GRAY_ARRAY_CHANNELS])
{
    return GrayArray_SetCalibration(&firmware->gray, black, white);
}

static bool CarFirmware_ReadTrack(CarFirmware *firmware, uint32_t now_ms)
{
    return GrayArray_Read(&firmware->gray, now_ms);
}

static bool CarFirmware_GetLatestTrack(const CarFirmware *firmware,
                                       CarGraySample *sample)
{
    return GrayArray_GetLatest(&firmware->gray, sample);
}

static void CarFirmware_BeginGrayCapture(CarFirmware *firmware,
                                         CarGrayCalibrationState state)
{
    for (uint8_t i = 0U; i < GRAY_ARRAY_CHANNELS; i++) {
        firmware->gray_cal_sum[i] = 0U;
    }
    firmware->gray_cal_frame_count = 0U;
    firmware->gray_cal_state = state;
}

static void CarFirmware_HandleGrayCalibrationButton(CarFirmware *firmware,
                                                     uint32_t now_ms)
{
    if (!Button_TakePressedEvent(&firmware->gray_cal_button)) {
        return;
    }
    if (CarApp_IsRunning(&firmware->app)) {
        Buzzer_PlayCue(&firmware->buzzer, CAR_CUE_FAULT, now_ms);
        return;
    }
    if (firmware->gray_cal_state == CAR_GRAY_CAL_WAIT_BLACK) {
        CarFirmware_BeginGrayCapture(firmware,
                                     CAR_GRAY_CAL_CAPTURE_BLACK);
    } else if ((firmware->gray_cal_state != CAR_GRAY_CAL_CAPTURE_WHITE) &&
               (firmware->gray_cal_state != CAR_GRAY_CAL_CAPTURE_BLACK)) {
        CarFirmware_BeginGrayCapture(firmware,
                                     CAR_GRAY_CAL_CAPTURE_WHITE);
    }
}

static void CarFirmware_HandleTaskButton(CarFirmware *firmware,
                                         uint32_t now_ms)
{
    CarStatus status;

    if (!Button_TakePressedEvent(&firmware->task_button)) {
        return;
    }
    status = CarFirmware_SelectNextTask(firmware);
    if (status == CAR_OK) {
        firmware->task_switch_count++;
        Buzzer_PlayCue(&firmware->buzzer, CAR_CUE_CHECKPOINT, now_ms);
    } else {
        firmware->task_switch_reject_count++;
        Buzzer_PlayCue(&firmware->buzzer, CAR_CUE_FAULT, now_ms);
    }
}

static void CarFirmware_AccumulateGrayCalibration(CarFirmware *firmware,
                                                   uint32_t now_ms)
{
    const uint16_t *raw;
    uint16_t *target;

    if ((firmware->gray_cal_state != CAR_GRAY_CAL_CAPTURE_WHITE) &&
        (firmware->gray_cal_state != CAR_GRAY_CAL_CAPTURE_BLACK)) {
        return;
    }
    raw = CarFirmware_TrackRaw(firmware);
    for (uint8_t i = 0U; i < GRAY_ARRAY_CHANNELS; i++) {
        firmware->gray_cal_sum[i] += raw[i];
    }
    firmware->gray_cal_frame_count++;
    if (firmware->gray_cal_frame_count < CAR_GRAY_CALIBRATION_FRAMES) {
        return;
    }

    target = (firmware->gray_cal_state == CAR_GRAY_CAL_CAPTURE_WHITE) ?
        firmware->gray_cal_white : firmware->gray_cal_black;
    for (uint8_t i = 0U; i < GRAY_ARRAY_CHANNELS; i++) {
        target[i] = (uint16_t)(firmware->gray_cal_sum[i] /
                              CAR_GRAY_CALIBRATION_FRAMES);
    }
    if (firmware->gray_cal_state == CAR_GRAY_CAL_CAPTURE_WHITE) {
        firmware->gray_cal_state = CAR_GRAY_CAL_WAIT_BLACK;
        return;
    }

    if (CarFirmware_SetTrackCalibration(firmware,
                                        firmware->gray_cal_black,
                                        firmware->gray_cal_white)) {
        for (uint8_t i = 0U; i < GRAY_ARRAY_CHANNELS; i++) {
            firmware->config.gray_black[i] = firmware->gray_cal_black[i];
            firmware->config.gray_white[i] = firmware->gray_cal_white[i];
        }
        firmware->config.gray_calibration_valid = true;
        firmware->gray_sample.valid = false;
        firmware->gray_cal_state = CAR_GRAY_CAL_READY;
        Buzzer_PlayCue(&firmware->buzzer, CAR_CUE_CHECKPOINT, now_ms);
        return;
    }

    firmware->gray_cal_bad_channel = 0U;
    firmware->gray_cal_bad_span = 0;
    for (uint8_t i = 0U; i < GRAY_ARRAY_CHANNELS; i++) {
        int32_t span = (int32_t)firmware->gray_cal_white[i] -
                       (int32_t)firmware->gray_cal_black[i];
        int32_t minimum_span = CarFirmware_GrayMinimumCalibrationSpan();

        if ((span > -minimum_span) && (span < minimum_span)) {
            firmware->gray_cal_bad_channel = i;
            firmware->gray_cal_bad_span = (int16_t)span;
            break;
        }
    }
    firmware->gray_cal_state = CAR_GRAY_CAL_ERROR;
    Buzzer_PlayCue(&firmware->buzzer, CAR_CUE_FAULT, now_ms);
}

static float CarFirmware_SelectGyro(const CarFirmware *firmware,
                                    const Icm42688Sample *sample)
{
    int16_t raw;

    switch (firmware->config.yaw_axis) {
        case CAR_IMU_AXIS_X:
            raw = sample->gyro_x;
            break;
        case CAR_IMU_AXIS_Y:
            raw = sample->gyro_y;
            break;
        case CAR_IMU_AXIS_Z:
        default:
            raw = sample->gyro_z;
            break;
    }
    return ((float)raw / firmware->imu.gyro_lsb_per_dps) *
           (float)firmware->config.yaw_sign;
}

static int16_t CarFirmware_RoundSpeed(float speed_mm_s)
{
    if (speed_mm_s > 32767.0f) {
        return 32767;
    }
    if (speed_mm_s < -32768.0f) {
        return -32768;
    }
    return (int16_t)(speed_mm_s + ((speed_mm_s >= 0.0f) ? 0.5f : -0.5f));
}

static void CarFirmware_StopDrive(CarFirmware *firmware)
{
    firmware->config.drive.stop(firmware->config.drive.context);
    firmware->drive_active = false;
}

static void CarFirmware_ApplyOutput(CarFirmware *firmware, uint32_t now_ms)
{
    if ((firmware->output.faults != CAR_FAULT_NONE) ||
        (firmware->hardware_faults != CAR_FAULT_NONE)) {
        CarFirmware_StopDrive(firmware);
        return;
    }
    if (firmware->output.motor.enable) {
        if (!firmware->config.drive.set_wheel_speeds(
                CarFirmware_RoundSpeed(firmware->output.motor.left_mm_s),
                CarFirmware_RoundSpeed(firmware->output.motor.right_mm_s),
                firmware->config.drive.context)) {
            CarFirmware_ForceStop(firmware, CAR_FAULT_MOTOR_IO);
            return;
        }
        firmware->drive_active = true;
    } else if (firmware->drive_active ||
               CarPeriodicTask_Due(&firmware->stop_refresh_task, now_ms)) {
        CarFirmware_StopDrive(firmware);
    }
}

static void CarFirmware_RunImu(CarFirmware *firmware, uint32_t now_ms)
{
    Icm42688Sample raw = {0};
    float gyro_dps;

    if (!Icm42688_ReadSample(&firmware->imu, now_ms, &raw)) {
        firmware->imu_sample.valid = false;
        return;
    }
    gyro_dps = CarFirmware_SelectGyro(firmware, &raw);
    (void)CarYawEstimator_Update(&firmware->yaw, gyro_dps, now_ms);
    if (CarYawEstimator_GetSample(&firmware->yaw,
                                  &firmware->imu_sample)) {
        firmware->imu_sample.yaw_rate_dps =
            gyro_dps - firmware->yaw.bias_dps;
    }
}

static void CarFirmware_RunControl(CarFirmware *firmware, uint32_t now_ms)
{
    CarInputSnapshot input = {0};
    bool start_event = Button_TakePressedEvent(&firmware->button);

    firmware->encoder_valid_current =
        firmware->config.drive.read_encoder(
            &input.encoder, firmware->config.drive.context);
    input.imu = firmware->imu_sample;
    input.gray = firmware->gray_sample;
    input.start_pressed = start_event;

    if (start_event) {
        firmware->button_event_count++;
        firmware->last_button_event_ms = now_ms;
        firmware->last_button_encoder_valid = input.encoder.valid;
        firmware->last_button_imu_valid = input.imu.valid;
        firmware->last_button_drive_ready = true;
        firmware->last_arm_status = CAR_OK;
        firmware->last_button_action = CAR_BUTTON_ACTION_NONE;
    }
    if (start_event && CarApp_IsRunning(&firmware->app)) {
        firmware->last_button_action = CAR_BUTTON_ACTION_EMERGENCY_STOP;
        CarFirmware_ForceStop(firmware, CAR_FAULT_EMERGENCY_STOP);
        Buzzer_PlayCue(&firmware->buzzer, CAR_CUE_FAULT, now_ms);
        return;
    } else if (start_event &&
               (firmware->hardware_faults != CAR_FAULT_NONE)) {
        firmware->last_arm_status = CAR_ERROR_STATE;
        firmware->last_button_action = CAR_BUTTON_ACTION_ARM_REJECTED;
        Buzzer_PlayCue(&firmware->buzzer, CAR_CUE_FAULT, now_ms);
    } else if (start_event && !firmware->app.armed) {
        if (H2026_ModeUsesLine(firmware->config.mode) &&
            !CarFirmware_GrayCalibrationReady(firmware)) {
            firmware->last_arm_status = CAR_ERROR_STATE;
            firmware->last_button_action = CAR_BUTTON_ACTION_GRAY_REQUIRED;
            Buzzer_PlayCue(&firmware->buzzer, CAR_CUE_FAULT, now_ms);
        } else {
            CarYawEstimator_ResetYaw(&firmware->yaw, 0.0f);
            input.imu.yaw_deg = 0.0f;
            firmware->last_arm_status = CarApp_Arm(
                &firmware->app, firmware->config.mode, now_ms, &input);
        }
        if ((firmware->last_arm_status != CAR_OK) &&
            (firmware->last_button_action !=
             CAR_BUTTON_ACTION_GRAY_REQUIRED)) {
            firmware->last_button_action = CAR_BUTTON_ACTION_ARM_REJECTED;
            Buzzer_PlayCue(&firmware->buzzer, CAR_CUE_FAULT, now_ms);
        } else if (firmware->last_arm_status == CAR_OK) {
            firmware->last_button_action = CAR_BUTTON_ACTION_ARM_OK;
            if (H2026_ModeWaitsForExternalCompletion(
                    firmware->config.mode)) {
                Buzzer_PlayCue(&firmware->buzzer, CAR_CUE_START, now_ms);
            }
        }
    }

    (void)CarApp_Update(&firmware->app, now_ms, &input,
                        &firmware->output);
    firmware->output.faults |= firmware->hardware_faults;
    if ((firmware->output.cue != CAR_CUE_NONE) &&
        (firmware->output.cue != firmware->last_output_cue)) {
        Buzzer_PlayCue(&firmware->buzzer, firmware->output.cue, now_ms);
    }
    firmware->last_output_cue = firmware->output.cue;
    CarFirmware_ApplyOutput(firmware, now_ms);
}

CarStatus CarFirmware_Init(CarFirmware *firmware,
                           const CarFirmwareConfig *config,
                           uint32_t now_ms)
{
    bool track_setup_ok;

    if ((firmware == 0) || (config == 0) ||
        (config->drive.set_wheel_speeds == 0) ||
        (config->drive.read_encoder == 0) ||
        (config->drive.stop == 0) || (config->drive.service == 0) ||
        ((config->yaw_sign != 1) && (config->yaw_sign != -1))) {
        return CAR_ERROR_ARG;
    }
    *firmware = (CarFirmware){0};
    firmware->config = *config;
    firmware->last_tick_ms = now_ms;
    Icm42688_InitObject(&firmware->imu, &config->imu);
    GrayArray_Init(&firmware->gray, &config->gray);
    Button_Init(&firmware->button, config->button_read,
                config->button_context, config->button_active_low,
                config->button_debounce_ms);
    Button_Init(&firmware->gray_cal_button,
                config->gray_cal_button_read,
                config->gray_cal_button_context,
                config->button_active_low,
                config->button_debounce_ms);
    Button_Init(&firmware->task_button,
                config->task_button_read,
                config->task_button_context,
                config->button_active_low,
                config->button_debounce_ms);
    Buzzer_Init(&firmware->buzzer, config->buzzer_set,
                config->buzzer_context);
    CarYawEstimator_Init(&firmware->yaw,
                         config->imu_calibration_samples,
                         config->imu_max_step_ms,
                         config->yaw_bias_dps,
                         config->yaw_bias_fixed);
    if (CarApp_Init(&firmware->app, &config->car) != CAR_OK) {
        return CAR_ERROR_ARG;
    }
    if (CarApp_SelectMode(&firmware->app, config->mode) != CAR_OK) {
        return CAR_ERROR_ARG;
    }
    if (!Icm42688_Initialize(&firmware->imu)) {
        firmware->hardware_faults |= CAR_FAULT_IMU_INIT;
    }
    track_setup_ok = config->gray_calibration_valid &&
        GrayArray_SetCalibration(&firmware->gray,
                                 config->gray_black,
                                 config->gray_white);
    if (!track_setup_ok && !config->require_runtime_gray_calibration &&
        H2026_ModeUsesLine(config->mode)) {
        firmware->hardware_faults |= CAR_FAULT_GRAY_NOT_CALIBRATED;
    }
    firmware->gray_cal_state =
        (config->require_runtime_gray_calibration &&
         (config->gray_cal_button_read != 0)) ?
        CAR_GRAY_CAL_WAIT_WHITE : CAR_GRAY_CAL_READY;

    CarPeriodicTask_Init(&firmware->imu_task, 5U, now_ms);
    CarPeriodicTask_Init(&firmware->gray_task, 10U, now_ms);
    CarPeriodicTask_Init(&firmware->control_task, 20U, now_ms);
    CarPeriodicTask_Init(&firmware->stop_refresh_task, 100U, now_ms);
    firmware->initialized = true;
    CarFirmware_StopDrive(firmware);
    return CAR_OK;
}

void CarFirmware_Tick(CarFirmware *firmware, uint32_t now_ms)
{
    if ((firmware == 0) || !firmware->initialized) {
        return;
    }
    firmware->last_tick_ms = now_ms;
    Button_Update(&firmware->button, now_ms);
    Button_Update(&firmware->gray_cal_button, now_ms);
    Button_Update(&firmware->task_button, now_ms);
    Buzzer_Update(&firmware->buzzer, now_ms);
    CarFirmware_HandleTaskButton(firmware, now_ms);
    CarFirmware_HandleGrayCalibrationButton(firmware, now_ms);

    if (CarPeriodicTask_Due(&firmware->imu_task, now_ms)) {
        CarFirmware_RunImu(firmware, now_ms);
    }
    if (CarPeriodicTask_Due(&firmware->gray_task, now_ms)) {
        if (CarFirmware_ReadTrack(firmware, now_ms)) {
            CarFirmware_AccumulateGrayCalibration(firmware, now_ms);
            if (!CarFirmware_GetLatestTrack(firmware,
                                            &firmware->gray_sample)) {
                firmware->gray_sample.valid = false;
            }
        } else {
            firmware->gray_sample.valid = false;
        }
    }
    if (CarPeriodicTask_Due(&firmware->control_task, now_ms)) {
        CarFirmware_RunControl(firmware, now_ms);
    }
    firmware->config.drive.service(firmware->config.drive.context);
}

void CarFirmware_ForceStop(CarFirmware *firmware, uint32_t fault)
{
    if (firmware == 0) {
        return;
    }
    firmware->hardware_faults |= fault;
    CarApp_Stop(&firmware->app, fault, firmware->last_tick_ms);
    firmware->output = (CarOutputSnapshot){0};
    firmware->output.faults = firmware->app.faults |
        firmware->hardware_faults;
    firmware->output.cue = CAR_CUE_FAULT;
    firmware->output.route_index = firmware->app.executor.index;
    firmware->output.route_finished = firmware->app.executor.finished;
    firmware->output.result_time_ms = firmware->app.result_time_ms;
    firmware->output.result_valid = firmware->app.result_valid;
    firmware->output.run_time_ms = firmware->app.result_valid ?
        firmware->app.result_time_ms :
        (firmware->app.stopped_time_valid ?
         firmware->app.stopped_time_ms : 0U);
    CarFirmware_StopDrive(firmware);
}

CarStatus CarFirmware_SelectNextTask(CarFirmware *firmware)
{
    H2026Mode next_mode;
    CarStatus status;

    if ((firmware == 0) || !firmware->initialized ||
        CarApp_IsRunning(&firmware->app)) {
        return CAR_ERROR_STATE;
    }
    next_mode = H2026_NextMode(firmware->config.mode);
    status = CarApp_SelectMode(&firmware->app, next_mode);
    if (status != CAR_OK) {
        return status;
    }
    firmware->config.mode = next_mode;
    firmware->output = (CarOutputSnapshot){0};
    firmware->last_button_action = CAR_BUTTON_ACTION_NONE;
    CarFirmware_StopDrive(firmware);
    return CAR_OK;
}

CarStatus CarFirmware_CompleteExternalTask(CarFirmware *firmware,
                                           uint32_t now_ms)
{
    CarStatus status;

    if ((firmware == 0) || !firmware->initialized) {
        return CAR_ERROR_ARG;
    }
    status = CarApp_CompleteExternalTask(&firmware->app, now_ms);
    if (status == CAR_OK) {
        firmware->output = (CarOutputSnapshot){0};
        firmware->output.result_valid = true;
        firmware->output.result_time_ms = firmware->app.result_time_ms;
        firmware->output.run_time_ms = firmware->app.result_time_ms;
        firmware->output.route_finished = true;
        Buzzer_PlayCue(&firmware->buzzer, CAR_CUE_FINISH, now_ms);
        CarFirmware_StopDrive(firmware);
    }
    return status;
}

const CarOutputSnapshot *CarFirmware_GetOutput(const CarFirmware *firmware)
{
    return (firmware == 0) ? 0 : &firmware->output;
}
