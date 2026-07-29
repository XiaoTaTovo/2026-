#include "firmware.h"

#include "app/h2026_task.h"

static bool CarFirmware_GrayCalibrationReady(const CarFirmware *firmware)
{
    return !firmware->config.require_runtime_gray_calibration ||
           (firmware->gray_cal_state == CAR_GRAY_CAL_READY);
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

    /* 运行中禁止改动标定，防止误按 KEY3 瞬间改变循迹输入。 */
    if (firmware->app.executor.running) {
        Buzzer_PlayCue(&firmware->buzzer, CAR_CUE_FAULT, now_ms);
        return;
    }

    if (firmware->gray_cal_state == CAR_GRAY_CAL_WAIT_BLACK) {
        CarFirmware_BeginGrayCapture(firmware,
                                     CAR_GRAY_CAL_CAPTURE_BLACK);
    } else if ((firmware->gray_cal_state != CAR_GRAY_CAL_CAPTURE_WHITE) &&
               (firmware->gray_cal_state != CAR_GRAY_CAL_CAPTURE_BLACK)) {
        /* READY/ERROR 状态再次按 KEY3，也从白场开始一轮全新标定。 */
        CarFirmware_BeginGrayCapture(firmware,
                                     CAR_GRAY_CAL_CAPTURE_WHITE);
    }
}

static void CarFirmware_AccumulateGrayCalibration(CarFirmware *firmware,
                                                   uint32_t now_ms)
{
    uint16_t *target;

    if ((firmware->gray_cal_state != CAR_GRAY_CAL_CAPTURE_WHITE) &&
        (firmware->gray_cal_state != CAR_GRAY_CAL_CAPTURE_BLACK)) {
        return;
    }

    for (uint8_t i = 0U; i < GRAY_ARRAY_CHANNELS; i++) {
        firmware->gray_cal_sum[i] += firmware->gray.raw[i];
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

    if (GrayArray_SetCalibration(&firmware->gray,
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
        if ((span > -20) && (span < 20)) {
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

static int16_t CarFirmware_ToMotorUnits(const CarFirmware *firmware,
                                        float speed_mm_s)
{
    float scaled = speed_mm_s * firmware->config.motor_units_per_mm_s;

    if (scaled > 32767.0f) {
        return 32767;
    }
    if (scaled < -32768.0f) {
        return -32768;
    }
    return (int16_t)(scaled + ((scaled >= 0.0f) ? 0.5f : -0.5f));
}

static bool CarFirmware_TimeReached(uint32_t now_ms, uint32_t deadline_ms)
{
    return (int32_t)(now_ms - deadline_ms) >= 0;
}

static void CarFirmware_ScheduleMotorPrepare(CarFirmware *firmware,
                                             uint32_t now_ms)
{
    firmware->motor_prepare_next_ms =
        now_ms + firmware->config.motor_command_spacing_ms;
}

static bool CarFirmware_BeginMotorPrepare(CarFirmware *firmware,
                                          uint32_t now_ms)
{
    if (!MotorBoard_Stop(&firmware->motor)) {
        return false;
    }
    firmware->motor_armed = false;
    firmware->motor_prepare_active = true;
    firmware->motor_prepare_step = CAR_MOTOR_PREP_POLARITY_A;
    CarFirmware_ScheduleMotorPrepare(firmware, now_ms);
    return true;
}

static bool CarFirmware_StepMotorPrepare(CarFirmware *firmware,
                                         uint32_t now_ms)
{
    bool sent;

    if (!firmware->motor_prepare_active ||
        !CarFirmware_TimeReached(now_ms, firmware->motor_prepare_next_ms)) {
        return true;
    }

    /*
     * The verified example enables closed loop first. The tested board can
     * run away at a zero target when polarity is still at its power-on value,
     * so this port keeps the verified values but applies polarity before
     * enabling closed loop.
     */
    while (firmware->motor_prepare_active) {
        switch (firmware->motor_prepare_step) {
            case CAR_MOTOR_PREP_POLARITY_A:
            case CAR_MOTOR_PREP_POLARITY_B:
            case CAR_MOTOR_PREP_POLARITY_C:
            case CAR_MOTOR_PREP_POLARITY_D:
            {
                MotorBoardChannel channel = (MotorBoardChannel)(
                    firmware->motor_prepare_step -
                    CAR_MOTOR_PREP_POLARITY_A);

                firmware->motor_prepare_step = (CarMotorPrepareStep)(
                    firmware->motor_prepare_step + 1);
                if (!firmware->config.set_encoder_polarity_on_arm) {
                    continue;
                }
                sent = MotorBoard_SetEncoderPolarity(
                    &firmware->motor, channel,
                    firmware->config.encoder_polarity[channel]);
                break;
            }
            case CAR_MOTOR_PREP_PID:
                firmware->motor_prepare_step =
                    CAR_MOTOR_PREP_ENABLE_CLOSED_LOOP;
                if (!firmware->config.set_speed_pid_on_arm) {
                    continue;
                }
                sent = MotorBoard_SetAllPid(
                    &firmware->motor, firmware->config.speed_pid);
                break;
            case CAR_MOTOR_PREP_ENABLE_CLOSED_LOOP:
                sent = MotorBoard_SetClosedLoop(&firmware->motor, true);
                firmware->motor_prepare_step = CAR_MOTOR_PREP_FINAL_ZERO;
                break;
            case CAR_MOTOR_PREP_FINAL_ZERO:
                sent = MotorBoard_Stop(&firmware->motor);
                if (sent) {
                    firmware->motor_prepare_active = false;
                    firmware->motor_prepare_step = CAR_MOTOR_PREP_IDLE;
                    firmware->motor_armed = true;
                    return true;
                }
                break;
            case CAR_MOTOR_PREP_IDLE:
            default:
                sent = false;
                break;
        }

        if (!sent) {
            firmware->motor_prepare_active = false;
            firmware->motor_prepare_step = CAR_MOTOR_PREP_IDLE;
            CarFirmware_ForceStop(firmware, CAR_FAULT_MOTOR_IO);
            return false;
        }
        CarFirmware_ScheduleMotorPrepare(firmware, now_ms);
        return true;
    }
    return true;
}

static void CarFirmware_ApplyOutput(CarFirmware *firmware, uint32_t now_ms)
{
    if ((firmware->output.faults != CAR_FAULT_NONE) ||
        (firmware->hardware_faults != CAR_FAULT_NONE)) {
        firmware->motor_prepare_active = false;
        firmware->motor_prepare_step = CAR_MOTOR_PREP_IDLE;
        (void)MotorBoard_EmergencyStop(&firmware->motor);
        firmware->motor_armed = false;
        return;
    }

    if (firmware->output.motor.enable) {
        if (!firmware->motor_armed) {
            if (!firmware->motor_prepare_active &&
                !CarFirmware_BeginMotorPrepare(firmware, now_ms)) {
                CarFirmware_ForceStop(firmware, CAR_FAULT_MOTOR_IO);
            }
            return;
        }
        if (!MotorBoard_SetWheelSpeeds(
                &firmware->motor,
                CarFirmware_ToMotorUnits(firmware,
                                         firmware->output.motor.left_mm_s),
                CarFirmware_ToMotorUnits(firmware,
                                         firmware->output.motor.right_mm_s))) {
            CarFirmware_ForceStop(firmware, CAR_FAULT_MOTOR_IO);
        }
    } else if (firmware->motor_prepare_active) {
        return;
    } else if (firmware->motor_armed ||
               CarPeriodicTask_Due(&firmware->stop_refresh_task, now_ms)) {
        if (!MotorBoard_Stop(&firmware->motor)) {
            CarFirmware_ForceStop(firmware, CAR_FAULT_MOTOR_IO);
        }
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
    (void)CarYawEstimator_Update(
        &firmware->yaw, gyro_dps, now_ms);
    if (CarYawEstimator_GetSample(&firmware->yaw,
                                  &firmware->imu_sample)) {
        /* Keep the bias-corrected rate observable; yaw alone can hide a stale
         * or mechanically detached IMU during an arc-to-straight handoff. */
        firmware->imu_sample.yaw_rate_dps = gyro_dps - firmware->yaw.bias_dps;
    }
}

static void CarFirmware_RunControl(CarFirmware *firmware, uint32_t now_ms)
{
    CarInputSnapshot input = {0};
    bool start_event = Button_TakePressedEvent(&firmware->button);

    firmware->encoder_valid_current = MotorBoard_GetEncoderSample(
        &firmware->motor, &input.encoder);
    input.imu = firmware->imu_sample;
    input.gray = firmware->gray_sample;
    input.start_pressed = start_event;

    if (start_event) {
        firmware->button_event_count++;
        firmware->last_button_event_ms = now_ms;
        firmware->last_button_encoder_valid = input.encoder.valid;
        firmware->last_button_imu_valid = input.imu.valid;
        firmware->last_button_motor_armed = firmware->motor_armed;
        firmware->last_arm_status = CAR_OK;
        firmware->last_button_action = CAR_BUTTON_ACTION_NONE;
    }

    if (start_event && firmware->app.executor.running) {
        input.emergency_stop = true;
        firmware->last_button_action = CAR_BUTTON_ACTION_EMERGENCY_STOP;
    } else if (start_event && !firmware->app.armed) {
        if (!CarFirmware_GrayCalibrationReady(firmware)) {
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
        }
    }

    (void)CarApp_Update(&firmware->app, now_ms, &input, &firmware->output);
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
    if ((firmware == 0) || (config == 0) ||
        (config->motor_units_per_mm_s <= 0.0f) ||
        (config->motor_command_spacing_ms == 0U) ||
        ((config->yaw_sign != 1) && (config->yaw_sign != -1))) {
        return CAR_ERROR_ARG;
    }

    *firmware = (CarFirmware){0};
    firmware->config = *config;
    firmware->last_tick_ms = now_ms;
    MotorBoard_Init(&firmware->motor, &config->motor);
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

    if (!Icm42688_Initialize(&firmware->imu)) {
        firmware->hardware_faults |= CAR_FAULT_IMU_INIT;
    }
    {
        bool gray_setup_ok = config->gray_calibration_valid &&
            GrayArray_SetCalibration(&firmware->gray,
                                     config->gray_black,
                                     config->gray_white);

        if (!gray_setup_ok && !config->require_runtime_gray_calibration &&
            ((config->mode == H2024_MODE_ITEM_2) ||
             (config->mode == H2024_MODE_ITEM_3) ||
             (config->mode == H2024_MODE_ITEM_4) ||
             /* 2026 路线都用灰度识别节点/弧线，未标定时禁止启动。 */
             (config->mode == H2026_MODE_ITEM_1) ||
             (config->mode == H2026_MODE_ITEM_2) ||
             (config->mode == H2026_MODE_ITEM_3) ||
             (config->mode == H2026_MODE_ITEM_4) ||
             H2026_ModeUsesLine(config->mode))) {
            firmware->hardware_faults |= CAR_FAULT_GRAY_NOT_CALIBRATED;
        }
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
    if ((firmware->hardware_faults == CAR_FAULT_NONE) &&
        !CarFirmware_BeginMotorPrepare(firmware, now_ms)) {
        CarFirmware_ForceStop(firmware, CAR_FAULT_MOTOR_IO);
    } else if (firmware->hardware_faults != CAR_FAULT_NONE) {
        (void)MotorBoard_Stop(&firmware->motor);
    }
    return CAR_OK;
}

void CarFirmware_OnMotorRxByte(CarFirmware *firmware,
                               uint8_t byte,
                               uint32_t now_ms)
{
    if ((firmware != 0) && firmware->initialized) {
        MotorBoard_OnRxByte(&firmware->motor, byte, now_ms);
    }
}

void CarFirmware_Tick(CarFirmware *firmware, uint32_t now_ms)
{
    if ((firmware == 0) || !firmware->initialized) {
        return;
    }
    firmware->last_tick_ms = now_ms;
    (void)CarFirmware_StepMotorPrepare(firmware, now_ms);
    Button_Update(&firmware->button, now_ms);
    Button_Update(&firmware->gray_cal_button, now_ms);
    Buzzer_Update(&firmware->buzzer, now_ms);
    CarFirmware_HandleGrayCalibrationButton(firmware, now_ms);

    if (CarPeriodicTask_Due(&firmware->imu_task, now_ms)) {
        CarFirmware_RunImu(firmware, now_ms);
    }
    if (CarPeriodicTask_Due(&firmware->gray_task, now_ms)) {
        if (GrayArray_Read(&firmware->gray, now_ms)) {
            CarFirmware_AccumulateGrayCalibration(firmware, now_ms);
            if (!GrayArray_GetLatest(&firmware->gray,
                                     &firmware->gray_sample)) {
                firmware->gray_sample.valid = false;
            }
        }
    }
    if (CarPeriodicTask_Due(&firmware->control_task, now_ms)) {
        CarFirmware_RunControl(firmware, now_ms);
    }
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
    firmware->motor_prepare_active = false;
    firmware->motor_prepare_step = CAR_MOTOR_PREP_IDLE;
    (void)MotorBoard_EmergencyStop(&firmware->motor);
    firmware->motor_armed = false;
}

const CarOutputSnapshot *CarFirmware_GetOutput(const CarFirmware *firmware)
{
    return (firmware == 0) ? 0 : &firmware->output;
}
