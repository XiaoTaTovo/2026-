#ifndef FIRMWARE_H
#define FIRMWARE_H

#include <stdbool.h>
#include <stdint.h>

#include "app/car_app.h"
#include "core/scheduler.h"
#include "core/yaw_estimator.h"
#include "drivers/button.h"
#include "drivers/buzzer.h"
#include "drivers/drive_port.h"
#include "drivers/gray_array.h"
#include "drivers/icm42688.h"
#include "drivers/red_array.h"

typedef enum {
    CAR_TRACK_SENSOR_GRAY_ARRAY = 0,
    CAR_TRACK_SENSOR_RED_ARRAY
} CarTrackSensorSource;

typedef enum {
    CAR_IMU_AXIS_X = 0,
    CAR_IMU_AXIS_Y,
    CAR_IMU_AXIS_Z
} CarImuAxis;

typedef enum {
    CAR_BUTTON_ACTION_NONE = 0,
    CAR_BUTTON_ACTION_ARM_OK,
    CAR_BUTTON_ACTION_ARM_REJECTED,
    CAR_BUTTON_ACTION_GRAY_REQUIRED,
    CAR_BUTTON_ACTION_EMERGENCY_STOP
} CarButtonAction;

typedef enum {
    CAR_GRAY_CAL_WAIT_WHITE = 0,
    CAR_GRAY_CAL_CAPTURE_WHITE,
    CAR_GRAY_CAL_WAIT_BLACK,
    CAR_GRAY_CAL_CAPTURE_BLACK,
    CAR_GRAY_CAL_READY,
    CAR_GRAY_CAL_ERROR
} CarGrayCalibrationState;

#define CAR_GRAY_CALIBRATION_FRAMES (16U)

typedef struct {
    CarConfig car;
    H2026Mode mode;
    CarDrivePort drive;
    Icm42688Port imu;
    GrayArrayPort gray;
    RedArrayPort red;
    CarTrackSensorSource track_sensor_source;
    ButtonReadFn button_read;
    void *button_context;
    ButtonReadFn gray_cal_button_read;
    void *gray_cal_button_context;
    ButtonReadFn task_button_read;
    void *task_button_context;
    bool require_runtime_gray_calibration;
    BuzzerSetFn buzzer_set;
    void *buzzer_context;

    CarImuAxis yaw_axis;
    int8_t yaw_sign;
    float yaw_bias_dps;
    bool yaw_bias_fixed;
    uint16_t imu_calibration_samples;
    uint32_t imu_max_step_ms;
    uint16_t button_debounce_ms;
    bool button_active_low;

    bool gray_calibration_valid;
    uint16_t gray_black[GRAY_ARRAY_CHANNELS];
    uint16_t gray_white[GRAY_ARRAY_CHANNELS];
    bool red_calibration_valid;
    uint16_t red_black[RED_ARRAY_CHANNELS];
    uint16_t red_white[RED_ARRAY_CHANNELS];
} CarFirmwareConfig;

typedef struct {
    CarFirmwareConfig config;
    Icm42688 imu;
    GrayArray gray;
    RedArray red;
    Button button;
    Button gray_cal_button;
    Button task_button;
    Buzzer buzzer;
    CarYawEstimator yaw;
    CarApp app;

    CarPeriodicTask imu_task;
    CarPeriodicTask gray_task;
    CarPeriodicTask control_task;
    CarPeriodicTask stop_refresh_task;

    CarImuSample imu_sample;
    CarGraySample gray_sample;
    CarOutputSnapshot output;
    CarCue last_output_cue;
    uint32_t hardware_faults;
    uint32_t last_tick_ms;
    uint32_t button_event_count;
    uint32_t task_switch_count;
    uint32_t task_switch_reject_count;
    uint32_t last_button_event_ms;
    CarButtonAction last_button_action;
    CarStatus last_arm_status;
    bool last_button_encoder_valid;
    bool last_button_imu_valid;
    bool last_button_drive_ready;
    bool encoder_valid_current;
    bool drive_active;

    CarGrayCalibrationState gray_cal_state;
    uint32_t gray_cal_sum[GRAY_ARRAY_CHANNELS];
    uint16_t gray_cal_white[GRAY_ARRAY_CHANNELS];
    uint16_t gray_cal_black[GRAY_ARRAY_CHANNELS];
    uint8_t gray_cal_frame_count;
    uint8_t gray_cal_bad_channel;
    int16_t gray_cal_bad_span;
    bool initialized;
} CarFirmware;

CarStatus CarFirmware_Init(CarFirmware *firmware,
                           const CarFirmwareConfig *config,
                           uint32_t now_ms);
void CarFirmware_Tick(CarFirmware *firmware, uint32_t now_ms);
void CarFirmware_ForceStop(CarFirmware *firmware, uint32_t fault);
CarStatus CarFirmware_SelectNextTask(CarFirmware *firmware);
CarStatus CarFirmware_CompleteExternalTask(CarFirmware *firmware,
                                           uint32_t now_ms);
const CarOutputSnapshot *CarFirmware_GetOutput(const CarFirmware *firmware);

#endif
