#ifndef CAR_TYPES_H
#define CAR_TYPES_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    CAR_OK = 0,
    CAR_ERROR_ARG = -1,
    CAR_ERROR_STATE = -2,
    CAR_ERROR_CAPACITY = -3
} CarStatus;

typedef struct {
    int16_t left_count;
    int16_t right_count;
    uint32_t timestamp_ms;
    bool valid;
} CarEncoderSample;

typedef struct {
    float yaw_deg;
    float yaw_rate_dps;
    uint32_t timestamp_ms;
    bool valid;
} CarImuSample;

typedef struct {
    uint16_t normalized[8];
    uint32_t timestamp_ms;
    bool valid;
} CarGraySample;

typedef struct {
    CarEncoderSample encoder;
    CarImuSample imu;
    CarGraySample gray;
    bool start_pressed;
    bool emergency_stop;
} CarInputSnapshot;

typedef struct {
    float left_mm_s;
    float right_mm_s;
    bool enable;
} CarMotorCommand;

typedef enum {
    CAR_CUE_NONE = 0,
    CAR_CUE_START,
    CAR_CUE_CHECKPOINT,
    CAR_CUE_FINISH,
    CAR_CUE_FAULT
} CarCue;

enum {
    CAR_FAULT_NONE = 0U,
    CAR_FAULT_EMERGENCY_STOP = 1U << 0,
    CAR_FAULT_ENCODER_STALE = 1U << 1,
    CAR_FAULT_IMU_STALE = 1U << 2,
    CAR_FAULT_GRAY_STALE = 1U << 3,
    CAR_FAULT_SEGMENT_TIMEOUT = 1U << 4,
    CAR_FAULT_ROUTE_INVALID = 1U << 5,
    CAR_FAULT_MOTOR_IO = 1U << 6,
    CAR_FAULT_IMU_INIT = 1U << 7,
    CAR_FAULT_GRAY_NOT_CALIBRATED = 1U << 8,
    CAR_FAULT_LINE_MISSED = 1U << 9
};

typedef struct {
    CarMotorCommand motor;
    CarCue cue;
    uint32_t faults;
    uint16_t route_index;
    bool route_running;
    bool route_finished;
    uint32_t run_time_ms;
    uint32_t result_time_ms;
    bool result_valid;
} CarOutputSnapshot;

#endif
