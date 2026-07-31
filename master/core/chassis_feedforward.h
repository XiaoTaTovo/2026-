#ifndef CHASSIS_FEEDFORWARD_H
#define CHASSIS_FEEDFORWARD_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "drivers/icm42688.h"

/* Master -> F407, little-endian, 115200 8N1. The transport is versioned so
 * later status and control extensions cannot reinterpret a running frame. */
#define CHASSIS_FEEDFORWARD_PROTOCOL_VERSION (1U)
#define CHASSIS_FEEDFORWARD_MESSAGE_STATE (0x31U)
#define CHASSIS_FEEDFORWARD_FRAME_SIZE (22U)

enum {
    CHASSIS_FEEDFORWARD_FLAG_ROUTE_RUNNING = 1U << 0,
    CHASSIS_FEEDFORWARD_FLAG_COMMAND_VALID = 1U << 1,
    CHASSIS_FEEDFORWARD_FLAG_IMU_ACCEL_VALID = 1U << 2,
    CHASSIS_FEEDFORWARD_FLAG_IMU_READ_VALID = 1U << 3,
    CHASSIS_FEEDFORWARD_FLAG_FAULT = 1U << 4
};

typedef enum {
    CHASSIS_FEEDFORWARD_AXIS_X = 0,
    CHASSIS_FEEDFORWARD_AXIS_Y,
    CHASSIS_FEEDFORWARD_AXIS_Z
} ChassisFeedforwardAxis;

typedef struct {
    ChassisFeedforwardAxis accel_axis;
    int8_t accel_sign;
    float accel_lsb_per_g;
    uint16_t stationary_calibration_samples;
    float imu_filter_alpha;
} ChassisFeedforwardConfig;

typedef struct {
    uint32_t timestamp_ms;
    int16_t center_speed_0p1_mm_s;
    int16_t command_accel_mm_s2;
    int16_t imu_accel_mm_s2;
    int16_t yaw_rate_0p1_dps;
    bool command_valid;
    bool imu_accel_valid;
    bool imu_read_valid;
} ChassisFeedforwardSample;

typedef struct {
    ChassisFeedforwardConfig config;
    float accel_bias_mm_s2;
    float accel_filtered_mm_s2;
    float previous_center_speed_mm_s;
    float command_accel_mm_s2;
    uint32_t previous_command_ms;
    float stationary_sum_mm_s2;
    uint16_t stationary_count;
    uint32_t imu_error_count;
    bool stationary_calibrated;
    bool command_initialized;
    bool imu_read_valid;
} ChassisFeedforward;

void ChassisFeedforward_Init(ChassisFeedforward *state,
                             const ChassisFeedforwardConfig *config);
void ChassisFeedforward_OnImuSample(ChassisFeedforward *state,
                                    const Icm42688Sample *raw);
void ChassisFeedforward_OnImuReadError(ChassisFeedforward *state);
void ChassisFeedforward_OnCommand(ChassisFeedforward *state,
                                  float center_speed_mm_s,
                                  uint32_t timestamp_ms);
bool ChassisFeedforward_GetSample(const ChassisFeedforward *state,
                                  uint32_t timestamp_ms,
                                  float yaw_rate_dps,
                                  ChassisFeedforwardSample *sample);
uint16_t ChassisFeedforward_MakeFlags(const ChassisFeedforwardSample *sample,
                                      bool route_running,
                                      bool fault_active);
uint16_t ChassisFeedforward_Crc16Modbus(const uint8_t *data, size_t length);
bool ChassisFeedforward_EncodeState(const ChassisFeedforwardSample *sample,
                                    uint16_t flags,
                                    uint16_t sequence,
                                    uint8_t frame[
                                        CHASSIS_FEEDFORWARD_FRAME_SIZE]);

#endif
