#ifndef CHASSIS_FEEDFORWARD_PROTOCOL_H
#define CHASSIS_FEEDFORWARD_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#define CHASSIS_FEEDFORWARD_PROTOCOL_VERSION 1U
#define CHASSIS_FEEDFORWARD_MESSAGE_STATE 0x31U
#define CHASSIS_FEEDFORWARD_FRAME_SIZE 22U
#define CHASSIS_FEEDFORWARD_FRESH_TIMEOUT_MS 150U

enum
{
    CHASSIS_FEEDFORWARD_FLAG_ROUTE_RUNNING = 1U << 0,
    CHASSIS_FEEDFORWARD_FLAG_COMMAND_VALID = 1U << 1,
    CHASSIS_FEEDFORWARD_FLAG_IMU_ACCEL_VALID = 1U << 2,
    CHASSIS_FEEDFORWARD_FLAG_IMU_READ_VALID = 1U << 3,
    CHASSIS_FEEDFORWARD_FLAG_FAULT = 1U << 4
};

typedef struct
{
    uint16_t sequence;
    uint32_t source_timestamp_ms;
    uint32_t received_ms;
    uint16_t flags;
    int16_t center_speed_0p1_mm_s;
    int16_t command_accel_mm_s2;
    int16_t imu_accel_mm_s2;
    int16_t yaw_rate_0p1_dps;
    bool valid;
} ChassisFeedforwardSample;

typedef struct
{
    uint8_t buffer[CHASSIS_FEEDFORWARD_FRAME_SIZE];
    uint8_t buffer_length;
    uint32_t accepted_frame_count;
    uint32_t crc_error_count;
    uint32_t format_error_count;
    uint32_t resynchronization_count;
} ChassisFeedforwardParser;

typedef struct
{
    float estimated_accel_mm_s2;
    float pitch_candidate_mrad;
    uint32_t update_count;
    bool valid;
} ChassisFeedforwardEstimate;

void ChassisFeedforwardParser_Init(ChassisFeedforwardParser *parser);
bool ChassisFeedforwardParser_OnByte(
    ChassisFeedforwardParser *parser,
    uint8_t byte,
    uint32_t received_ms,
    ChassisFeedforwardSample *sample);
bool ChassisFeedforward_IsFresh(const ChassisFeedforwardSample *sample,
                                uint32_t now_ms);
void ChassisFeedforwardEstimate_Update(
    ChassisFeedforwardEstimate *estimate,
    const ChassisFeedforwardSample *sample);

#endif
