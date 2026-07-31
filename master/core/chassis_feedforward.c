#include "core/chassis_feedforward.h"

#define CHASSIS_FEEDFORWARD_MM_S2_PER_G (9806.65f)
#define CHASSIS_FEEDFORWARD_DEFAULT_LSB_PER_G (2048.0f)
#define CHASSIS_FEEDFORWARD_DEFAULT_FILTER_ALPHA (0.15f)

static float ChassisFeedforward_Clamp(float value, float low, float high)
{
    if (value < low) {
        return low;
    }
    if (value > high) {
        return high;
    }
    return value;
}

static int16_t ChassisFeedforward_RoundI16(float value)
{
    value = ChassisFeedforward_Clamp(value, -32768.0f, 32767.0f);
    return (int16_t)(value + ((value >= 0.0f) ? 0.5f : -0.5f));
}

static int16_t ChassisFeedforward_SelectAccel(
    const ChassisFeedforward *state,
    const Icm42688Sample *raw)
{
    switch (state->config.accel_axis) {
        case CHASSIS_FEEDFORWARD_AXIS_Y:
            return raw->accel_y;
        case CHASSIS_FEEDFORWARD_AXIS_Z:
            return raw->accel_z;
        case CHASSIS_FEEDFORWARD_AXIS_X:
        default:
            return raw->accel_x;
    }
}

static float ChassisFeedforward_RawToMmS2(
    const ChassisFeedforward *state,
    int16_t raw)
{
    return ((float)raw / state->config.accel_lsb_per_g) *
           CHASSIS_FEEDFORWARD_MM_S2_PER_G *
           (float)state->config.accel_sign;
}

static void ChassisFeedforward_WriteLe16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8U);
}

static void ChassisFeedforward_WriteLe32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8U);
    data[2] = (uint8_t)(value >> 16U);
    data[3] = (uint8_t)(value >> 24U);
}

void ChassisFeedforward_Init(ChassisFeedforward *state,
                             const ChassisFeedforwardConfig *config)
{
    if (state == 0) {
        return;
    }
    *state = (ChassisFeedforward){0};
    if (config != 0) {
        state->config = *config;
    }
    if ((state->config.accel_sign != 1) && (state->config.accel_sign != -1)) {
        state->config.accel_sign = 1;
    }
    if (state->config.accel_lsb_per_g <= 0.0f) {
        state->config.accel_lsb_per_g = CHASSIS_FEEDFORWARD_DEFAULT_LSB_PER_G;
    }
    if ((state->config.imu_filter_alpha <= 0.0f) ||
        (state->config.imu_filter_alpha > 1.0f)) {
        state->config.imu_filter_alpha = CHASSIS_FEEDFORWARD_DEFAULT_FILTER_ALPHA;
    }
    state->stationary_calibrated =
        state->config.stationary_calibration_samples == 0U;
}

void ChassisFeedforward_OnImuSample(ChassisFeedforward *state,
                                    const Icm42688Sample *raw)
{
    float raw_mm_s2;
    float corrected_mm_s2;

    if ((state == 0) || (raw == 0) || !raw->valid) {
        ChassisFeedforward_OnImuReadError(state);
        return;
    }
    raw_mm_s2 = ChassisFeedforward_RawToMmS2(
        state, ChassisFeedforward_SelectAccel(state, raw));
    state->imu_read_valid = true;
    if (!state->stationary_calibrated) {
        state->stationary_sum_mm_s2 += raw_mm_s2;
        state->stationary_count++;
        if (state->stationary_count >=
            state->config.stationary_calibration_samples) {
            state->accel_bias_mm_s2 = state->stationary_sum_mm_s2 /
                (float)state->stationary_count;
            state->accel_filtered_mm_s2 = 0.0f;
            state->stationary_calibrated = true;
        }
        return;
    }
    corrected_mm_s2 = raw_mm_s2 - state->accel_bias_mm_s2;
    state->accel_filtered_mm_s2 += state->config.imu_filter_alpha *
        (corrected_mm_s2 - state->accel_filtered_mm_s2);
}

void ChassisFeedforward_OnImuReadError(ChassisFeedforward *state)
{
    if (state == 0) {
        return;
    }
    state->imu_read_valid = false;
    state->imu_error_count++;
}

void ChassisFeedforward_OnCommand(ChassisFeedforward *state,
                                  float center_speed_mm_s,
                                  uint32_t timestamp_ms)
{
    if (state == 0) {
        return;
    }
    if (!state->command_initialized) {
        state->previous_center_speed_mm_s = center_speed_mm_s;
        state->previous_command_ms = timestamp_ms;
        state->command_accel_mm_s2 = 0.0f;
        state->command_initialized = true;
        return;
    }
    if (timestamp_ms != state->previous_command_ms) {
        uint32_t dt_ms = timestamp_ms - state->previous_command_ms;

        if (dt_ms != 0U) {
            state->command_accel_mm_s2 =
                (center_speed_mm_s - state->previous_center_speed_mm_s) *
                1000.0f / (float)dt_ms;
        }
    }
    state->previous_center_speed_mm_s = center_speed_mm_s;
    state->previous_command_ms = timestamp_ms;
}

bool ChassisFeedforward_GetSample(const ChassisFeedforward *state,
                                  uint32_t timestamp_ms,
                                  float yaw_rate_dps,
                                  ChassisFeedforwardSample *sample)
{
    if ((state == 0) || (sample == 0) || !state->command_initialized) {
        return false;
    }
    *sample = (ChassisFeedforwardSample){
        .timestamp_ms = timestamp_ms,
        .center_speed_0p1_mm_s = ChassisFeedforward_RoundI16(
            state->previous_center_speed_mm_s * 10.0f),
        .command_accel_mm_s2 = ChassisFeedforward_RoundI16(
            state->command_accel_mm_s2),
        .imu_accel_mm_s2 = ChassisFeedforward_RoundI16(
            state->accel_filtered_mm_s2),
        .yaw_rate_0p1_dps = ChassisFeedforward_RoundI16(yaw_rate_dps * 10.0f),
        .command_valid = true,
        .imu_accel_valid = state->stationary_calibrated && state->imu_read_valid,
        .imu_read_valid = state->imu_read_valid
    };
    return true;
}

uint16_t ChassisFeedforward_MakeFlags(const ChassisFeedforwardSample *sample,
                                      bool route_running,
                                      bool fault_active)
{
    uint16_t flags = 0U;

    if (sample == 0) {
        return fault_active ? CHASSIS_FEEDFORWARD_FLAG_FAULT : 0U;
    }
    if (route_running) {
        flags |= CHASSIS_FEEDFORWARD_FLAG_ROUTE_RUNNING;
    }
    if (sample->command_valid) {
        flags |= CHASSIS_FEEDFORWARD_FLAG_COMMAND_VALID;
    }
    if (sample->imu_accel_valid) {
        flags |= CHASSIS_FEEDFORWARD_FLAG_IMU_ACCEL_VALID;
    }
    if (sample->imu_read_valid) {
        flags |= CHASSIS_FEEDFORWARD_FLAG_IMU_READ_VALID;
    }
    if (fault_active) {
        flags |= CHASSIS_FEEDFORWARD_FLAG_FAULT;
    }
    return flags;
}

uint16_t ChassisFeedforward_Crc16Modbus(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFFU;

    if (data == 0) {
        return 0U;
    }
    for (size_t index = 0U; index < length; ++index) {
        crc ^= data[index];
        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            crc = ((crc & 1U) != 0U) ? (uint16_t)((crc >> 1U) ^ 0xA001U) :
                                        (uint16_t)(crc >> 1U);
        }
    }
    return crc;
}

bool ChassisFeedforward_EncodeState(const ChassisFeedforwardSample *sample,
                                    uint16_t flags,
                                    uint16_t sequence,
                                    uint8_t frame[
                                        CHASSIS_FEEDFORWARD_FRAME_SIZE])
{
    uint16_t crc;

    if ((sample == 0) || (frame == 0)) {
        return false;
    }
    frame[0] = 0xA5U;
    frame[1] = 0x5AU;
    frame[2] = CHASSIS_FEEDFORWARD_PROTOCOL_VERSION;
    frame[3] = CHASSIS_FEEDFORWARD_MESSAGE_STATE;
    ChassisFeedforward_WriteLe16(&frame[4], sequence);
    ChassisFeedforward_WriteLe32(&frame[6], sample->timestamp_ms);
    ChassisFeedforward_WriteLe16(&frame[10], flags);
    ChassisFeedforward_WriteLe16(&frame[12],
                                 (uint16_t)sample->center_speed_0p1_mm_s);
    ChassisFeedforward_WriteLe16(&frame[14],
                                 (uint16_t)sample->command_accel_mm_s2);
    ChassisFeedforward_WriteLe16(&frame[16],
                                 (uint16_t)sample->imu_accel_mm_s2);
    ChassisFeedforward_WriteLe16(&frame[18],
                                 (uint16_t)sample->yaw_rate_0p1_dps);
    crc = ChassisFeedforward_Crc16Modbus(&frame[2], 18U);
    ChassisFeedforward_WriteLe16(&frame[20], crc);
    return true;
}
