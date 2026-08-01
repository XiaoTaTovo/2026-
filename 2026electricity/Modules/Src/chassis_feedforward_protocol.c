#include "chassis_feedforward_protocol.h"

#include <string.h>

#define CHASSIS_FEEDFORWARD_SOF_0 0xA5U
#define CHASSIS_FEEDFORWARD_SOF_1 0x5AU
#define CHASSIS_FEEDFORWARD_BODY_OFFSET 2U
#define CHASSIS_FEEDFORWARD_BODY_SIZE 18U
#define CHASSIS_FEEDFORWARD_CRC_OFFSET 20U
#define CHASSIS_FEEDFORWARD_IMU_RESIDUAL_ALPHA 0.15f
#define CHASSIS_FEEDFORWARD_MM_S2_PER_G 9806.65f

static uint16_t read_le16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static uint32_t read_le32(const uint8_t *data)
{
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8U) |
           ((uint32_t)data[2] << 16U) |
           ((uint32_t)data[3] << 24U);
}

static uint16_t crc16_modbus(const uint8_t *data, uint8_t length)
{
    uint16_t crc = 0xFFFFU;

    for (uint8_t index = 0U; index < length; ++index)
    {
        crc ^= data[index];
        for (uint8_t bit = 0U; bit < 8U; ++bit)
        {
            crc = ((crc & 1U) != 0U) ?
                (uint16_t)((crc >> 1U) ^ 0xA001U) :
                (uint16_t)(crc >> 1U);
        }
    }
    return crc;
}

static void restart_after_invalid_frame(ChassisFeedforwardParser *parser)
{
    for (uint8_t index = 1U;
         (uint8_t)(index + 1U) < parser->buffer_length;
         ++index)
    {
        if ((parser->buffer[index] == CHASSIS_FEEDFORWARD_SOF_0) &&
            (parser->buffer[index + 1U] == CHASSIS_FEEDFORWARD_SOF_1))
        {
            uint8_t retained_length =
                (uint8_t)(parser->buffer_length - index);

            memmove(parser->buffer, &parser->buffer[index], retained_length);
            parser->buffer_length = retained_length;
            parser->resynchronization_count++;
            return;
        }
    }
    parser->buffer_length =
        (parser->buffer[parser->buffer_length - 1U] ==
         CHASSIS_FEEDFORWARD_SOF_0) ? 1U : 0U;
    if (parser->buffer_length == 1U)
    {
        parser->buffer[0] = CHASSIS_FEEDFORWARD_SOF_0;
    }
    parser->resynchronization_count++;
}

static bool decode_frame(ChassisFeedforwardParser *parser,
                         uint32_t received_ms,
                         ChassisFeedforwardSample *sample)
{
    const uint8_t *frame = parser->buffer;

    if (read_le16(&frame[CHASSIS_FEEDFORWARD_CRC_OFFSET]) !=
        crc16_modbus(&frame[CHASSIS_FEEDFORWARD_BODY_OFFSET],
                     CHASSIS_FEEDFORWARD_BODY_SIZE))
    {
        parser->crc_error_count++;
        return false;
    }
    if ((frame[2] != CHASSIS_FEEDFORWARD_PROTOCOL_VERSION) ||
        (frame[3] != CHASSIS_FEEDFORWARD_MESSAGE_STATE))
    {
        parser->format_error_count++;
        return false;
    }
    *sample = (ChassisFeedforwardSample){
        .sequence = read_le16(&frame[4]),
        .source_timestamp_ms = read_le32(&frame[6]),
        .received_ms = received_ms,
        .flags = read_le16(&frame[10]),
        .center_speed_0p1_mm_s = (int16_t)read_le16(&frame[12]),
        .command_accel_mm_s2 = (int16_t)read_le16(&frame[14]),
        .imu_accel_mm_s2 = (int16_t)read_le16(&frame[16]),
        .yaw_rate_0p1_dps = (int16_t)read_le16(&frame[18]),
        .valid = true
    };
    return true;
}

void ChassisFeedforwardParser_Init(ChassisFeedforwardParser *parser)
{
    if (parser != NULL)
    {
        memset(parser, 0, sizeof(*parser));
    }
}

bool ChassisFeedforwardParser_OnByte(
    ChassisFeedforwardParser *parser,
    uint8_t byte,
    uint32_t received_ms,
    ChassisFeedforwardSample *sample)
{
    bool decoded;

    if ((parser == NULL) || (sample == NULL))
    {
        return false;
    }
    if (parser->buffer_length == 0U)
    {
        if (byte == CHASSIS_FEEDFORWARD_SOF_0)
        {
            parser->buffer[0] = byte;
            parser->buffer_length = 1U;
        }
        return false;
    }
    if (parser->buffer_length == 1U)
    {
        if (byte == CHASSIS_FEEDFORWARD_SOF_1)
        {
            parser->buffer[1] = byte;
            parser->buffer_length = 2U;
        }
        else if (byte == CHASSIS_FEEDFORWARD_SOF_0)
        {
            parser->buffer[0] = byte;
        }
        else
        {
            parser->buffer_length = 0U;
        }
        return false;
    }
    parser->buffer[parser->buffer_length++] = byte;
    if (parser->buffer_length != CHASSIS_FEEDFORWARD_FRAME_SIZE)
    {
        return false;
    }
    decoded = decode_frame(parser, received_ms, sample);
    if (decoded)
    {
        parser->accepted_frame_count++;
        parser->buffer_length = 0U;
        return true;
    }
    restart_after_invalid_frame(parser);
    return false;
}

bool ChassisFeedforward_IsFresh(
    const ChassisFeedforwardSample *sample,
    uint32_t now_ms)
{
    return (sample != NULL) && sample->valid &&
           ((uint32_t)(now_ms - sample->received_ms) <=
            CHASSIS_FEEDFORWARD_FRESH_TIMEOUT_MS);
}

void ChassisFeedforwardEstimate_Update(
    ChassisFeedforwardEstimate *estimate,
    const ChassisFeedforwardSample *sample)
{
    float command_accel;
    float imu_accel;

    if ((estimate == NULL) || (sample == NULL) || !sample->valid)
    {
        return;
    }
    command_accel = (float)sample->command_accel_mm_s2;
    imu_accel = (float)sample->imu_accel_mm_s2;
    estimate->estimated_accel_mm_s2 = command_accel;
    if ((sample->flags & CHASSIS_FEEDFORWARD_FLAG_IMU_ACCEL_VALID) != 0U)
    {
        estimate->estimated_accel_mm_s2 +=
            CHASSIS_FEEDFORWARD_IMU_RESIDUAL_ALPHA *
            (imu_accel - command_accel);
    }
    estimate->pitch_candidate_mrad = estimate->estimated_accel_mm_s2 *
        1000.0f / CHASSIS_FEEDFORWARD_MM_S2_PER_G;
    estimate->update_count++;
    estimate->valid = true;
}
