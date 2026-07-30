#include "ball_observation_protocol.h"

#include <string.h>

#define BALL_OBSERVATION_SOF_0 0xA5U
#define BALL_OBSERVATION_SOF_1 0x5AU
#define BALL_OBSERVATION_BODY_OFFSET 2U
#define BALL_OBSERVATION_BODY_SIZE 16U
#define BALL_OBSERVATION_CRC_OFFSET 18U

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
    uint8_t index;

    for (index = 0U; index < length; ++index)
    {
        uint8_t bit;

        crc ^= data[index];
        for (bit = 0U; bit < 8U; ++bit)
        {
            crc = ((crc & 1U) != 0U) ?
                (uint16_t)((crc >> 1U) ^ 0xA001U) :
                (uint16_t)(crc >> 1U);
        }
    }

    return crc;
}

static void restart_after_invalid_frame(BallObservationParser *parser)
{
    uint8_t index;

    for (index = 1U;
         (uint8_t)(index + 1U) < parser->buffer_length;
         ++index)
    {
        if ((parser->buffer[index] == BALL_OBSERVATION_SOF_0) &&
            (parser->buffer[index + 1U] == BALL_OBSERVATION_SOF_1))
        {
            uint8_t retained_length =
                (uint8_t)(parser->buffer_length - index);

            memmove(parser->buffer, &parser->buffer[index], retained_length);
            parser->buffer_length = retained_length;
            parser->resynchronization_count++;
            return;
        }
    }

    if (parser->buffer[parser->buffer_length - 1U] == BALL_OBSERVATION_SOF_0)
    {
        parser->buffer[0] = BALL_OBSERVATION_SOF_0;
        parser->buffer_length = 1U;
    }
    else
    {
        parser->buffer_length = 0U;
    }
    parser->resynchronization_count++;
}

static bool decode_frame(
    BallObservationParser *parser,
    uint32_t rx_complete_ms,
    BallObservation *observation)
{
    const uint8_t *frame = parser->buffer;
    uint8_t valid;
    uint8_t reason;
    int16_t position;
    uint16_t confidence;
    uint16_t capture_age;

    if (read_le16(&frame[BALL_OBSERVATION_CRC_OFFSET]) !=
        crc16_modbus(&frame[BALL_OBSERVATION_BODY_OFFSET],
                      BALL_OBSERVATION_BODY_SIZE))
    {
        parser->crc_error_count++;
        return false;
    }

    if ((frame[2] != BALL_OBSERVATION_PROTOCOL_VERSION) ||
        (frame[3] != BALL_OBSERVATION_MESSAGE_TYPE) ||
        (frame[5] != BALL_OBSERVATION_PAYLOAD_SIZE))
    {
        parser->format_error_count++;
        return false;
    }

    valid = frame[10];
    reason = frame[11];
    position = (int16_t)read_le16(&frame[12]);
    confidence = read_le16(&frame[14]);
    capture_age = read_le16(&frame[16]);

    if ((valid > 1U) ||
        (reason > (uint8_t)BALL_OBSERVATION_REASON_WARMUP) ||
        (confidence > 1000U) ||
        ((valid != 0U) &&
         ((reason != (uint8_t)BALL_OBSERVATION_REASON_OK) ||
          (position == BALL_OBSERVATION_INVALID_POSITION) ||
          (capture_age == BALL_OBSERVATION_UNKNOWN_CAPTURE_AGE_MS))) ||
        ((valid == 0U) &&
         ((reason == (uint8_t)BALL_OBSERVATION_REASON_OK) ||
          (position != BALL_OBSERVATION_INVALID_POSITION))))
    {
        parser->semantic_error_count++;
        return false;
    }

    observation->tx_uptime_ms = read_le32(&frame[6]);
    observation->rx_complete_ms = rx_complete_ms;
    observation->x_0_1mm = position;
    observation->confidence_permille = confidence;
    observation->capture_age_ms = capture_age;
    observation->sequence = frame[4];
    observation->reason = (BallObservationReason)reason;
    observation->valid = valid != 0U;
    return true;
}

void BallObservationParser_Init(BallObservationParser *parser)
{
    if (parser != NULL)
    {
        memset(parser, 0, sizeof(*parser));
    }
}

bool BallObservationParser_OnByte(
    BallObservationParser *parser,
    uint8_t byte,
    uint32_t rx_complete_ms,
    BallObservation *observation)
{
    bool decoded;

    if ((parser == NULL) || (observation == NULL))
    {
        return false;
    }

    if (parser->buffer_length == 0U)
    {
        if (byte == BALL_OBSERVATION_SOF_0)
        {
            parser->buffer[0] = byte;
            parser->buffer_length = 1U;
        }
        return false;
    }

    if (parser->buffer_length == 1U)
    {
        if (byte == BALL_OBSERVATION_SOF_1)
        {
            parser->buffer[1] = byte;
            parser->buffer_length = 2U;
        }
        else if (byte == BALL_OBSERVATION_SOF_0)
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
    if (parser->buffer_length != BALL_OBSERVATION_FRAME_SIZE)
    {
        return false;
    }

    decoded = decode_frame(parser, rx_complete_ms, observation);
    if (decoded)
    {
        parser->accepted_frame_count++;
        parser->buffer_length = 0U;
        return true;
    }

    restart_after_invalid_frame(parser);
    return false;
}
