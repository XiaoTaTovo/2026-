#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "ball_observation_protocol.h"

static const uint8_t golden_frame[BALL_OBSERVATION_FRAME_SIZE] =
{
    0xA5U, 0x5AU, 0x02U, 0x20U, 0x2AU, 0x08U, 0x56U,
    0x34U, 0x12U, 0x00U, 0x01U, 0x00U, 0xF4U, 0x01U,
    0x6BU, 0x03U, 0x2AU, 0x00U, 0xB3U, 0xB0U
};

static uint16_t crc16_modbus(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFFU;
    size_t index;

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

static void update_crc(uint8_t frame[BALL_OBSERVATION_FRAME_SIZE])
{
    uint16_t crc = crc16_modbus(&frame[2], 16U);

    frame[18] = (uint8_t)crc;
    frame[19] = (uint8_t)(crc >> 8U);
}

static bool feed(
    BallObservationParser *parser,
    const uint8_t *data,
    size_t length,
    uint32_t rx_complete_ms,
    BallObservation *observation)
{
    bool produced = false;
    size_t index;

    for (index = 0U; index < length; ++index)
    {
        if (BallObservationParser_OnByte(
                parser,
                data[index],
                rx_complete_ms,
                observation))
        {
            produced = true;
        }
    }
    return produced;
}

static void test_golden_frame(void)
{
    BallObservationParser parser;
    BallObservation observation;

    BallObservationParser_Init(&parser);
    assert(feed(&parser, golden_frame, sizeof(golden_frame), 1234U,
                &observation));
    assert(parser.accepted_frame_count == 1U);
    assert(observation.tx_uptime_ms == 0x00123456U);
    assert(observation.rx_complete_ms == 1234U);
    assert(observation.sequence == 42U);
    assert(observation.valid);
    assert(observation.reason == BALL_OBSERVATION_REASON_OK);
    assert(observation.x_0_1mm == 500);
    assert(observation.confidence_permille == 875U);
    assert(observation.capture_age_ms == 42U);
}

static void test_invalid_observation_is_delivered_not_reused(void)
{
    uint8_t frame[BALL_OBSERVATION_FRAME_SIZE];
    BallObservationParser parser;
    BallObservation observation;

    memcpy(frame, golden_frame, sizeof(frame));
    frame[4] = 43U;
    frame[10] = 0U;
    frame[11] = BALL_OBSERVATION_REASON_NO_BALL;
    frame[12] = 0x00U;
    frame[13] = 0x80U;
    frame[14] = 0U;
    frame[15] = 0U;
    update_crc(frame);

    BallObservationParser_Init(&parser);
    assert(feed(&parser, frame, sizeof(frame), 200U, &observation));
    assert(!observation.valid);
    assert(observation.reason == BALL_OBSERVATION_REASON_NO_BALL);
    assert(observation.x_0_1mm == BALL_OBSERVATION_INVALID_POSITION);
}

static void test_bad_crc_and_semantics_are_rejected(void)
{
    uint8_t corrupt_crc[BALL_OBSERVATION_FRAME_SIZE];
    uint8_t bad_semantics[BALL_OBSERVATION_FRAME_SIZE];
    BallObservationParser parser;
    BallObservation observation;

    memcpy(corrupt_crc, golden_frame, sizeof(corrupt_crc));
    corrupt_crc[19] ^= 0x80U;
    BallObservationParser_Init(&parser);
    assert(!feed(&parser, corrupt_crc, sizeof(corrupt_crc), 0U,
                 &observation));
    assert(parser.crc_error_count == 1U);

    memcpy(bad_semantics, golden_frame, sizeof(bad_semantics));
    bad_semantics[11] = BALL_OBSERVATION_REASON_NO_BALL;
    update_crc(bad_semantics);
    assert(!feed(&parser, bad_semantics, sizeof(bad_semantics), 0U,
                 &observation));
    assert(parser.semantic_error_count == 1U);
}

static void test_dropped_byte_recovers_at_next_frame(void)
{
    uint8_t stream[BALL_OBSERVATION_FRAME_SIZE * 2U - 1U];
    BallObservationParser parser;
    BallObservation observation;
    size_t index;

    for (index = 0U; index < 10U; ++index)
    {
        stream[index] = golden_frame[index];
    }
    memcpy(&stream[10], golden_frame + 11U,
           BALL_OBSERVATION_FRAME_SIZE - 11U);
    memcpy(&stream[BALL_OBSERVATION_FRAME_SIZE - 1U], golden_frame,
           BALL_OBSERVATION_FRAME_SIZE);

    BallObservationParser_Init(&parser);
    assert(feed(&parser, stream, sizeof(stream), 300U, &observation));
    assert(parser.crc_error_count == 1U);
    assert(parser.resynchronization_count == 1U);
    assert(parser.accepted_frame_count == 1U);
    assert(observation.sequence == 42U);
}

int main(void)
{
    test_golden_frame();
    test_invalid_observation_is_delivered_not_reused();
    test_bad_crc_and_semantics_are_rejected();
    test_dropped_byte_recovers_at_next_frame();
    puts("BALL_OBSERVATION_PROTOCOL_TEST=PASS");
    return 0;
}
