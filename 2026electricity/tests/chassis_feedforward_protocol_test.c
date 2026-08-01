#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "chassis_feedforward_protocol.h"

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

static void write_le16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8U);
}

static void write_le32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8U);
    data[2] = (uint8_t)(value >> 16U);
    data[3] = (uint8_t)(value >> 24U);
}

static void make_frame(uint8_t frame[CHASSIS_FEEDFORWARD_FRAME_SIZE])
{
    uint16_t crc;

    frame[0] = 0xA5U;
    frame[1] = 0x5AU;
    frame[2] = CHASSIS_FEEDFORWARD_PROTOCOL_VERSION;
    frame[3] = CHASSIS_FEEDFORWARD_MESSAGE_STATE;
    write_le16(&frame[4], 0x1234U);
    write_le32(&frame[6], 0x10203040UL);
    write_le16(&frame[10],
               CHASSIS_FEEDFORWARD_FLAG_COMMAND_VALID |
               CHASSIS_FEEDFORWARD_FLAG_IMU_ACCEL_VALID);
    write_le16(&frame[12], (uint16_t)123);
    write_le16(&frame[14], (uint16_t)(int16_t)-800);
    write_le16(&frame[16], (uint16_t)(int16_t)-600);
    write_le16(&frame[18], (uint16_t)(int16_t)-45);
    crc = crc16_modbus(&frame[2], 18U);
    write_le16(&frame[20], crc);
}

static bool feed(
    ChassisFeedforwardParser *parser,
    const uint8_t *data,
    size_t length,
    uint32_t received_ms,
    ChassisFeedforwardSample *sample)
{
    bool decoded = false;
    size_t index;

    for (index = 0U; index < length; ++index)
    {
        if (ChassisFeedforwardParser_OnByte(
                parser, data[index], received_ms, sample))
        {
            decoded = true;
        }
    }
    return decoded;
}

static void test_decode_freshness_and_estimation(void)
{
    uint8_t frame[CHASSIS_FEEDFORWARD_FRAME_SIZE];
    ChassisFeedforwardParser parser;
    ChassisFeedforwardSample sample;
    ChassisFeedforwardEstimate estimate = {0};

    make_frame(frame);
    ChassisFeedforwardParser_Init(&parser);
    assert(feed(&parser, frame, sizeof(frame), 1000U, &sample));
    assert(parser.accepted_frame_count == 1U);
    assert(sample.sequence == 0x1234U);
    assert(sample.source_timestamp_ms == 0x10203040UL);
    assert(sample.command_accel_mm_s2 == -800);
    assert(sample.imu_accel_mm_s2 == -600);
    assert(sample.yaw_rate_0p1_dps == -45);
    assert(ChassisFeedforward_IsFresh(&sample, 1150U));
    assert(!ChassisFeedforward_IsFresh(&sample, 1151U));
    ChassisFeedforwardEstimate_Update(&estimate, &sample);
    assert(estimate.valid);
    assert(estimate.update_count == 1U);
    assert(estimate.estimated_accel_mm_s2 > -771.0f);
    assert(estimate.estimated_accel_mm_s2 < -769.0f);
}

static void test_corrupt_frame_resynchronizes(void)
{
    uint8_t frame[CHASSIS_FEEDFORWARD_FRAME_SIZE];
    uint8_t stream[CHASSIS_FEEDFORWARD_FRAME_SIZE * 2U];
    ChassisFeedforwardParser parser;
    ChassisFeedforwardSample sample;

    make_frame(frame);
    frame[19] ^= 0x80U;
    for (size_t index = 0U; index < sizeof(frame); ++index)
    {
        stream[index] = frame[index];
    }
    make_frame(&stream[CHASSIS_FEEDFORWARD_FRAME_SIZE]);
    ChassisFeedforwardParser_Init(&parser);
    assert(feed(&parser, stream, sizeof(stream), 1U, &sample));
    assert(parser.crc_error_count == 1U);
    assert(parser.accepted_frame_count == 1U);
    assert(sample.sequence == 0x1234U);
}

int main(void)
{
    test_decode_freshness_and_estimation();
    test_corrupt_frame_resynchronizes();
    puts("CHASSIS_FEEDFORWARD_PROTOCOL_TEST=PASS");
    return 0;
}
