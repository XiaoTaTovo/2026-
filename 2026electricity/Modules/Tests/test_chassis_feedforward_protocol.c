#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "chassis_feedforward_protocol.h"

static uint16_t Crc16Modbus(const uint8_t *data, uint8_t length)
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

static void WriteLe16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8U);
}

static void WriteLe32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8U);
    data[2] = (uint8_t)(value >> 16U);
    data[3] = (uint8_t)(value >> 24U);
}

int main(void)
{
    ChassisFeedforwardParser parser;
    ChassisFeedforwardSample sample = {0};
    ChassisFeedforwardEstimate estimate = {0};
    uint8_t frame[CHASSIS_FEEDFORWARD_FRAME_SIZE] = {0};

    frame[0] = 0xA5U;
    frame[1] = 0x5AU;
    frame[2] = CHASSIS_FEEDFORWARD_PROTOCOL_VERSION;
    frame[3] = CHASSIS_FEEDFORWARD_MESSAGE_STATE;
    WriteLe16(&frame[4], 0x1234U);
    WriteLe32(&frame[6], 0x87654321U);
    WriteLe16(&frame[10], CHASSIS_FEEDFORWARD_FLAG_COMMAND_VALID);
    WriteLe16(&frame[12], 3500U);
    WriteLe16(&frame[14], 2000U);
    WriteLe16(&frame[16], (uint16_t)-300);
    WriteLe16(&frame[18], (uint16_t)-25);
    WriteLe16(&frame[20], Crc16Modbus(&frame[2], 18U));

    ChassisFeedforwardParser_Init(&parser);
    for (uint8_t index = 0U; index < sizeof(frame); ++index)
    {
        bool complete = ChassisFeedforwardParser_OnByte(
            &parser, frame[index], 100U, &sample);

        assert(complete == (index == (sizeof(frame) - 1U)));
    }
    assert(sample.valid);
    assert(sample.sequence == 0x1234U);
    assert(sample.source_timestamp_ms == 0x87654321U);
    assert(sample.center_speed_0p1_mm_s == 3500);
    assert(sample.command_accel_mm_s2 == 2000);
    assert(sample.imu_accel_mm_s2 == -300);
    assert(sample.yaw_rate_0p1_dps == -25);
    assert(ChassisFeedforward_IsFresh(&sample, 250U));
    assert(!ChassisFeedforward_IsFresh(&sample, 251U));
    ChassisFeedforwardEstimate_Update(&estimate, &sample);
    assert(estimate.valid);
    assert(estimate.estimated_accel_mm_s2 == 2000.0f);

    frame[20] ^= 0x01U;
    ChassisFeedforwardParser_Init(&parser);
    for (uint8_t index = 0U; index < sizeof(frame); ++index)
    {
        assert(!ChassisFeedforwardParser_OnByte(
            &parser, frame[index], 100U, &sample));
    }
    assert(parser.crc_error_count == 1U);
    puts("chassis feedforward protocol tests passed");
    return 0;
}
