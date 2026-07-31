#include "x42s_protocol.h"

#define X42S_BROADCAST_ADDRESS 0x00U
#define X42S_READ_ADDRESS_FUNCTION 0x15U
#define X42S_READ_STATUS_FUNCTION 0x3AU
#define X42S_READ_POSITION_FUNCTION 0x36U
#define X42S_ENABLE_FUNCTION 0xF3U
#define X42S_ENABLE_AUXILIARY 0xABU
#define X42S_EMM_VELOCITY_FUNCTION 0xF6U
#define X42S_EMM_POSITION_FUNCTION 0xFDU
#define X42S_STOP_FUNCTION 0xFEU
#define X42S_STOP_AUXILIARY 0x98U

/*
 * Source: ZDT_X42S user manual V1.0.5, pages 40, 50, 57 and 60.
 * These functions only build and validate bytes; they never authorize motion.
 */

X42sProtocolResult X42sProtocol_BuildReadAddress(
    uint8_t *frame,
    size_t capacity)
{
    if (frame == NULL)
    {
        return X42S_PROTOCOL_INVALID_ARGUMENT;
    }
    if (capacity < X42S_READ_ADDRESS_REQUEST_SIZE)
    {
        return X42S_PROTOCOL_BUFFER_TOO_SMALL;
    }

    frame[0] = X42S_BROADCAST_ADDRESS;
    frame[1] = X42S_READ_ADDRESS_FUNCTION;
    frame[2] = X42S_FIXED_CHECKSUM;
    return X42S_PROTOCOL_OK;
}

X42sProtocolResult X42sProtocol_ParseReadAddress(
    const uint8_t *frame,
    size_t length,
    uint8_t *address)
{
    if ((frame == NULL) || (address == NULL))
    {
        return X42S_PROTOCOL_INVALID_ARGUMENT;
    }
    if (length != X42S_READ_ADDRESS_RESPONSE_SIZE)
    {
        return X42S_PROTOCOL_INVALID_LENGTH;
    }
    if (frame[0] == X42S_BROADCAST_ADDRESS)
    {
        return X42S_PROTOCOL_INVALID_ADDRESS;
    }
    if (frame[1] != X42S_READ_ADDRESS_FUNCTION)
    {
        return X42S_PROTOCOL_INVALID_FUNCTION;
    }
    if (frame[2] != frame[0])
    {
        return X42S_PROTOCOL_INVALID_ADDRESS_ECHO;
    }
    if (frame[3] != X42S_FIXED_CHECKSUM)
    {
        return X42S_PROTOCOL_INVALID_CHECKSUM;
    }

    *address = frame[0];
    return X42S_PROTOCOL_OK;
}

X42sProtocolResult X42sProtocol_BuildReadStatus(
    uint8_t address,
    uint8_t *frame,
    size_t capacity)
{
    if (frame == NULL)
    {
        return X42S_PROTOCOL_INVALID_ARGUMENT;
    }
    if (capacity < X42S_READ_STATUS_REQUEST_SIZE)
    {
        return X42S_PROTOCOL_BUFFER_TOO_SMALL;
    }
    if (address == X42S_BROADCAST_ADDRESS)
    {
        return X42S_PROTOCOL_INVALID_ADDRESS;
    }

    frame[0] = address;
    frame[1] = X42S_READ_STATUS_FUNCTION;
    frame[2] = X42S_FIXED_CHECKSUM;
    return X42S_PROTOCOL_OK;
}

X42sProtocolResult X42sProtocol_ParseReadStatus(
    const uint8_t *frame,
    size_t length,
    uint8_t expected_address,
    uint8_t *status)
{
    if ((frame == NULL) || (status == NULL))
    {
        return X42S_PROTOCOL_INVALID_ARGUMENT;
    }
    if (length != X42S_READ_STATUS_RESPONSE_SIZE)
    {
        return X42S_PROTOCOL_INVALID_LENGTH;
    }
    if ((expected_address == X42S_BROADCAST_ADDRESS) ||
        (frame[0] != expected_address))
    {
        return X42S_PROTOCOL_INVALID_ADDRESS_ECHO;
    }
    if (frame[1] != X42S_READ_STATUS_FUNCTION)
    {
        return X42S_PROTOCOL_INVALID_FUNCTION;
    }
    if (frame[3] != X42S_FIXED_CHECKSUM)
    {
        return X42S_PROTOCOL_INVALID_CHECKSUM;
    }

    *status = frame[2];
    return X42S_PROTOCOL_OK;
}

X42sProtocolResult X42sProtocol_BuildReadPosition(
    uint8_t address,
    uint8_t *frame,
    size_t capacity)
{
    if (frame == NULL)
    {
        return X42S_PROTOCOL_INVALID_ARGUMENT;
    }
    if (capacity < X42S_READ_POSITION_REQUEST_SIZE)
    {
        return X42S_PROTOCOL_BUFFER_TOO_SMALL;
    }
    if (address == X42S_BROADCAST_ADDRESS)
    {
        return X42S_PROTOCOL_INVALID_ADDRESS;
    }

    frame[0] = address;
    frame[1] = X42S_READ_POSITION_FUNCTION;
    frame[2] = X42S_FIXED_CHECKSUM;
    return X42S_PROTOCOL_OK;
}

X42sProtocolResult X42sProtocol_ParseReadPosition(
    const uint8_t *frame,
    size_t length,
    uint8_t expected_address,
    X42sRawPosition *position)
{
    uint32_t magnitude;

    if ((frame == NULL) || (position == NULL))
    {
        return X42S_PROTOCOL_INVALID_ARGUMENT;
    }
    if (length != X42S_READ_POSITION_RESPONSE_SIZE)
    {
        return X42S_PROTOCOL_INVALID_LENGTH;
    }
    if ((expected_address == X42S_BROADCAST_ADDRESS) ||
        (frame[0] != expected_address))
    {
        return X42S_PROTOCOL_INVALID_ADDRESS_ECHO;
    }
    if (frame[1] != X42S_READ_POSITION_FUNCTION)
    {
        return X42S_PROTOCOL_INVALID_FUNCTION;
    }
    if (frame[2] > 1U)
    {
        return X42S_PROTOCOL_INVALID_SIGN;
    }
    if (frame[7] != X42S_FIXED_CHECKSUM)
    {
        return X42S_PROTOCOL_INVALID_CHECKSUM;
    }

    magnitude = ((uint32_t)frame[3] << 24U) |
                ((uint32_t)frame[4] << 16U) |
                ((uint32_t)frame[5] << 8U) |
                (uint32_t)frame[6];
    position->negative = (frame[2] == 1U);
    position->magnitude = magnitude;
    return X42S_PROTOCOL_OK;
}

X42sProtocolResult X42sProtocol_BuildEmmEnable(
    uint8_t address,
    bool enable,
    bool synchronize,
    uint8_t *frame,
    size_t capacity)
{
    if (frame == NULL)
    {
        return X42S_PROTOCOL_INVALID_ARGUMENT;
    }
    if (capacity < X42S_EMM_ENABLE_REQUEST_SIZE)
    {
        return X42S_PROTOCOL_BUFFER_TOO_SMALL;
    }
    if (address == X42S_BROADCAST_ADDRESS)
    {
        return X42S_PROTOCOL_INVALID_ADDRESS;
    }

    frame[0] = address;
    frame[1] = X42S_ENABLE_FUNCTION;
    frame[2] = X42S_ENABLE_AUXILIARY;
    frame[3] = enable ? 1U : 0U;
    frame[4] = synchronize ? 1U : 0U;
    frame[5] = X42S_FIXED_CHECKSUM;
    return X42S_PROTOCOL_OK;
}

X42sProtocolResult X42sProtocol_BuildEmmVelocity(
    uint8_t address,
    uint8_t direction,
    uint16_t speed_rpm,
    uint8_t acceleration,
    bool synchronize,
    uint8_t *frame,
    size_t capacity)
{
    if (frame == NULL)
    {
        return X42S_PROTOCOL_INVALID_ARGUMENT;
    }
    if (capacity < X42S_EMM_VELOCITY_REQUEST_SIZE)
    {
        return X42S_PROTOCOL_BUFFER_TOO_SMALL;
    }
    if (address == X42S_BROADCAST_ADDRESS)
    {
        return X42S_PROTOCOL_INVALID_ADDRESS;
    }
    if ((direction > 1U) || (speed_rpm > X42S_EMM_MAX_SPEED_RPM))
    {
        return X42S_PROTOCOL_VALUE_OUT_OF_RANGE;
    }

    frame[0] = address;
    frame[1] = X42S_EMM_VELOCITY_FUNCTION;
    frame[2] = direction;
    frame[3] = (uint8_t)(speed_rpm >> 8U);
    frame[4] = (uint8_t)speed_rpm;
    frame[5] = acceleration;
    frame[6] = synchronize ? 1U : 0U;
    frame[7] = X42S_FIXED_CHECKSUM;
    return X42S_PROTOCOL_OK;
}

X42sProtocolResult X42sProtocol_BuildEmmPosition(
    uint8_t address,
    uint8_t direction,
    uint16_t speed_rpm,
    uint8_t acceleration,
    uint32_t pulse_count,
    uint8_t motion_mode,
    bool synchronize,
    uint8_t *frame,
    size_t capacity)
{
    if (frame == NULL)
    {
        return X42S_PROTOCOL_INVALID_ARGUMENT;
    }
    if (capacity < X42S_EMM_POSITION_REQUEST_SIZE)
    {
        return X42S_PROTOCOL_BUFFER_TOO_SMALL;
    }
    if (address == X42S_BROADCAST_ADDRESS)
    {
        return X42S_PROTOCOL_INVALID_ADDRESS;
    }
    if ((direction > 1U) ||
        (speed_rpm > X42S_EMM_MAX_SPEED_RPM) ||
        (motion_mode > 2U))
    {
        return X42S_PROTOCOL_VALUE_OUT_OF_RANGE;
    }

    frame[0] = address;
    frame[1] = X42S_EMM_POSITION_FUNCTION;
    frame[2] = direction;
    frame[3] = (uint8_t)(speed_rpm >> 8U);
    frame[4] = (uint8_t)speed_rpm;
    frame[5] = acceleration;
    frame[6] = (uint8_t)(pulse_count >> 24U);
    frame[7] = (uint8_t)(pulse_count >> 16U);
    frame[8] = (uint8_t)(pulse_count >> 8U);
    frame[9] = (uint8_t)pulse_count;
    frame[10] = motion_mode;
    frame[11] = synchronize ? 1U : 0U;
    frame[12] = X42S_FIXED_CHECKSUM;
    return X42S_PROTOCOL_OK;
}

X42sProtocolResult X42sProtocol_BuildStop(
    uint8_t address,
    bool synchronize,
    uint8_t *frame,
    size_t capacity)
{
    if (frame == NULL)
    {
        return X42S_PROTOCOL_INVALID_ARGUMENT;
    }
    if (capacity < X42S_STOP_REQUEST_SIZE)
    {
        return X42S_PROTOCOL_BUFFER_TOO_SMALL;
    }
    if (address == X42S_BROADCAST_ADDRESS)
    {
        return X42S_PROTOCOL_INVALID_ADDRESS;
    }

    frame[0] = address;
    frame[1] = X42S_STOP_FUNCTION;
    frame[2] = X42S_STOP_AUXILIARY;
    frame[3] = synchronize ? 1U : 0U;
    frame[4] = X42S_FIXED_CHECKSUM;
    return X42S_PROTOCOL_OK;
}

X42sProtocolResult X42sProtocol_ParseCommandResponse(
    const uint8_t *frame,
    size_t length,
    uint8_t expected_address,
    uint8_t expected_function,
    X42sCommandStatus *status)
{
    uint8_t response_status;

    if ((frame == NULL) || (status == NULL))
    {
        return X42S_PROTOCOL_INVALID_ARGUMENT;
    }
    if (length != X42S_COMMAND_RESPONSE_SIZE)
    {
        return X42S_PROTOCOL_INVALID_LENGTH;
    }
    if ((expected_address == X42S_BROADCAST_ADDRESS) ||
        (frame[0] != expected_address))
    {
        return X42S_PROTOCOL_INVALID_ADDRESS_ECHO;
    }
    if (frame[1] != expected_function)
    {
        return X42S_PROTOCOL_INVALID_FUNCTION;
    }
    if (frame[3] != X42S_FIXED_CHECKSUM)
    {
        return X42S_PROTOCOL_INVALID_CHECKSUM;
    }

    response_status = frame[2];
    if ((response_status != X42S_COMMAND_STATUS_ACCEPTED) &&
        (response_status != X42S_COMMAND_STATUS_ALREADY_AT_ORIGIN) &&
        (response_status != X42S_COMMAND_STATUS_LIMIT_ACTIVE) &&
        (response_status != X42S_COMMAND_STATUS_PARAMETER_ERROR) &&
        (response_status != X42S_COMMAND_STATUS_FORMAT_ERROR) &&
        (response_status != X42S_COMMAND_STATUS_ACTION_COMPLETE))
    {
        return X42S_PROTOCOL_INVALID_COMMAND_STATUS;
    }

    *status = (X42sCommandStatus)response_status;
    return X42S_PROTOCOL_OK;
}
