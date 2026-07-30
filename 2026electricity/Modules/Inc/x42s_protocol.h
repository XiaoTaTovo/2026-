#ifndef X42S_PROTOCOL_H
#define X42S_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define X42S_FIXED_CHECKSUM 0x6BU
#define X42S_DEFAULT_ADDRESS 0x01U
#define X42S_READ_ADDRESS_REQUEST_SIZE 3U
#define X42S_READ_ADDRESS_RESPONSE_SIZE 4U
#define X42S_READ_STATUS_REQUEST_SIZE 3U
#define X42S_READ_STATUS_RESPONSE_SIZE 4U
#define X42S_READ_POSITION_REQUEST_SIZE 3U
#define X42S_READ_POSITION_RESPONSE_SIZE 8U
#define X42S_EMM_ENABLE_REQUEST_SIZE 6U
#define X42S_EMM_POSITION_REQUEST_SIZE 13U
#define X42S_STOP_REQUEST_SIZE 5U
#define X42S_COMMAND_RESPONSE_SIZE 4U
#define X42S_EMM_MAX_SPEED_RPM 3000U

typedef enum
{
    X42S_PROTOCOL_OK = 0,
    X42S_PROTOCOL_INVALID_ARGUMENT,
    X42S_PROTOCOL_BUFFER_TOO_SMALL,
    X42S_PROTOCOL_INVALID_LENGTH,
    X42S_PROTOCOL_INVALID_ADDRESS,
    X42S_PROTOCOL_INVALID_FUNCTION,
    X42S_PROTOCOL_INVALID_ADDRESS_ECHO,
    X42S_PROTOCOL_INVALID_SIGN,
    X42S_PROTOCOL_VALUE_OUT_OF_RANGE,
    X42S_PROTOCOL_INVALID_COMMAND_STATUS,
    X42S_PROTOCOL_INVALID_CHECKSUM
} X42sProtocolResult;

typedef enum
{
    X42S_COMMAND_STATUS_ACCEPTED = 0x02,
    X42S_COMMAND_STATUS_ALREADY_AT_ORIGIN = 0x12,
    X42S_COMMAND_STATUS_LIMIT_ACTIVE = 0x22,
    X42S_COMMAND_STATUS_PARAMETER_ERROR = 0xE2,
    X42S_COMMAND_STATUS_FORMAT_ERROR = 0xEE,
    X42S_COMMAND_STATUS_ACTION_COMPLETE = 0x9F
} X42sCommandStatus;

typedef struct
{
    bool negative;
    uint32_t magnitude;
} X42sRawPosition;

X42sProtocolResult X42sProtocol_BuildReadAddress(
    uint8_t *frame,
    size_t capacity);

X42sProtocolResult X42sProtocol_ParseReadAddress(
    const uint8_t *frame,
    size_t length,
    uint8_t *address);

X42sProtocolResult X42sProtocol_BuildReadStatus(
    uint8_t address,
    uint8_t *frame,
    size_t capacity);

X42sProtocolResult X42sProtocol_ParseReadStatus(
    const uint8_t *frame,
    size_t length,
    uint8_t expected_address,
    uint8_t *status);

X42sProtocolResult X42sProtocol_BuildReadPosition(
    uint8_t address,
    uint8_t *frame,
    size_t capacity);

X42sProtocolResult X42sProtocol_ParseReadPosition(
    const uint8_t *frame,
    size_t length,
    uint8_t expected_address,
    X42sRawPosition *position);

X42sProtocolResult X42sProtocol_BuildEmmEnable(
    uint8_t address,
    bool enable,
    bool synchronize,
    uint8_t *frame,
    size_t capacity);

X42sProtocolResult X42sProtocol_BuildEmmPosition(
    uint8_t address,
    uint8_t direction,
    uint16_t speed_rpm,
    uint8_t acceleration,
    uint32_t pulse_count,
    uint8_t motion_mode,
    bool synchronize,
    uint8_t *frame,
    size_t capacity);

X42sProtocolResult X42sProtocol_BuildStop(
    uint8_t address,
    bool synchronize,
    uint8_t *frame,
    size_t capacity);

X42sProtocolResult X42sProtocol_ParseCommandResponse(
    const uint8_t *frame,
    size_t length,
    uint8_t expected_address,
    uint8_t expected_function,
    X42sCommandStatus *status);

#endif
