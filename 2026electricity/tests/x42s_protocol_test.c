#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "x42s_protocol.h"

static void test_build_read_address(void)
{
    uint8_t frame[X42S_READ_ADDRESS_REQUEST_SIZE] = {0U};
    const uint8_t expected[X42S_READ_ADDRESS_REQUEST_SIZE] = {0x00U, 0x15U, 0x6BU};

    assert(X42sProtocol_BuildReadAddress(frame, sizeof(frame)) == X42S_PROTOCOL_OK);
    assert(memcmp(frame, expected, sizeof(frame)) == 0);
    assert(X42sProtocol_BuildReadAddress(frame, sizeof(frame) - 1U) ==
           X42S_PROTOCOL_BUFFER_TOO_SMALL);
}

static void test_parse_read_address(void)
{
    const uint8_t valid_response[X42S_READ_ADDRESS_RESPONSE_SIZE] =
        {0x12U, 0x15U, 0x12U, 0x6BU};
    uint8_t invalid_response[X42S_READ_ADDRESS_RESPONSE_SIZE];
    uint8_t address = 0U;

    assert(X42sProtocol_ParseReadAddress(
               valid_response,
               sizeof(valid_response),
               &address) == X42S_PROTOCOL_OK);
    assert(address == 0x12U);
    assert(X42sProtocol_ParseReadAddress(
               valid_response,
               sizeof(valid_response) - 1U,
               &address) == X42S_PROTOCOL_INVALID_LENGTH);

    memcpy(invalid_response, valid_response, sizeof(invalid_response));
    invalid_response[0] = 0x00U;
    assert(X42sProtocol_ParseReadAddress(
               invalid_response,
               sizeof(invalid_response),
               &address) == X42S_PROTOCOL_INVALID_ADDRESS);

    memcpy(invalid_response, valid_response, sizeof(invalid_response));
    invalid_response[1] = 0x36U;
    assert(X42sProtocol_ParseReadAddress(
               invalid_response,
               sizeof(invalid_response),
               &address) == X42S_PROTOCOL_INVALID_FUNCTION);

    memcpy(invalid_response, valid_response, sizeof(invalid_response));
    invalid_response[2] = 0x13U;
    assert(X42sProtocol_ParseReadAddress(
               invalid_response,
               sizeof(invalid_response),
               &address) == X42S_PROTOCOL_INVALID_ADDRESS_ECHO);

    memcpy(invalid_response, valid_response, sizeof(invalid_response));
    invalid_response[3] = 0x00U;
    assert(X42sProtocol_ParseReadAddress(
               invalid_response,
               sizeof(invalid_response),
               &address) == X42S_PROTOCOL_INVALID_CHECKSUM);
}

static void test_read_status(void)
{
    uint8_t request[X42S_READ_STATUS_REQUEST_SIZE] = {0U};
    const uint8_t expected_request[X42S_READ_STATUS_REQUEST_SIZE] =
        {0x01U, 0x3AU, 0x6BU};
    const uint8_t valid_response[X42S_READ_STATUS_RESPONSE_SIZE] =
        {0x01U, 0x3AU, 0x05U, 0x6BU};
    uint8_t status = 0U;

    assert(X42sProtocol_BuildReadStatus(
               X42S_DEFAULT_ADDRESS,
               request,
               sizeof(request)) == X42S_PROTOCOL_OK);
    assert(memcmp(request, expected_request, sizeof(request)) == 0);
    assert(X42sProtocol_BuildReadStatus(
               X42S_DEFAULT_ADDRESS,
               request,
               sizeof(request) - 1U) == X42S_PROTOCOL_BUFFER_TOO_SMALL);
    assert(X42sProtocol_BuildReadStatus(0U, request, sizeof(request)) ==
           X42S_PROTOCOL_INVALID_ADDRESS);

    assert(X42sProtocol_ParseReadStatus(
               valid_response,
               sizeof(valid_response),
               X42S_DEFAULT_ADDRESS,
               &status) == X42S_PROTOCOL_OK);
    assert(status == 0x05U);
}

static void test_read_position(void)
{
    uint8_t request[X42S_READ_POSITION_REQUEST_SIZE] = {0U};
    const uint8_t expected_request[X42S_READ_POSITION_REQUEST_SIZE] =
        {0x01U, 0x36U, 0x6BU};
    uint8_t response[X42S_READ_POSITION_RESPONSE_SIZE] =
        {0x01U, 0x36U, 0x01U, 0x12U, 0x34U, 0x56U, 0x78U, 0x6BU};
    X42sRawPosition position = {false, 0U};

    assert(X42sProtocol_BuildReadPosition(
               X42S_DEFAULT_ADDRESS,
               request,
               sizeof(request)) == X42S_PROTOCOL_OK);
    assert(memcmp(request, expected_request, sizeof(request)) == 0);
    assert(X42sProtocol_BuildReadPosition(
               X42S_DEFAULT_ADDRESS,
               request,
               sizeof(request) - 1U) == X42S_PROTOCOL_BUFFER_TOO_SMALL);

    assert(X42sProtocol_ParseReadPosition(
               response,
               sizeof(response),
               X42S_DEFAULT_ADDRESS,
               &position) == X42S_PROTOCOL_OK);
    assert(position.negative);
    assert(position.magnitude == 0x12345678U);

    response[2] = 2U;
    assert(X42sProtocol_ParseReadPosition(
               response,
               sizeof(response),
               X42S_DEFAULT_ADDRESS,
           &position) == X42S_PROTOCOL_INVALID_SIGN);
}

static void test_emm_motion_command_frames(void)
{
    uint8_t enable[X42S_EMM_ENABLE_REQUEST_SIZE] = {0U};
    uint8_t disable[X42S_EMM_ENABLE_REQUEST_SIZE] = {0U};
    uint8_t position[X42S_EMM_POSITION_REQUEST_SIZE] = {0U};
    uint8_t stop[X42S_STOP_REQUEST_SIZE] = {0U};
    const uint8_t expected_enable[X42S_EMM_ENABLE_REQUEST_SIZE] =
        {0x01U, 0xF3U, 0xABU, 0x01U, 0x00U, 0x6BU};
    const uint8_t expected_disable[X42S_EMM_ENABLE_REQUEST_SIZE] =
        {0x01U, 0xF3U, 0xABU, 0x00U, 0x00U, 0x6BU};
    const uint8_t expected_position[X42S_EMM_POSITION_REQUEST_SIZE] =
        {0x01U, 0xFDU, 0x01U, 0x05U, 0xDCU, 0x00U, 0x00U,
         0x00U, 0x7DU, 0x00U, 0x00U, 0x00U, 0x6BU};
    const uint8_t expected_stop[X42S_STOP_REQUEST_SIZE] =
        {0x01U, 0xFEU, 0x98U, 0x00U, 0x6BU};

    assert(X42sProtocol_BuildEmmEnable(
               1U, true, false, enable, sizeof(enable)) == X42S_PROTOCOL_OK);
    assert(memcmp(enable, expected_enable, sizeof(enable)) == 0);
    assert(X42sProtocol_BuildEmmEnable(
               1U, false, false, disable, sizeof(disable)) == X42S_PROTOCOL_OK);
    assert(memcmp(disable, expected_disable, sizeof(disable)) == 0);

    assert(X42sProtocol_BuildEmmPosition(
               1U, 1U, 1500U, 0U, 32000U, 0U, false,
               position, sizeof(position)) == X42S_PROTOCOL_OK);
    assert(memcmp(position, expected_position, sizeof(position)) == 0);
    assert(X42sProtocol_BuildEmmPosition(
               1U, 0U, X42S_EMM_MAX_SPEED_RPM + 1U, 1U, 1U, 2U, false,
               position, sizeof(position)) == X42S_PROTOCOL_VALUE_OUT_OF_RANGE);

    assert(X42sProtocol_BuildStop(
               1U, false, stop, sizeof(stop)) == X42S_PROTOCOL_OK);
    assert(memcmp(stop, expected_stop, sizeof(stop)) == 0);
}

static void test_command_response(void)
{
    uint8_t response[X42S_COMMAND_RESPONSE_SIZE] =
        {0x01U, 0xFDU, 0x02U, 0x6BU};
    X42sCommandStatus status = X42S_COMMAND_STATUS_FORMAT_ERROR;

    assert(X42sProtocol_ParseCommandResponse(
               response, sizeof(response), 1U, 0xFDU, &status) ==
           X42S_PROTOCOL_OK);
    assert(status == X42S_COMMAND_STATUS_ACCEPTED);

    response[2] = 0xE2U;
    assert(X42sProtocol_ParseCommandResponse(
               response, sizeof(response), 1U, 0xFDU, &status) ==
           X42S_PROTOCOL_OK);
    assert(status == X42S_COMMAND_STATUS_PARAMETER_ERROR);

    response[2] = 0x03U;
    assert(X42sProtocol_ParseCommandResponse(
               response, sizeof(response), 1U, 0xFDU, &status) ==
           X42S_PROTOCOL_INVALID_COMMAND_STATUS);
}

int main(void)
{
    test_build_read_address();
    test_parse_read_address();
    test_read_status();
    test_read_position();
    test_emm_motion_command_frames();
    test_command_response();
    puts("X42S_EMM_MOTION_PROTOCOL_TEST=PASS");
    return 0;
}
