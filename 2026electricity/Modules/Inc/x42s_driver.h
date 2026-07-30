#ifndef X42S_DRIVER_H
#define X42S_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

#include "bsp_uart_dma.h"
#include "x42s_protocol.h"

#define X42S_DRIVER_ADDRESS_QUERY_TIMEOUT_MS 100U
#define X42S_DRIVER_STATUS_QUERY_TIMEOUT_MS 100U
#define X42S_DRIVER_POSITION_QUERY_TIMEOUT_MS 100U
#define X42S_DRIVER_COMMAND_TIMEOUT_MS 100U

typedef enum
{
    X42S_DRIVER_OK = 0,
    X42S_DRIVER_INVALID_ARGUMENT,
    X42S_DRIVER_NOT_STARTED,
    X42S_DRIVER_BUSY,
    X42S_DRIVER_INVALID_CONFIG,
    X42S_DRIVER_TRANSPORT_ERROR
} X42sDriverResult;

typedef enum
{
    X42S_DRIVER_STATE_UNINITIALIZED = 0,
    X42S_DRIVER_STATE_READY,
    X42S_DRIVER_STATE_WAITING_FOR_ADDRESS,
    X42S_DRIVER_STATE_ADDRESS_VALID,
    X42S_DRIVER_STATE_ADDRESS_TIMEOUT,
    X42S_DRIVER_STATE_WAITING_FOR_STATUS,
    X42S_DRIVER_STATE_STATUS_VALID,
    X42S_DRIVER_STATE_STATUS_TIMEOUT,
    X42S_DRIVER_STATE_WAITING_FOR_POSITION,
    X42S_DRIVER_STATE_POSITION_VALID,
    X42S_DRIVER_STATE_POSITION_TIMEOUT,
    X42S_DRIVER_STATE_WAITING_FOR_COMMAND,
    X42S_DRIVER_STATE_COMMAND_VALID,
    X42S_DRIVER_STATE_COMMAND_TIMEOUT
} X42sDriverState;

typedef struct
{
    BspUartDmaPort transport;
    X42sDriverState state;
    X42sProtocolResult last_protocol_result;

    uint8_t address;
    uint8_t response_window[X42S_READ_ADDRESS_RESPONSE_SIZE];
    uint8_t response_window_length;
    uint32_t request_started_ms;

    uint8_t status;
    uint8_t status_query_address;
    uint8_t status_response_window[X42S_READ_STATUS_RESPONSE_SIZE];
    uint8_t status_response_window_length;
    uint32_t status_request_started_ms;

    X42sRawPosition position;
    uint8_t position_query_address;
    uint8_t position_response_window[X42S_READ_POSITION_RESPONSE_SIZE];
    uint8_t position_response_window_length;
    uint32_t position_request_started_ms;

    X42sCommandStatus command_status;
    uint8_t command_query_address;
    uint8_t command_expected_function;
    uint8_t command_response_window[X42S_COMMAND_RESPONSE_SIZE];
    uint8_t command_response_window_length;
    uint32_t command_request_started_ms;

    uint32_t address_request_count;
    uint32_t valid_address_response_count;
    uint32_t address_timeout_count;
    uint32_t protocol_error_count;
    uint32_t status_request_count;
    uint32_t valid_status_response_count;
    uint32_t status_timeout_count;
    uint32_t position_request_count;
    uint32_t valid_position_response_count;
    uint32_t position_timeout_count;
    uint32_t command_request_count;
    uint32_t valid_command_response_count;
    uint32_t command_timeout_count;
    bool initialized;
    bool started;
} X42sDriver;

X42sDriverResult X42sDriver_Init(
    X42sDriver *driver,
    UART_HandleTypeDef *uart);

X42sDriverResult X42sDriver_Start(X42sDriver *driver);

X42sDriverResult X42sDriver_RequestReadAddress(
    X42sDriver *driver,
    uint32_t now_ms);

X42sDriverResult X42sDriver_RequestReadStatus(
    X42sDriver *driver,
    uint8_t address,
    uint32_t now_ms);

X42sDriverResult X42sDriver_RequestReadPosition(
    X42sDriver *driver,
    uint8_t address,
    uint32_t now_ms);

X42sDriverResult X42sDriver_RequestEnable(
    X42sDriver *driver,
    uint8_t address,
    bool enable,
    bool synchronize,
    uint32_t now_ms);

X42sDriverResult X42sDriver_RequestEmmPosition(
    X42sDriver *driver,
    uint8_t address,
    uint8_t direction,
    uint16_t speed_rpm,
    uint8_t acceleration,
    uint32_t pulse_count,
    uint8_t motion_mode,
    bool synchronize,
    uint32_t now_ms);

X42sDriverResult X42sDriver_RequestStop(
    X42sDriver *driver,
    uint8_t address,
    bool synchronize,
    uint32_t now_ms);

void X42sDriver_Service(X42sDriver *driver, uint32_t now_ms);

X42sDriverState X42sDriver_GetState(const X42sDriver *driver);
uint8_t X42sDriver_GetAddress(const X42sDriver *driver);
uint8_t X42sDriver_GetStatus(const X42sDriver *driver);
X42sRawPosition X42sDriver_GetPosition(const X42sDriver *driver);
X42sCommandStatus X42sDriver_GetCommandStatus(const X42sDriver *driver);
uint8_t X42sDriver_GetCommandFunction(const X42sDriver *driver);

#endif
