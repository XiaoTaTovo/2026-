#include "x42s_driver.h"

#include <string.h>

static X42sDriverResult map_transport_result(BspUartDmaResult result)
{
    switch (result)
    {
        case BSP_UART_DMA_OK:
            return X42S_DRIVER_OK;
        case BSP_UART_DMA_FULL:
            return X42S_DRIVER_BUSY;
        case BSP_UART_DMA_INVALID_ARGUMENT:
            return X42S_DRIVER_INVALID_ARGUMENT;
        case BSP_UART_DMA_INVALID_CONFIG:
            return X42S_DRIVER_INVALID_CONFIG;
        default:
            return X42S_DRIVER_TRANSPORT_ERROR;
    }
}

static bool transaction_pending(const X42sDriver *driver)
{
    return (driver->state == X42S_DRIVER_STATE_WAITING_FOR_ADDRESS) ||
           (driver->state == X42S_DRIVER_STATE_WAITING_FOR_STATUS) ||
           (driver->state == X42S_DRIVER_STATE_WAITING_FOR_POSITION) ||
           (driver->state == X42S_DRIVER_STATE_WAITING_FOR_COMMAND);
}

static void discard_pending_rx(X42sDriver *driver)
{
    uint8_t ignored_byte;

    while (BspUartDma_Read(&driver->transport, &ignored_byte, 1U) == 1U)
    {
    }
}

static void consume_address_response_byte(X42sDriver *driver, uint8_t byte)
{
    X42sProtocolResult result;

    if (driver->response_window_length < X42S_READ_ADDRESS_RESPONSE_SIZE)
    {
        driver->response_window[driver->response_window_length++] = byte;
    }
    else
    {
        driver->response_window[0] = driver->response_window[1];
        driver->response_window[1] = driver->response_window[2];
        driver->response_window[2] = driver->response_window[3];
        driver->response_window[3] = byte;
    }

    if (driver->response_window_length != X42S_READ_ADDRESS_RESPONSE_SIZE)
    {
        return;
    }

    result = X42sProtocol_ParseReadAddress(
        driver->response_window,
        X42S_READ_ADDRESS_RESPONSE_SIZE,
        &driver->address);
    driver->last_protocol_result = result;

    if (result == X42S_PROTOCOL_OK)
    {
        driver->valid_address_response_count++;
        driver->state = X42S_DRIVER_STATE_ADDRESS_VALID;
        return;
    }

    driver->protocol_error_count++;
    driver->response_window[0] = driver->response_window[1];
    driver->response_window[1] = driver->response_window[2];
    driver->response_window[2] = driver->response_window[3];
    driver->response_window_length = X42S_READ_ADDRESS_RESPONSE_SIZE - 1U;
}

static void consume_status_response_byte(X42sDriver *driver, uint8_t byte)
{
    X42sProtocolResult result;

    if (driver->status_response_window_length < X42S_READ_STATUS_RESPONSE_SIZE)
    {
        driver->status_response_window[driver->status_response_window_length++] = byte;
    }
    else
    {
        driver->status_response_window[0] = driver->status_response_window[1];
        driver->status_response_window[1] = driver->status_response_window[2];
        driver->status_response_window[2] = driver->status_response_window[3];
        driver->status_response_window[3] = byte;
    }

    if (driver->status_response_window_length != X42S_READ_STATUS_RESPONSE_SIZE)
    {
        return;
    }

    result = X42sProtocol_ParseReadStatus(
        driver->status_response_window,
        X42S_READ_STATUS_RESPONSE_SIZE,
        driver->status_query_address,
        &driver->status);
    driver->last_protocol_result = result;

    if (result == X42S_PROTOCOL_OK)
    {
        driver->valid_status_response_count++;
        driver->state = X42S_DRIVER_STATE_STATUS_VALID;
        return;
    }

    driver->protocol_error_count++;
    driver->status_response_window[0] = driver->status_response_window[1];
    driver->status_response_window[1] = driver->status_response_window[2];
    driver->status_response_window[2] = driver->status_response_window[3];
    driver->status_response_window_length = X42S_READ_STATUS_RESPONSE_SIZE - 1U;
}

static void consume_position_response_byte(X42sDriver *driver, uint8_t byte)
{
    X42sProtocolResult result;
    uint8_t i;

    if (driver->position_response_window_length < X42S_READ_POSITION_RESPONSE_SIZE)
    {
        driver->position_response_window[driver->position_response_window_length++] = byte;
    }
    else
    {
        for (i = 1U; i < X42S_READ_POSITION_RESPONSE_SIZE; ++i)
        {
            driver->position_response_window[i - 1U] =
                driver->position_response_window[i];
        }
        driver->position_response_window[X42S_READ_POSITION_RESPONSE_SIZE - 1U] = byte;
    }

    if (driver->position_response_window_length != X42S_READ_POSITION_RESPONSE_SIZE)
    {
        return;
    }

    result = X42sProtocol_ParseReadPosition(
        driver->position_response_window,
        X42S_READ_POSITION_RESPONSE_SIZE,
        driver->position_query_address,
        &driver->position);
    driver->last_protocol_result = result;

    if (result == X42S_PROTOCOL_OK)
    {
        driver->valid_position_response_count++;
        driver->state = X42S_DRIVER_STATE_POSITION_VALID;
        return;
    }

    driver->protocol_error_count++;
    for (i = 1U; i < X42S_READ_POSITION_RESPONSE_SIZE; ++i)
    {
        driver->position_response_window[i - 1U] =
            driver->position_response_window[i];
    }
    driver->position_response_window_length =
        X42S_READ_POSITION_RESPONSE_SIZE - 1U;
}

static void consume_command_response_byte(X42sDriver *driver, uint8_t byte)
{
    X42sProtocolResult result;

    if (driver->command_response_window_length < X42S_COMMAND_RESPONSE_SIZE)
    {
        driver->command_response_window[
            driver->command_response_window_length++] = byte;
    }
    else
    {
        driver->command_response_window[0] =
            driver->command_response_window[1];
        driver->command_response_window[1] =
            driver->command_response_window[2];
        driver->command_response_window[2] =
            driver->command_response_window[3];
        driver->command_response_window[3] = byte;
    }

    if (driver->command_response_window_length != X42S_COMMAND_RESPONSE_SIZE)
    {
        return;
    }

    result = X42sProtocol_ParseCommandResponse(
        driver->command_response_window,
        X42S_COMMAND_RESPONSE_SIZE,
        driver->command_query_address,
        driver->command_expected_function,
        &driver->command_status);
    driver->last_protocol_result = result;

    if (result == X42S_PROTOCOL_OK)
    {
        driver->valid_command_response_count++;
        driver->state = X42S_DRIVER_STATE_COMMAND_VALID;
        return;
    }

    driver->protocol_error_count++;
    driver->command_response_window[0] =
        driver->command_response_window[1];
    driver->command_response_window[1] =
        driver->command_response_window[2];
    driver->command_response_window[2] =
        driver->command_response_window[3];
    driver->command_response_window_length = X42S_COMMAND_RESPONSE_SIZE - 1U;
}

static X42sDriverResult send_command(
    X42sDriver *driver,
    uint8_t address,
    const uint8_t *request,
    size_t request_length,
    uint32_t now_ms,
    bool supersede_pending)
{
    BspUartDmaResult transport_result;

    if ((driver == NULL) || (request == NULL) || (request_length < 2U))
    {
        return X42S_DRIVER_INVALID_ARGUMENT;
    }
    if (!driver->started)
    {
        return X42S_DRIVER_NOT_STARTED;
    }
    if (transaction_pending(driver) && !supersede_pending)
    {
        return X42S_DRIVER_BUSY;
    }

    discard_pending_rx(driver);
    transport_result = BspUartDma_Write(
        &driver->transport,
        request,
        request_length);
    if (transport_result != BSP_UART_DMA_OK)
    {
        return map_transport_result(transport_result);
    }

    driver->command_query_address = address;
    driver->command_expected_function = request[1];
    driver->command_response_window_length = 0U;
    driver->command_request_started_ms = now_ms;
    driver->command_request_count++;
    driver->state = X42S_DRIVER_STATE_WAITING_FOR_COMMAND;
    return X42S_DRIVER_OK;
}

X42sDriverResult X42sDriver_Init(
    X42sDriver *driver,
    UART_HandleTypeDef *uart)
{
    BspUartDmaResult result;

    if ((driver == NULL) || (uart == NULL))
    {
        return X42S_DRIVER_INVALID_ARGUMENT;
    }

    memset(driver, 0, sizeof(*driver));
    result = BspUartDma_Init(&driver->transport, uart);
    if (result != BSP_UART_DMA_OK)
    {
        return map_transport_result(result);
    }

    driver->initialized = true;
    driver->state = X42S_DRIVER_STATE_READY;
    return X42S_DRIVER_OK;
}

X42sDriverResult X42sDriver_Start(X42sDriver *driver)
{
    BspUartDmaResult result;

    if (driver == NULL)
    {
        return X42S_DRIVER_INVALID_ARGUMENT;
    }
    if (!driver->initialized)
    {
        return X42S_DRIVER_NOT_STARTED;
    }

    result = BspUartDma_Start(&driver->transport);
    if (result != BSP_UART_DMA_OK)
    {
        return map_transport_result(result);
    }

    driver->started = true;
    driver->state = X42S_DRIVER_STATE_READY;
    return X42S_DRIVER_OK;
}

X42sDriverResult X42sDriver_RequestReadAddress(
    X42sDriver *driver,
    uint32_t now_ms)
{
    uint8_t request[X42S_READ_ADDRESS_REQUEST_SIZE];
    BspUartDmaResult transport_result;
    X42sProtocolResult protocol_result;

    if (driver == NULL)
    {
        return X42S_DRIVER_INVALID_ARGUMENT;
    }
    if (!driver->started)
    {
        return X42S_DRIVER_NOT_STARTED;
    }
    if (transaction_pending(driver))
    {
        return X42S_DRIVER_BUSY;
    }

    protocol_result = X42sProtocol_BuildReadAddress(request, sizeof(request));
    if (protocol_result != X42S_PROTOCOL_OK)
    {
        driver->last_protocol_result = protocol_result;
        return X42S_DRIVER_TRANSPORT_ERROR;
    }

    discard_pending_rx(driver);
    driver->response_window_length = 0U;
    transport_result = BspUartDma_Write(
        &driver->transport,
        request,
        sizeof(request));
    if (transport_result != BSP_UART_DMA_OK)
    {
        return map_transport_result(transport_result);
    }

    driver->request_started_ms = now_ms;
    driver->address_request_count++;
    driver->state = X42S_DRIVER_STATE_WAITING_FOR_ADDRESS;
    return X42S_DRIVER_OK;
}

X42sDriverResult X42sDriver_RequestReadStatus(
    X42sDriver *driver,
    uint8_t address,
    uint32_t now_ms)
{
    uint8_t request[X42S_READ_STATUS_REQUEST_SIZE];
    BspUartDmaResult transport_result;
    X42sProtocolResult protocol_result;

    if (driver == NULL)
    {
        return X42S_DRIVER_INVALID_ARGUMENT;
    }
    if (!driver->started)
    {
        return X42S_DRIVER_NOT_STARTED;
    }
    if (transaction_pending(driver))
    {
        return X42S_DRIVER_BUSY;
    }

    protocol_result = X42sProtocol_BuildReadStatus(
        address,
        request,
        sizeof(request));
    if (protocol_result != X42S_PROTOCOL_OK)
    {
        driver->last_protocol_result = protocol_result;
        return X42S_DRIVER_TRANSPORT_ERROR;
    }

    discard_pending_rx(driver);
    driver->status_query_address = address;
    driver->status_response_window_length = 0U;
    transport_result = BspUartDma_Write(
        &driver->transport,
        request,
        sizeof(request));
    if (transport_result != BSP_UART_DMA_OK)
    {
        return map_transport_result(transport_result);
    }

    driver->status_request_started_ms = now_ms;
    driver->status_request_count++;
    driver->state = X42S_DRIVER_STATE_WAITING_FOR_STATUS;
    return X42S_DRIVER_OK;
}

X42sDriverResult X42sDriver_RequestReadPosition(
    X42sDriver *driver,
    uint8_t address,
    uint32_t now_ms)
{
    uint8_t request[X42S_READ_POSITION_REQUEST_SIZE];
    BspUartDmaResult transport_result;
    X42sProtocolResult protocol_result;

    if (driver == NULL)
    {
        return X42S_DRIVER_INVALID_ARGUMENT;
    }
    if (!driver->started)
    {
        return X42S_DRIVER_NOT_STARTED;
    }
    if (transaction_pending(driver))
    {
        return X42S_DRIVER_BUSY;
    }

    protocol_result = X42sProtocol_BuildReadPosition(
        address,
        request,
        sizeof(request));
    if (protocol_result != X42S_PROTOCOL_OK)
    {
        driver->last_protocol_result = protocol_result;
        return X42S_DRIVER_TRANSPORT_ERROR;
    }

    discard_pending_rx(driver);
    driver->position_query_address = address;
    driver->position_response_window_length = 0U;
    transport_result = BspUartDma_Write(
        &driver->transport,
        request,
        sizeof(request));
    if (transport_result != BSP_UART_DMA_OK)
    {
        return map_transport_result(transport_result);
    }

    driver->position_request_started_ms = now_ms;
    driver->position_request_count++;
    driver->state = X42S_DRIVER_STATE_WAITING_FOR_POSITION;
    return X42S_DRIVER_OK;
}

X42sDriverResult X42sDriver_RequestEnable(
    X42sDriver *driver,
    uint8_t address,
    bool enable,
    bool synchronize,
    uint32_t now_ms)
{
    uint8_t request[X42S_EMM_ENABLE_REQUEST_SIZE];
    X42sProtocolResult protocol_result;

    if (driver == NULL)
    {
        return X42S_DRIVER_INVALID_ARGUMENT;
    }

    protocol_result = X42sProtocol_BuildEmmEnable(
        address,
        enable,
        synchronize,
        request,
        sizeof(request));
    if (protocol_result != X42S_PROTOCOL_OK)
    {
        driver->last_protocol_result = protocol_result;
        return X42S_DRIVER_INVALID_ARGUMENT;
    }

    return send_command(
        driver,
        address,
        request,
        sizeof(request),
        now_ms,
        false);
}

X42sDriverResult X42sDriver_RequestEmmPosition(
    X42sDriver *driver,
    uint8_t address,
    uint8_t direction,
    uint16_t speed_rpm,
    uint8_t acceleration,
    uint32_t pulse_count,
    uint8_t motion_mode,
    bool synchronize,
    uint32_t now_ms)
{
    uint8_t request[X42S_EMM_POSITION_REQUEST_SIZE];
    X42sProtocolResult protocol_result;

    if (driver == NULL)
    {
        return X42S_DRIVER_INVALID_ARGUMENT;
    }

    protocol_result = X42sProtocol_BuildEmmPosition(
        address,
        direction,
        speed_rpm,
        acceleration,
        pulse_count,
        motion_mode,
        synchronize,
        request,
        sizeof(request));
    if (protocol_result != X42S_PROTOCOL_OK)
    {
        driver->last_protocol_result = protocol_result;
        return X42S_DRIVER_INVALID_ARGUMENT;
    }

    return send_command(
        driver,
        address,
        request,
        sizeof(request),
        now_ms,
        false);
}

X42sDriverResult X42sDriver_RequestStop(
    X42sDriver *driver,
    uint8_t address,
    bool synchronize,
    uint32_t now_ms)
{
    uint8_t request[X42S_STOP_REQUEST_SIZE];
    X42sProtocolResult protocol_result;

    if (driver == NULL)
    {
        return X42S_DRIVER_INVALID_ARGUMENT;
    }

    protocol_result = X42sProtocol_BuildStop(
        address,
        synchronize,
        request,
        sizeof(request));
    if (protocol_result != X42S_PROTOCOL_OK)
    {
        driver->last_protocol_result = protocol_result;
        return X42S_DRIVER_INVALID_ARGUMENT;
    }

    /* STOP supersedes the parser transaction. UART bytes already queued or
     * actively shifting cannot be retracted; with this app's single-command
     * policy the worst preceding frame is 13 bytes (about 1.2 ms at 115200). */
    return send_command(
        driver,
        address,
        request,
        sizeof(request),
        now_ms,
        true);
}

void X42sDriver_Service(X42sDriver *driver, uint32_t now_ms)
{
    uint8_t byte;

    if ((driver == NULL) || !driver->started)
    {
        return;
    }

    BspUartDma_Service(&driver->transport);

    while ((driver->state == X42S_DRIVER_STATE_WAITING_FOR_ADDRESS) &&
           (BspUartDma_Read(&driver->transport, &byte, 1U) == 1U))
    {
        consume_address_response_byte(driver, byte);
    }

    while ((driver->state == X42S_DRIVER_STATE_WAITING_FOR_STATUS) &&
           (BspUartDma_Read(&driver->transport, &byte, 1U) == 1U))
    {
        consume_status_response_byte(driver, byte);
    }

    while ((driver->state == X42S_DRIVER_STATE_WAITING_FOR_POSITION) &&
           (BspUartDma_Read(&driver->transport, &byte, 1U) == 1U))
    {
        consume_position_response_byte(driver, byte);
    }

    while ((driver->state == X42S_DRIVER_STATE_WAITING_FOR_COMMAND) &&
           (BspUartDma_Read(&driver->transport, &byte, 1U) == 1U))
    {
        consume_command_response_byte(driver, byte);
    }

    if ((driver->state == X42S_DRIVER_STATE_WAITING_FOR_ADDRESS) &&
        ((uint32_t)(now_ms - driver->request_started_ms) >=
         X42S_DRIVER_ADDRESS_QUERY_TIMEOUT_MS))
    {
        driver->address_timeout_count++;
        driver->state = X42S_DRIVER_STATE_ADDRESS_TIMEOUT;
    }

    if ((driver->state == X42S_DRIVER_STATE_WAITING_FOR_STATUS) &&
        ((uint32_t)(now_ms - driver->status_request_started_ms) >=
         X42S_DRIVER_STATUS_QUERY_TIMEOUT_MS))
    {
        driver->status_timeout_count++;
        driver->state = X42S_DRIVER_STATE_STATUS_TIMEOUT;
    }

    if ((driver->state == X42S_DRIVER_STATE_WAITING_FOR_POSITION) &&
        ((uint32_t)(now_ms - driver->position_request_started_ms) >=
         X42S_DRIVER_POSITION_QUERY_TIMEOUT_MS))
    {
        driver->position_timeout_count++;
        driver->state = X42S_DRIVER_STATE_POSITION_TIMEOUT;
    }

    if ((driver->state == X42S_DRIVER_STATE_WAITING_FOR_COMMAND) &&
        ((uint32_t)(now_ms - driver->command_request_started_ms) >=
         X42S_DRIVER_COMMAND_TIMEOUT_MS))
    {
        driver->command_timeout_count++;
        driver->state = X42S_DRIVER_STATE_COMMAND_TIMEOUT;
    }
}

X42sDriverState X42sDriver_GetState(const X42sDriver *driver)
{
    return (driver == NULL) ? X42S_DRIVER_STATE_UNINITIALIZED : driver->state;
}

uint8_t X42sDriver_GetAddress(const X42sDriver *driver)
{
    return (driver == NULL) ? 0U : driver->address;
}

uint8_t X42sDriver_GetStatus(const X42sDriver *driver)
{
    return (driver == NULL) ? 0U : driver->status;
}

X42sRawPosition X42sDriver_GetPosition(const X42sDriver *driver)
{
    X42sRawPosition position = {false, 0U};

    if (driver != NULL)
    {
        position = driver->position;
    }
    return position;
}

X42sCommandStatus X42sDriver_GetCommandStatus(const X42sDriver *driver)
{
    return (driver == NULL) ? X42S_COMMAND_STATUS_FORMAT_ERROR :
        driver->command_status;
}

uint8_t X42sDriver_GetCommandFunction(const X42sDriver *driver)
{
    return (driver == NULL) ? 0U : driver->command_expected_function;
}
