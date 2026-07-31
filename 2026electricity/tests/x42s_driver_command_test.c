#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "x42s_driver.h"

static uint8_t g_last_tx[32];
static size_t g_last_tx_length;
static bool g_drop_response;
static uint8_t g_response_status;

static void queue_response(BspUartDmaPort *port, uint8_t function)
{
    port->rx_data[0] = X42S_DEFAULT_ADDRESS;
    port->rx_data[1] = function;
    port->rx_data[2] = g_response_status;
    port->rx_data[3] = X42S_FIXED_CHECKSUM;
    port->rx_length = X42S_COMMAND_RESPONSE_SIZE;
    port->rx_offset = 0U;
}

BspUartDmaResult BspUartDma_Init(
    BspUartDmaPort *port,
    UART_HandleTypeDef *uart)
{
    if ((port == NULL) || (uart == NULL))
    {
        return BSP_UART_DMA_INVALID_ARGUMENT;
    }
    memset(port, 0, sizeof(*port));
    port->uart = uart;
    return BSP_UART_DMA_OK;
}

BspUartDmaResult BspUartDma_Start(BspUartDmaPort *port)
{
    if ((port == NULL) || (port->uart == NULL))
    {
        return BSP_UART_DMA_INVALID_ARGUMENT;
    }
    port->started = true;
    return BSP_UART_DMA_OK;
}

void BspUartDma_Service(BspUartDmaPort *port)
{
    (void)port;
}

size_t BspUartDma_Read(
    BspUartDmaPort *port,
    uint8_t *data,
    size_t capacity)
{
    size_t count = 0U;

    while ((count < capacity) && (port->rx_offset < port->rx_length))
    {
        data[count++] = port->rx_data[port->rx_offset++];
    }
    if (port->rx_offset == port->rx_length)
    {
        port->rx_offset = 0U;
        port->rx_length = 0U;
    }
    return count;
}

BspUartDmaResult BspUartDma_Write(
    BspUartDmaPort *port,
    const uint8_t *data,
    size_t length)
{
    assert(port != NULL);
    assert(data != NULL);
    assert(length <= sizeof(g_last_tx));

    memcpy(g_last_tx, data, length);
    g_last_tx_length = length;
    if (!g_drop_response)
    {
        queue_response(port, data[1]);
    }
    return BSP_UART_DMA_OK;
}

static void initialize(X42sDriver *driver, UART_HandleTypeDef *uart)
{
    memset(uart, 0, sizeof(*uart));
    memset(g_last_tx, 0, sizeof(g_last_tx));
    g_last_tx_length = 0U;
    g_drop_response = false;
    g_response_status = X42S_COMMAND_STATUS_ACCEPTED;
    assert(X42sDriver_Init(driver, uart) == X42S_DRIVER_OK);
    assert(X42sDriver_Start(driver) == X42S_DRIVER_OK);
}

static void test_enable_ack(void)
{
    UART_HandleTypeDef uart;
    X42sDriver driver;
    static const uint8_t expected[] = {0x01U, 0xF3U, 0xABU, 0x01U, 0x00U, 0x6BU};

    initialize(&driver, &uart);
    assert(X42sDriver_RequestEnable(
        &driver, X42S_DEFAULT_ADDRESS, true, false, 10U) == X42S_DRIVER_OK);
    assert(g_last_tx_length == sizeof(expected));
    assert(memcmp(g_last_tx, expected, sizeof(expected)) == 0);
    assert(X42sDriver_GetState(&driver) ==
           X42S_DRIVER_STATE_WAITING_FOR_COMMAND);

    X42sDriver_Service(&driver, 11U);
    assert(X42sDriver_GetState(&driver) == X42S_DRIVER_STATE_COMMAND_VALID);
    assert(X42sDriver_GetCommandFunction(&driver) == 0xF3U);
    assert(X42sDriver_GetCommandStatus(&driver) ==
           X42S_COMMAND_STATUS_ACCEPTED);
    assert(driver.valid_command_response_count == 1U);
}

static void test_position_command_and_error_status(void)
{
    UART_HandleTypeDef uart;
    X42sDriver driver;
    static const uint8_t expected[] = {
        0x01U, 0xFDU, 0x01U, 0x00U, 0x3CU, 0x64U, 0x00U,
        0x00U, 0x00U, 0x20U, 0x02U, 0x00U, 0x6BU
    };

    initialize(&driver, &uart);
    g_response_status = X42S_COMMAND_STATUS_PARAMETER_ERROR;
    assert(X42sDriver_RequestEmmPosition(
        &driver,
        X42S_DEFAULT_ADDRESS,
        1U,
        60U,
        100U,
        32U,
        2U,
        false,
        20U) == X42S_DRIVER_OK);
    assert(g_last_tx_length == sizeof(expected));
    assert(memcmp(g_last_tx, expected, sizeof(expected)) == 0);

    X42sDriver_Service(&driver, 21U);
    assert(X42sDriver_GetState(&driver) == X42S_DRIVER_STATE_COMMAND_VALID);
    assert(X42sDriver_GetCommandStatus(&driver) ==
           X42S_COMMAND_STATUS_PARAMETER_ERROR);
}

static void test_velocity_command_and_ack(void)
{
    UART_HandleTypeDef uart;
    X42sDriver driver;
    static const uint8_t expected[] = {
        0x01U, 0xF6U, 0x00U, 0x00U, 0x3CU, 0x64U, 0x00U, 0x6BU
    };

    initialize(&driver, &uart);
    assert(X42sDriver_RequestEmmVelocity(
        &driver,
        X42S_DEFAULT_ADDRESS,
        0U,
        60U,
        100U,
        false,
        22U) == X42S_DRIVER_OK);
    assert(g_last_tx_length == sizeof(expected));
    assert(memcmp(g_last_tx, expected, sizeof(expected)) == 0);
    assert(X42sDriver_GetState(&driver) ==
           X42S_DRIVER_STATE_WAITING_FOR_COMMAND);

    X42sDriver_Service(&driver, 23U);
    assert(X42sDriver_GetState(&driver) == X42S_DRIVER_STATE_COMMAND_VALID);
    assert(X42sDriver_GetCommandFunction(&driver) == 0xF6U);
    assert(X42sDriver_GetCommandStatus(&driver) ==
           X42S_COMMAND_STATUS_ACCEPTED);
}

static void test_stop_supersedes_pending_command(void)
{
    UART_HandleTypeDef uart;
    X42sDriver driver;
    static const uint8_t expected_stop[] = {0x01U, 0xFEU, 0x98U, 0x00U, 0x6BU};

    initialize(&driver, &uart);
    g_drop_response = true;
    assert(X42sDriver_RequestEnable(
        &driver, X42S_DEFAULT_ADDRESS, true, false, 30U) == X42S_DRIVER_OK);
    assert(X42sDriver_RequestEnable(
        &driver, X42S_DEFAULT_ADDRESS, false, false, 31U) == X42S_DRIVER_BUSY);

    g_drop_response = false;
    assert(X42sDriver_RequestStop(
        &driver, X42S_DEFAULT_ADDRESS, false, 32U) == X42S_DRIVER_OK);
    assert(g_last_tx_length == sizeof(expected_stop));
    assert(memcmp(g_last_tx, expected_stop, sizeof(expected_stop)) == 0);
    X42sDriver_Service(&driver, 33U);
    assert(X42sDriver_GetState(&driver) == X42S_DRIVER_STATE_COMMAND_VALID);
    assert(X42sDriver_GetCommandFunction(&driver) == 0xFEU);
}

static void test_late_superseded_velocity_ack_is_not_protocol_fault(void)
{
    UART_HandleTypeDef uart;
    X42sDriver driver;
    uint8_t responses[8] = {
        X42S_DEFAULT_ADDRESS, 0xF6U, X42S_COMMAND_STATUS_ACCEPTED,
        X42S_FIXED_CHECKSUM,
        X42S_DEFAULT_ADDRESS, 0xFEU, X42S_COMMAND_STATUS_ACCEPTED,
        X42S_FIXED_CHECKSUM
    };

    initialize(&driver, &uart);
    assert(X42sDriver_RequestEmmVelocity(
        &driver,
        X42S_DEFAULT_ADDRESS,
        0U,
        1U,
        100U,
        false,
        40U) == X42S_DRIVER_OK);
    /* Simulate the old ACK remaining in the receive stream while STOP
     * replaces the command transaction. */
    g_drop_response = true;
    assert(X42sDriver_RequestStop(
        &driver,
        X42S_DEFAULT_ADDRESS,
        false,
        41U) == X42S_DRIVER_OK);
    memcpy(driver.transport.rx_data, responses, sizeof(responses));
    driver.transport.rx_length = sizeof(responses);
    driver.transport.rx_offset = 0U;
    X42sDriver_Service(&driver, 42U);

    assert(X42sDriver_GetState(&driver) ==
           X42S_DRIVER_STATE_COMMAND_VALID);
    assert(X42sDriver_GetCommandFunction(&driver) == 0xFEU);
    assert(driver.superseded_response_count == 1U);
    assert(driver.protocol_error_count == 0U);
}

static void test_stop_ack_before_superseded_velocity_ack_is_drained(void)
{
    UART_HandleTypeDef uart;
    X42sDriver driver;
    uint8_t responses[8] = {
        X42S_DEFAULT_ADDRESS, 0xFEU, X42S_COMMAND_STATUS_ACCEPTED,
        X42S_FIXED_CHECKSUM,
        X42S_DEFAULT_ADDRESS, 0xF6U, X42S_COMMAND_STATUS_ACCEPTED,
        X42S_FIXED_CHECKSUM
    };

    initialize(&driver, &uart);
    g_drop_response = true;
    assert(X42sDriver_RequestEmmVelocity(
        &driver,
        X42S_DEFAULT_ADDRESS,
        0U,
        1U,
        100U,
        false,
        40U) == X42S_DRIVER_OK);
    assert(X42sDriver_RequestStop(
        &driver,
        X42S_DEFAULT_ADDRESS,
        false,
        41U) == X42S_DRIVER_OK);
    memcpy(driver.transport.rx_data, responses, sizeof(responses));
    driver.transport.rx_length = sizeof(responses);
    driver.transport.rx_offset = 0U;
    X42sDriver_Service(&driver, 42U);

    assert(X42sDriver_GetState(&driver) ==
           X42S_DRIVER_STATE_COMMAND_VALID);
    assert(X42sDriver_GetCommandFunction(&driver) == 0xFEU);
    assert(driver.superseded_response_count == 1U);
    assert(!driver.superseded_command_pending);
    assert(driver.transport.rx_length == 0U);
    assert(driver.protocol_error_count == 0U);
}

static void test_superseded_ack_is_kept_across_next_command(void)
{
    UART_HandleTypeDef uart;
    X42sDriver driver;
    static const uint8_t stop_response[4] = {
        X42S_DEFAULT_ADDRESS, 0xFEU, X42S_COMMAND_STATUS_ACCEPTED,
        X42S_FIXED_CHECKSUM
    };
    static const uint8_t delayed_and_current_responses[8] = {
        X42S_DEFAULT_ADDRESS, 0xF6U, X42S_COMMAND_STATUS_ACCEPTED,
        X42S_FIXED_CHECKSUM,
        X42S_DEFAULT_ADDRESS, 0xF3U, X42S_COMMAND_STATUS_ACCEPTED,
        X42S_FIXED_CHECKSUM
    };

    initialize(&driver, &uart);
    g_drop_response = true;
    assert(X42sDriver_RequestEmmVelocity(
        &driver,
        X42S_DEFAULT_ADDRESS,
        0U,
        1U,
        100U,
        false,
        40U) == X42S_DRIVER_OK);
    assert(X42sDriver_RequestStop(
        &driver,
        X42S_DEFAULT_ADDRESS,
        false,
        41U) == X42S_DRIVER_OK);
    memcpy(driver.transport.rx_data, stop_response, sizeof(stop_response));
    driver.transport.rx_length = sizeof(stop_response);
    driver.transport.rx_offset = 0U;
    X42sDriver_Service(&driver, 42U);
    assert(driver.superseded_command_pending);

    assert(X42sDriver_RequestEnable(
        &driver,
        X42S_DEFAULT_ADDRESS,
        true,
        false,
        43U) == X42S_DRIVER_OK);
    memcpy(driver.transport.rx_data,
           delayed_and_current_responses,
           sizeof(delayed_and_current_responses));
    driver.transport.rx_length = sizeof(delayed_and_current_responses);
    driver.transport.rx_offset = 0U;
    X42sDriver_Service(&driver, 44U);

    assert(X42sDriver_GetState(&driver) ==
           X42S_DRIVER_STATE_COMMAND_VALID);
    assert(X42sDriver_GetCommandFunction(&driver) == 0xF3U);
    assert(driver.superseded_response_count == 1U);
    assert(!driver.superseded_command_pending);
    assert(driver.protocol_error_count == 0U);
}

static void test_missing_superseded_ack_expires(void)
{
    UART_HandleTypeDef uart;
    X42sDriver driver;
    static const uint8_t stop_response[4] = {
        X42S_DEFAULT_ADDRESS, 0xFEU, X42S_COMMAND_STATUS_ACCEPTED,
        X42S_FIXED_CHECKSUM
    };

    initialize(&driver, &uart);
    g_drop_response = true;
    assert(X42sDriver_RequestEmmVelocity(
        &driver,
        X42S_DEFAULT_ADDRESS,
        0U,
        1U,
        100U,
        false,
        40U) == X42S_DRIVER_OK);
    assert(X42sDriver_RequestStop(
        &driver,
        X42S_DEFAULT_ADDRESS,
        false,
        41U) == X42S_DRIVER_OK);
    memcpy(driver.transport.rx_data, stop_response, sizeof(stop_response));
    driver.transport.rx_length = sizeof(stop_response);
    driver.transport.rx_offset = 0U;
    X42sDriver_Service(&driver, 42U);
    assert(driver.superseded_command_pending);

    X42sDriver_Service(
        &driver,
        41U + X42S_DRIVER_COMMAND_TIMEOUT_MS);
    assert(!driver.superseded_command_pending);
    assert(driver.protocol_error_count == 0U);
}

static void test_corrupt_superseded_ack_remains_protocol_fault(void)
{
    UART_HandleTypeDef uart;
    X42sDriver driver;
    uint8_t corrupt_response[4] = {
        X42S_DEFAULT_ADDRESS, 0xF6U, X42S_COMMAND_STATUS_ACCEPTED, 0x00U
    };

    initialize(&driver, &uart);
    assert(X42sDriver_RequestEmmVelocity(
        &driver,
        X42S_DEFAULT_ADDRESS,
        0U,
        1U,
        100U,
        false,
        40U) == X42S_DRIVER_OK);
    g_drop_response = true;
    assert(X42sDriver_RequestStop(
        &driver,
        X42S_DEFAULT_ADDRESS,
        false,
        41U) == X42S_DRIVER_OK);
    memcpy(driver.transport.rx_data,
           corrupt_response,
           sizeof(corrupt_response));
    driver.transport.rx_length = sizeof(corrupt_response);
    driver.transport.rx_offset = 0U;
    X42sDriver_Service(&driver, 42U);

    assert(X42sDriver_GetState(&driver) ==
           X42S_DRIVER_STATE_WAITING_FOR_COMMAND);
    assert(driver.superseded_response_count == 0U);
    assert(driver.protocol_error_count == 1U);
}

static void test_split_superseded_velocity_ack_is_not_protocol_fault(void)
{
    static const uint8_t old_response[4] = {
        X42S_DEFAULT_ADDRESS, 0xF6U, X42S_COMMAND_STATUS_ACCEPTED,
        X42S_FIXED_CHECKSUM
    };
    static const uint8_t stop_response[4] = {
        X42S_DEFAULT_ADDRESS, 0xFEU, X42S_COMMAND_STATUS_ACCEPTED,
        X42S_FIXED_CHECKSUM
    };
    uint8_t response_stream[7];
    uint8_t split;

    for (split = 1U; split < X42S_COMMAND_RESPONSE_SIZE; ++split)
    {
        UART_HandleTypeDef uart;
        X42sDriver driver;

        initialize(&driver, &uart);
        g_drop_response = true;
        assert(X42sDriver_RequestEmmVelocity(
            &driver,
            X42S_DEFAULT_ADDRESS,
            0U,
            1U,
            100U,
            false,
            40U) == X42S_DRIVER_OK);
        memcpy(driver.command_response_window, old_response, split);
        driver.command_response_window_length = split;
        assert(X42sDriver_RequestStop(
            &driver,
            X42S_DEFAULT_ADDRESS,
            false,
            41U) == X42S_DRIVER_OK);
        memcpy(response_stream, old_response + split,
               X42S_COMMAND_RESPONSE_SIZE - split);
        memcpy(response_stream + X42S_COMMAND_RESPONSE_SIZE - split,
               stop_response,
               sizeof(stop_response));
        memcpy(driver.transport.rx_data, response_stream,
               sizeof(response_stream));
        driver.transport.rx_length = sizeof(response_stream);
        driver.transport.rx_offset = 0U;
        X42sDriver_Service(&driver, 42U);

        assert(X42sDriver_GetState(&driver) ==
               X42S_DRIVER_STATE_COMMAND_VALID);
        assert(X42sDriver_GetCommandFunction(&driver) == 0xFEU);
        assert(driver.superseded_response_count == 1U);
        assert(driver.protocol_error_count == 0U);
    }
}

static void test_command_timeout_handles_tick_wrap(void)
{
    UART_HandleTypeDef uart;
    X42sDriver driver;

    initialize(&driver, &uart);
    g_drop_response = true;
    assert(X42sDriver_RequestEnable(
        &driver,
        X42S_DEFAULT_ADDRESS,
        true,
        false,
        UINT32_MAX - 50U) == X42S_DRIVER_OK);
    X42sDriver_Service(&driver, 48U);
    assert(X42sDriver_GetState(&driver) ==
           X42S_DRIVER_STATE_WAITING_FOR_COMMAND);
    X42sDriver_Service(&driver, 49U);
    assert(X42sDriver_GetState(&driver) ==
           X42S_DRIVER_STATE_COMMAND_TIMEOUT);
    assert(driver.command_timeout_count == 1U);
}

int main(void)
{
    test_enable_ack();
    test_position_command_and_error_status();
    test_velocity_command_and_ack();
    test_stop_supersedes_pending_command();
    test_late_superseded_velocity_ack_is_not_protocol_fault();
    test_stop_ack_before_superseded_velocity_ack_is_drained();
    test_superseded_ack_is_kept_across_next_command();
    test_missing_superseded_ack_expires();
    test_corrupt_superseded_ack_remains_protocol_fault();
    test_split_superseded_velocity_ack_is_not_protocol_fault();
    test_command_timeout_handles_tick_wrap();
    puts("X42S_DRIVER_COMMAND_TEST=PASS");
    return 0;
}
