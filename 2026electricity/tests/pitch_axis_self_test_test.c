#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "pitch_axis_self_test.h"

typedef enum
{
    FAKE_RESPONSE_NORMAL = 0,
    FAKE_RESPONSE_WRONG_ADDRESS,
    FAKE_RESPONSE_DROP_STATUS,
    FAKE_RESPONSE_BAD_STATUS_CHECKSUM,
    FAKE_RESPONSE_WRITE_ERROR
} FakeResponseMode;

static FakeResponseMode g_response_mode;
static uint32_t g_position_count;

static void queue_response(
    BspUartDmaPort *port,
    const uint8_t *response,
    size_t length)
{
    assert(port != NULL);
    assert(response != NULL);
    assert(length <= sizeof(port->rx_data));

    memcpy(port->rx_data, response, length);
    port->rx_length = length;
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

    if ((port == NULL) || ((data == NULL) && (capacity != 0U)))
    {
        return 0U;
    }

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
    uint8_t response[8] = {0U};

    if ((port == NULL) || (data == NULL) || (length != 3U))
    {
        return BSP_UART_DMA_INVALID_ARGUMENT;
    }
    if (g_response_mode == FAKE_RESPONSE_WRITE_ERROR)
    {
        return BSP_UART_DMA_FULL;
    }

    switch (data[1])
    {
        case 0x3AU:
            if (g_response_mode != FAKE_RESPONSE_DROP_STATUS)
            {
                response[0] =
                    (g_response_mode == FAKE_RESPONSE_WRONG_ADDRESS) ?
                    0x02U : 0x01U;
                response[1] = 0x3AU;
                response[2] = 0x03U;
                response[3] =
                    (g_response_mode == FAKE_RESPONSE_BAD_STATUS_CHECKSUM) ?
                    0x00U : 0x6BU;
                queue_response(port, response, 4U);
            }
            break;

        case 0x36U:
            g_position_count++;
            response[0] = 0x01U;
            response[1] = 0x36U;
            response[2] = 0x00U;
            response[3] = (uint8_t)(g_position_count >> 24U);
            response[4] = (uint8_t)(g_position_count >> 16U);
            response[5] = (uint8_t)(g_position_count >> 8U);
            response[6] = (uint8_t)g_position_count;
            response[7] = 0x6BU;
            queue_response(port, response, 8U);
            break;

        default:
            return BSP_UART_DMA_INVALID_ARGUMENT;
    }

    return BSP_UART_DMA_OK;
}

static void initialize_driver(X42sDriver *driver, UART_HandleTypeDef *uart)
{
    memset(uart, 0, sizeof(*uart));
    assert(X42sDriver_Init(driver, uart) == X42S_DRIVER_OK);
    assert(X42sDriver_Start(driver) == X42S_DRIVER_OK);
}

static uint32_t run_until_terminal(
    PitchAxisSelfTest *self_test,
    uint32_t start_ms,
    uint32_t maximum_iterations)
{
    uint32_t now_ms = start_ms;
    uint32_t iteration;

    for (iteration = 0U; iteration < maximum_iterations; ++iteration)
    {
        PitchAxisSelfTestState state;

        PitchAxisSelfTest_Service(self_test, now_ms);
        state = PitchAxisSelfTest_GetState(self_test);
        if ((state == PITCH_AXIS_SELF_TEST_STATE_COMM_PASS) ||
            (state == PITCH_AXIS_SELF_TEST_STATE_FAILED))
        {
            return now_ms;
        }
        now_ms++;
    }

    assert(false);
    return now_ms;
}

static void test_one_thousand_complete_cycles(void)
{
    UART_HandleTypeDef uart;
    X42sDriver driver;
    PitchAxisSelfTest self_test;
    PitchAxisSelfTestReport report;

    g_response_mode = FAKE_RESPONSE_NORMAL;
    g_position_count = 0U;
    initialize_driver(&driver, &uart);
    assert(PitchAxisSelfTest_Init(
               &self_test,
               &driver,
               X42S_DEFAULT_ADDRESS,
               1000U,
               10U,
               0U) == PITCH_AXIS_SELF_TEST_OK);
    assert(PitchAxisSelfTest_Start(&self_test, 0U) ==
           PITCH_AXIS_SELF_TEST_OK);

    (void)run_until_terminal(&self_test, 0U, 25000U);
    assert(PitchAxisSelfTest_GetState(&self_test) ==
           PITCH_AXIS_SELF_TEST_STATE_COMM_PASS);
    assert(PitchAxisSelfTest_GetReport(&self_test, &report));
    assert(report.completed_cycles == 1000U);
    assert(report.address_echo_valid_count == 2000U);
    assert(report.status_valid_count == 1000U);
    assert(report.position_valid_count == 1000U);
    assert(report.request_error_count == 0U);
    assert(report.protocol_error_count == 0U);
    assert(report.first_status == 0x03U);
    assert(report.last_status == 0x03U);
    assert(report.status_change_count == 0U);
    assert(report.first_position_raw == 1);
    assert(report.minimum_position_raw == 1);
    assert(report.maximum_position_raw == 1000);
    assert(report.latest_position_raw == 1000);
    assert(report.maximum_position_step_raw == 1U);
}

static void test_response_address_mismatch_fails_closed(void)
{
    UART_HandleTypeDef uart;
    X42sDriver driver;
    PitchAxisSelfTest self_test;

    g_response_mode = FAKE_RESPONSE_WRONG_ADDRESS;
    initialize_driver(&driver, &uart);
    assert(PitchAxisSelfTest_Init(
               &self_test,
               &driver,
               X42S_DEFAULT_ADDRESS,
               2U,
               1U,
               0U) == PITCH_AXIS_SELF_TEST_OK);
    assert(PitchAxisSelfTest_Start(&self_test, 0U) ==
           PITCH_AXIS_SELF_TEST_OK);
    (void)run_until_terminal(&self_test, 0U, 20U);

    assert(PitchAxisSelfTest_GetState(&self_test) ==
           PITCH_AXIS_SELF_TEST_STATE_FAILED);
    assert(PitchAxisSelfTest_GetFailure(&self_test) ==
           PITCH_AXIS_SELF_TEST_FAILURE_PROTOCOL);
}

static void test_status_timeout_fails_closed(void)
{
    UART_HandleTypeDef uart;
    X42sDriver driver;
    PitchAxisSelfTest self_test;

    g_response_mode = FAKE_RESPONSE_DROP_STATUS;
    initialize_driver(&driver, &uart);
    assert(PitchAxisSelfTest_Init(
               &self_test,
               &driver,
               X42S_DEFAULT_ADDRESS,
               2U,
               1U,
               0U) == PITCH_AXIS_SELF_TEST_OK);
    assert(PitchAxisSelfTest_Start(&self_test, 0U) ==
           PITCH_AXIS_SELF_TEST_OK);
    (void)run_until_terminal(&self_test, 0U, 200U);

    assert(PitchAxisSelfTest_GetState(&self_test) ==
           PITCH_AXIS_SELF_TEST_STATE_FAILED);
    assert(PitchAxisSelfTest_GetFailure(&self_test) ==
           PITCH_AXIS_SELF_TEST_FAILURE_STATUS_TIMEOUT);
}

static void test_protocol_error_fails_closed(void)
{
    UART_HandleTypeDef uart;
    X42sDriver driver;
    PitchAxisSelfTest self_test;
    PitchAxisSelfTestReport report;

    g_response_mode = FAKE_RESPONSE_BAD_STATUS_CHECKSUM;
    initialize_driver(&driver, &uart);
    assert(PitchAxisSelfTest_Init(
               &self_test,
               &driver,
               X42S_DEFAULT_ADDRESS,
               2U,
               1U,
               0U) == PITCH_AXIS_SELF_TEST_OK);
    assert(PitchAxisSelfTest_Start(&self_test, 0U) ==
           PITCH_AXIS_SELF_TEST_OK);
    (void)run_until_terminal(&self_test, 0U, 20U);

    assert(PitchAxisSelfTest_GetFailure(&self_test) ==
           PITCH_AXIS_SELF_TEST_FAILURE_PROTOCOL);
    assert(PitchAxisSelfTest_GetReport(&self_test, &report));
    assert(report.protocol_error_count > 0U);
}

static void test_tick_wrap_is_bounded(void)
{
    UART_HandleTypeDef uart;
    X42sDriver driver;
    PitchAxisSelfTest self_test;

    g_response_mode = FAKE_RESPONSE_NORMAL;
    g_position_count = 0U;
    initialize_driver(&driver, &uart);
    assert(PitchAxisSelfTest_Init(
               &self_test,
               &driver,
               X42S_DEFAULT_ADDRESS,
               2U,
               10U,
               20U) == PITCH_AXIS_SELF_TEST_OK);
    assert(PitchAxisSelfTest_Start(&self_test, UINT32_MAX - 10U) ==
           PITCH_AXIS_SELF_TEST_OK);
    (void)run_until_terminal(&self_test, UINT32_MAX - 10U, 100U);

    assert(PitchAxisSelfTest_GetState(&self_test) ==
           PITCH_AXIS_SELF_TEST_STATE_COMM_PASS);
}

int main(void)
{
    test_one_thousand_complete_cycles();
    test_response_address_mismatch_fails_closed();
    test_status_timeout_fails_closed();
    test_protocol_error_fails_closed();
    test_tick_wrap_is_bounded();
    return 0;
}
