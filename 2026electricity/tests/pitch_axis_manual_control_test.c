#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "pitch_axis_manual_control.h"

static bool g_drop_response;
static uint8_t g_command_status;
static int64_t g_position;
static uint32_t g_write_count;
static uint32_t g_motion_write_count;
static uint32_t g_stop_write_count;
static uint8_t g_last_motion_mode;

static void queue_bytes(
    BspUartDmaPort *port,
    const uint8_t *data,
    size_t length)
{
    assert(length <= sizeof(port->rx_data));
    memcpy(port->rx_data, data, length);
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

    assert(port != NULL);
    assert(data != NULL);
    assert(length >= 3U);
    g_write_count++;
    if (g_drop_response)
    {
        return BSP_UART_DMA_OK;
    }

    if (data[1] == 0x36U)
    {
        uint64_t magnitude = (g_position < 0) ?
            (uint64_t)(-g_position) : (uint64_t)g_position;
        response[0] = X42S_DEFAULT_ADDRESS;
        response[1] = 0x36U;
        response[2] = (g_position < 0) ? 1U : 0U;
        response[3] = (uint8_t)(magnitude >> 24U);
        response[4] = (uint8_t)(magnitude >> 16U);
        response[5] = (uint8_t)(magnitude >> 8U);
        response[6] = (uint8_t)magnitude;
        response[7] = X42S_FIXED_CHECKSUM;
        queue_bytes(port, response, 8U);
    }
    else
    {
        response[0] = X42S_DEFAULT_ADDRESS;
        response[1] = data[1];
        response[2] = g_command_status;
        response[3] = X42S_FIXED_CHECKSUM;
        queue_bytes(port, response, 4U);

        if ((data[1] == 0xFDU) &&
            (g_command_status == X42S_COMMAND_STATUS_ACCEPTED))
        {
            uint32_t pulses = ((uint32_t)data[6] << 24U) |
                              ((uint32_t)data[7] << 16U) |
                              ((uint32_t)data[8] << 8U) |
                              (uint32_t)data[9];
            g_motion_write_count++;
            g_last_motion_mode = data[10];
            g_position += (data[2] == 0U) ?
                (int64_t)pulses : -(int64_t)pulses;
        }
        else if (data[1] == 0xFEU)
        {
            g_stop_write_count++;
        }
    }

    return BSP_UART_DMA_OK;
}

static PitchAxisManualConfig default_config(void)
{
    PitchAxisManualConfig config;
    memset(&config, 0, sizeof(config));
    config.address = X42S_DEFAULT_ADDRESS;
    config.positive_direction = 0U;
    config.negative_direction = 1U;
    config.speed_rpm = 60U;
    config.acceleration = 100U;
    config.step_pulses = 32U;
    config.motion_mode = 0U;
    config.debounce_ms = 30U;
    config.settle_ms = 1200U;
    return config;
}

static void initialize(
    X42sDriver *driver,
    UART_HandleTypeDef *uart,
    PitchAxisManualControl *control,
    uint32_t now_ms)
{
    PitchAxisManualConfig config = default_config();

    memset(uart, 0, sizeof(*uart));
    g_drop_response = false;
    g_command_status = X42S_COMMAND_STATUS_ACCEPTED;
    g_position = 0;
    g_write_count = 0U;
    g_motion_write_count = 0U;
    g_stop_write_count = 0U;
    g_last_motion_mode = 0xFFU;
    assert(X42sDriver_Init(driver, uart) == X42S_DRIVER_OK);
    assert(X42sDriver_Start(driver) == X42S_DRIVER_OK);
    assert(PitchAxisManualControl_Init(
        control, driver, &config, now_ms));
}

static void service(
    PitchAxisManualControl *control,
    uint32_t now_ms,
    bool key1,
    bool key2,
    bool key3,
    bool key4)
{
    PitchAxisManualButtons buttons = {key1, key2, key3, key4};
    PitchAxisManualControl_Service(control, now_ms, buttons);
}

static uint32_t press_key(
    PitchAxisManualControl *control,
    uint32_t now_ms,
    uint8_t key)
{
    service(control, now_ms, key == 1U, key == 2U, key == 3U, key == 4U);
    now_ms += 30U;
    service(control, now_ms, key == 1U, key == 2U, key == 3U, key == 4U);
    return now_ms;
}

static uint32_t release_all(
    PitchAxisManualControl *control,
    uint32_t now_ms)
{
    service(control, now_ms, false, false, false, false);
    now_ms += 30U;
    service(control, now_ms, false, false, false, false);
    return now_ms;
}

static void test_gate_debounce_enable_and_no_hold_repeat(void)
{
    UART_HandleTypeDef uart;
    X42sDriver driver;
    PitchAxisManualControl control;
    PitchAxisManualReport report;
    uint32_t now_ms = 0U;

    initialize(&driver, &uart, &control, now_ms);
    now_ms = press_key(&control, now_ms + 1U, 1U);
    assert(g_write_count == 0U);
    assert(PitchAxisManualControl_GetReport(&control, &report));
    assert(report.reject_count == 1U);

    now_ms = release_all(&control, now_ms + 1U);
    PitchAxisManualControl_SetCommunicationResult(&control, true, now_ms);
    now_ms = press_key(&control, now_ms + 1U, 1U);
    assert(g_write_count == 1U);
    assert(PitchAxisManualControl_GetReport(&control, &report));
    assert(report.enabled);
    assert(report.state == PITCH_MANUAL_STATE_ENABLED_IDLE);

    service(&control, now_ms + 1000U, true, false, false, false);
    assert(g_write_count == 1U);
}

static void test_positive_and_negative_single_steps(void)
{
    UART_HandleTypeDef uart;
    X42sDriver driver;
    PitchAxisManualControl control;
    PitchAxisManualReport report;
    uint32_t now_ms = 0U;

    initialize(&driver, &uart, &control, now_ms);
    PitchAxisManualControl_SetCommunicationResult(&control, true, now_ms);
    now_ms = press_key(&control, 1U, 1U);
    now_ms = release_all(&control, now_ms + 1U);

    now_ms = press_key(&control, now_ms + 1U, 2U);
    assert(g_motion_write_count == 1U);
    assert(g_last_motion_mode == 0U);
    service(&control, now_ms + 1U, false, true, false, false);
    service(&control, now_ms + 1201U, false, true, false, false);
    service(&control, now_ms + 1202U, false, true, false, false);
    assert(PitchAxisManualControl_GetReport(&control, &report));
    assert(report.state == PITCH_MANUAL_STATE_ENABLED_IDLE);
    assert(report.position_delta == 32);

    now_ms = release_all(&control, now_ms + 1203U);
    now_ms = press_key(&control, now_ms + 1U, 2U);
    assert(g_motion_write_count == 2U);
    assert(g_last_motion_mode == 0U);
    service(&control, now_ms + 1U, false, true, false, false);
    service(&control, now_ms + 1201U, false, true, false, false);
    service(&control, now_ms + 1202U, false, true, false, false);
    assert(PitchAxisManualControl_GetReport(&control, &report));
    assert(report.position_delta == 32);

    now_ms = release_all(&control, now_ms + 1203U);
    now_ms = press_key(&control, now_ms + 1U, 3U);
    assert(g_motion_write_count == 3U);
    assert(g_last_motion_mode == 0U);
    service(&control, now_ms + 1U, false, false, true, false);
    service(&control, now_ms + 1201U, false, false, true, false);
    service(&control, now_ms + 1202U, false, false, true, false);
    assert(PitchAxisManualControl_GetReport(&control, &report));
    assert(report.position_delta == -32);
}

static void test_disabled_conflict_and_stop_priority(void)
{
    UART_HandleTypeDef uart;
    X42sDriver driver;
    PitchAxisManualControl control;
    PitchAxisManualReport report;
    PitchAxisManualButtons conflict = {false, true, true, false};
    uint32_t now_ms = 0U;

    initialize(&driver, &uart, &control, now_ms);
    PitchAxisManualControl_SetCommunicationResult(&control, true, now_ms);
    now_ms = press_key(&control, 1U, 2U);
    assert(g_motion_write_count == 0U);
    now_ms = release_all(&control, now_ms + 1U);

    PitchAxisManualControl_Service(&control, now_ms + 1U, conflict);
    PitchAxisManualControl_Service(&control, now_ms + 31U, conflict);
    assert(PitchAxisManualControl_GetReport(&control, &report));
    assert(report.reject_count == 2U);

    service(&control, now_ms + 32U, true, true, true, true);
    service(&control, now_ms + 62U, true, true, true, true);
    assert(PitchAxisManualControl_GetReport(&control, &report));
    assert(report.fault_latched);
    assert(report.failure == PITCH_MANUAL_FAILURE_STOP_BUTTON);
    assert(g_stop_write_count == 1U);
}

static void test_rejected_ack_and_timeout_latch_stop(void)
{
    UART_HandleTypeDef uart;
    X42sDriver driver;
    PitchAxisManualControl control;
    PitchAxisManualReport report;

    initialize(&driver, &uart, &control, 0U);
    PitchAxisManualControl_SetCommunicationResult(&control, true, 0U);
    g_command_status = X42S_COMMAND_STATUS_PARAMETER_ERROR;
    (void)press_key(&control, 1U, 1U);
    assert(PitchAxisManualControl_GetReport(&control, &report));
    assert(report.fault_latched);
    assert(report.failure == PITCH_MANUAL_FAILURE_COMMAND_REJECTED);
    assert(g_stop_write_count == 1U);

    initialize(&driver, &uart, &control, UINT32_MAX - 40U);
    PitchAxisManualControl_SetCommunicationResult(
        &control, true, UINT32_MAX - 40U);
    g_drop_response = true;
    service(&control, UINT32_MAX - 39U, true, false, false, false);
    service(&control, UINT32_MAX - 9U, true, false, false, false);
    service(&control, 91U, true, false, false, false);
    assert(PitchAxisManualControl_GetReport(&control, &report));
    assert(report.fault_latched);
    assert(report.failure == PITCH_MANUAL_FAILURE_COMMAND_TIMEOUT);
    assert(report.timeout_count == 1U);
}

static void test_busy_rejection_and_debounce_tick_wrap(void)
{
    UART_HandleTypeDef uart;
    X42sDriver driver;
    PitchAxisManualControl control;
    PitchAxisManualReport report;
    uint32_t now_ms;

    initialize(&driver, &uart, &control, 0U);
    PitchAxisManualControl_SetCommunicationResult(&control, true, 0U);
    g_drop_response = true;
    now_ms = press_key(&control, 1U, 1U);
    now_ms = release_all(&control, now_ms + 1U);
    (void)press_key(&control, now_ms + 1U, 2U);
    assert(PitchAxisManualControl_GetReport(&control, &report));
    assert(report.reject_count == 1U);
    assert(g_motion_write_count == 0U);

    initialize(&driver, &uart, &control, UINT32_MAX - 10U);
    PitchAxisManualControl_SetCommunicationResult(
        &control, true, UINT32_MAX - 10U);
    service(&control, UINT32_MAX - 9U, true, false, false, false);
    service(&control, 21U, true, false, false, false);
    assert(PitchAxisManualControl_GetReport(&control, &report));
    assert(report.enabled);
    assert(report.state == PITCH_MANUAL_STATE_ENABLED_IDLE);
}

int main(void)
{
    test_gate_debounce_enable_and_no_hold_repeat();
    test_positive_and_negative_single_steps();
    test_disabled_conflict_and_stop_priority();
    test_rejected_ack_and_timeout_latch_stop();
    test_busy_rejection_and_debounce_tick_wrap();
    puts("PITCH_AXIS_MANUAL_CONTROL_TEST=PASS");
    return 0;
}
