#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "pitch_axis_velocity_test.h"

static bool g_drop_response;
static uint8_t g_command_status;
static int64_t g_position;
static uint32_t g_velocity_write_count;
static uint32_t g_stop_write_count;
static uint8_t g_velocity_direction;
static uint16_t g_velocity_speed_rpm;
static bool g_velocity_pending;

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
        queue_bytes(port, response, sizeof(response));
        return BSP_UART_DMA_OK;
    }

    if ((data[1] == 0xF6U) &&
        (g_command_status == X42S_COMMAND_STATUS_ACCEPTED))
    {
        g_velocity_write_count++;
        g_velocity_direction = data[2];
        g_velocity_speed_rpm =
            (uint16_t)(((uint16_t)data[3] << 8U) | data[4]);
        g_velocity_pending = true;
    }
    else if (data[1] == 0xFEU)
    {
        g_stop_write_count++;
        if ((g_command_status == X42S_COMMAND_STATUS_ACCEPTED) &&
            g_velocity_pending)
        {
            g_position += (g_velocity_direction == 0U) ? 400 : -400;
            g_velocity_pending = false;
        }
    }

    response[0] = X42S_DEFAULT_ADDRESS;
    response[1] = data[1];
    response[2] = g_command_status;
    response[3] = X42S_FIXED_CHECKSUM;
    queue_bytes(port, response, 4U);
    return BSP_UART_DMA_OK;
}

static PitchAxisVelocityTestConfig default_config(void)
{
    PitchAxisVelocityTestConfig config;

    memset(&config, 0, sizeof(config));
    config.address = X42S_DEFAULT_ADDRESS;
    config.positive_direction = 0U;
    config.negative_direction = 1U;
    config.speed_rpm = 10U;
    config.acceleration = 100U;
    config.run_ms = 300U;
    config.debounce_ms = 30U;
    config.automatic_max_speed_rpm = 1U;
    config.automatic_decision_timeout_ms = 200U;
    config.automatic_vision_loss_grace_ms = 0U;
    config.automatic_motion_budget_ms = 3000U;
    return config;
}

static void initialize(
    X42sDriver *driver,
    UART_HandleTypeDef *uart,
    PitchAxisVelocityTest *test,
    uint32_t now_ms)
{
    PitchAxisVelocityTestConfig config = default_config();

    memset(uart, 0, sizeof(*uart));
    g_drop_response = false;
    g_command_status = X42S_COMMAND_STATUS_ACCEPTED;
    g_position = 0;
    g_velocity_write_count = 0U;
    g_stop_write_count = 0U;
    g_velocity_direction = 0U;
    g_velocity_speed_rpm = 0U;
    g_velocity_pending = false;
    assert(X42sDriver_Init(driver, uart) == X42S_DRIVER_OK);
    assert(X42sDriver_Start(driver) == X42S_DRIVER_OK);
    assert(PitchAxisVelocityTest_Init(test, driver, &config, now_ms));
}

static void service(
    PitchAxisVelocityTest *test,
    uint32_t now_ms,
    bool key1,
    bool key2,
    bool key3,
    bool key4)
{
    PitchAxisVelocityTestButtons buttons = {key1, key2, key3, key4};

    PitchAxisVelocityTest_Service(test, now_ms, buttons);
}

static uint32_t press_key(
    PitchAxisVelocityTest *test,
    uint32_t now_ms,
    uint8_t key)
{
    service(test, now_ms, key == 1U, key == 2U, key == 3U, key == 4U);
    now_ms += 30U;
    service(test, now_ms, key == 1U, key == 2U, key == 3U, key == 4U);
    return now_ms;
}

static uint32_t release_all(PitchAxisVelocityTest *test, uint32_t now_ms)
{
    service(test, now_ms, false, false, false, false);
    now_ms += 30U;
    service(test, now_ms, false, false, false, false);
    return now_ms;
}

static uint32_t enable_test(
    PitchAxisVelocityTest *test,
    uint32_t now_ms)
{
    PitchAxisVelocityTestReport report;

    PitchAxisVelocityTest_SetCommunicationResult(test, true, now_ms);
    assert(PitchAxisVelocityTest_GetReport(test, &report));
    assert(!report.enabled);
    assert(!report.automatic_armed);
    assert(report.state == PITCH_VELOCITY_TEST_STATE_DISABLED_READY);
    assert(PitchAxisVelocityTest_SetAutomaticArmed(test, true, now_ms));
    service(test, now_ms + 1U, false, false, false, false);
    assert(PitchAxisVelocityTest_GetReport(test, &report));
    assert(report.enabled);
    assert(report.state == PITCH_VELOCITY_TEST_STATE_ENABLED_STOPPED);
    assert(report.automatic_armed);
    return now_ms + 1U;
}

static void test_self_test_pass_waits_for_explicit_start(void)
{
    UART_HandleTypeDef uart;
    X42sDriver driver;
    PitchAxisVelocityTest test;
    PitchAxisVelocityTestReport report;

    initialize(&driver, &uart, &test, 0U);
    PitchAxisVelocityTest_SetCommunicationResult(&test, true, 10U);
    service(&test, 11U, false, false, false, false);

    assert(PitchAxisVelocityTest_GetReport(&test, &report));
    assert(report.communication_ready);
    assert(!report.enabled);
    assert(!report.automatic_armed);
    assert(report.state == PITCH_VELOCITY_TEST_STATE_DISABLED_READY);
    assert(report.command_count == 0U);
}

static void complete_timed_motion(
    PitchAxisVelocityTest *test,
    uint32_t start_ms)
{
    service(test, start_ms + 1U, false, true, false, false);
    service(test, start_ms + 301U, false, true, false, false);
    service(test, start_ms + 302U, false, true, false, false);
    service(test, start_ms + 303U, false, true, false, false);
}

static void test_gate_enable_and_timed_positive_motion(void)
{
    UART_HandleTypeDef uart;
    X42sDriver driver;
    PitchAxisVelocityTest test;
    PitchAxisVelocityTestReport report;
    uint32_t now_ms = 0U;

    initialize(&driver, &uart, &test, now_ms);
    now_ms = press_key(&test, now_ms + 1U, 1U);
    assert(PitchAxisVelocityTest_GetReport(&test, &report));
    assert(report.reject_count == 1U);
    assert(!report.enabled);

    now_ms = release_all(&test, now_ms + 1U);
    now_ms = enable_test(&test, now_ms + 1U);
    now_ms = press_key(&test, now_ms + 1U, 1U);
    assert(PitchAxisVelocityTest_GetReport(&test, &report));
    assert(report.enabled);
    assert(report.automatic_armed);
    assert(report.command_count == 1U);
    now_ms = release_all(&test, now_ms + 1U);
    assert(PitchAxisVelocityTest_SetAutomaticArmed(&test, false, now_ms));
    now_ms = press_key(&test, now_ms + 1U, 2U);
    assert(g_velocity_write_count == 1U);
    assert(g_velocity_direction == 0U);

    complete_timed_motion(&test, now_ms);
    assert(PitchAxisVelocityTest_GetReport(&test, &report));
    assert(report.state == PITCH_VELOCITY_TEST_STATE_ENABLED_STOPPED);
    assert(report.position_delta == 400);
    assert(!report.velocity_command_active);
    assert(g_stop_write_count == 1U);
}

static void test_negative_motion_and_stop_button(void)
{
    UART_HandleTypeDef uart;
    X42sDriver driver;
    PitchAxisVelocityTest test;
    PitchAxisVelocityTestReport report;
    uint32_t now_ms = 0U;

    initialize(&driver, &uart, &test, now_ms);
    now_ms = enable_test(&test, now_ms);
    assert(PitchAxisVelocityTest_SetAutomaticArmed(&test, false, now_ms));
    now_ms = press_key(&test, now_ms + 1U, 3U);
    assert(g_velocity_write_count == 1U);
    assert(g_velocity_direction == 1U);
    complete_timed_motion(&test, now_ms);
    assert(PitchAxisVelocityTest_GetReport(&test, &report));
    assert(report.position_delta == -400);

    now_ms = release_all(&test, now_ms + 400U);
    now_ms = press_key(&test, now_ms + 1U, 2U);
    service(&test, now_ms + 1U, false, true, false, false);
    now_ms = press_key(&test, now_ms + 2U, 4U);
    service(&test, now_ms + 1U, false, false, false, true);
    assert(PitchAxisVelocityTest_GetReport(&test, &report));
    assert(report.fault_latched);
    assert(report.failure == PITCH_VELOCITY_TEST_FAILURE_STOP_BUTTON);
    assert(g_stop_write_count == 2U);
}

static void test_rejected_velocity_ack_latches_stop(void)
{
    UART_HandleTypeDef uart;
    X42sDriver driver;
    PitchAxisVelocityTest test;
    PitchAxisVelocityTestReport report;
    uint32_t now_ms = 0U;

    initialize(&driver, &uart, &test, now_ms);
    now_ms = enable_test(&test, now_ms);
    assert(PitchAxisVelocityTest_SetAutomaticArmed(&test, false, now_ms));
    g_command_status = X42S_COMMAND_STATUS_PARAMETER_ERROR;
    now_ms = press_key(&test, now_ms + 1U, 2U);
    service(&test, now_ms + 1U, false, true, false, false);
    service(&test, now_ms + 2U, false, true, false, false);
    assert(PitchAxisVelocityTest_GetReport(&test, &report));
    assert(report.fault_latched);
    assert(report.failure == PITCH_VELOCITY_TEST_FAILURE_COMMAND_REJECTED);
    assert(g_stop_write_count == 1U);
}

static void test_stop_button_when_disabled_does_not_wait_for_ack(void)
{
    UART_HandleTypeDef uart;
    X42sDriver driver;
    PitchAxisVelocityTest test;
    PitchAxisVelocityTestReport report;
    uint32_t now_ms;

    initialize(&driver, &uart, &test, 0U);
    now_ms = press_key(&test, 1U, 4U);
    (void)now_ms;
    assert(PitchAxisVelocityTest_GetReport(&test, &report));
    assert(report.fault_latched);
    assert(report.failure == PITCH_VELOCITY_TEST_FAILURE_STOP_BUTTON);
    assert(report.state == PITCH_VELOCITY_TEST_STATE_FAULT_LATCHED);
    assert(g_stop_write_count == 0U);
}

static PitchAxisAutomaticDecision automatic_decision(
    bool safe,
    bool motion,
    uint8_t direction,
    uint16_t speed_rpm,
    uint8_t sequence)
{
    PitchAxisAutomaticDecision decision;

    memset(&decision, 0, sizeof(decision));
    decision.source_safe = safe;
    decision.motion_requested = motion;
    decision.motor_direction = direction;
    decision.speed_rpm = speed_rpm;
    decision.outer_control_0_01rpm =
        (direction == 0U) ? -(int16_t)(speed_rpm * 100U) :
                            (int16_t)(speed_rpm * 100U);
    decision.sequence = sequence;
    decision.unsafe_reason = PITCH_AUTOMATIC_DISARM_VISION_INVALID;
    return decision;
}

static void test_position_loop_bounds_target_and_stops_before_reversal(void)
{
    UART_HandleTypeDef uart;
    X42sDriver driver;
    PitchAxisVelocityTest test;
    PitchAxisVelocityTestConfig config = default_config();
    PitchAxisVelocityTestReport report;
    PitchAxisAutomaticDecision decision;
    uint32_t now_ms = 0U;

    memset(&uart, 0, sizeof(uart));
    g_drop_response = false;
    g_command_status = X42S_COMMAND_STATUS_ACCEPTED;
    g_position = 0;
    g_velocity_write_count = 0U;
    g_stop_write_count = 0U;
    g_velocity_pending = false;
    config.automatic_max_speed_rpm = 100U;
    config.automatic_position_tracking_enabled = true;
    config.automatic_direction0_increases_raw = true;
    config.automatic_position_raw_per_mm = 1000U;
    config.automatic_tilt_scale_um_per_outer_rpm = 20U;
    config.automatic_tilt_limit_um = 500U;
    config.automatic_position_deadband_um = 20U;
    config.automatic_position_slow_zone_um = 200U;
    config.automatic_position_min_speed_rpm = 10U;
    config.automatic_position_poll_period_ms = 20U;
    assert(X42sDriver_Init(&driver, &uart) == X42S_DRIVER_OK);
    assert(X42sDriver_Start(&driver) == X42S_DRIVER_OK);
    assert(PitchAxisVelocityTest_Init(&test, &driver, &config, now_ms));
    now_ms = enable_test(&test, now_ms);

    decision = automatic_decision(true, true, 0U, 100U, 1U);
    decision.outer_control_0_01rpm = -30000;
    PitchAxisVelocityTest_SubmitAutomaticDecision(
        &test, &decision, now_ms + 2U);
    service(&test, now_ms + 2U, false, false, false, false);
    service(&test, now_ms + 3U, false, false, false, false);
    assert(g_velocity_write_count == 1U);
    assert(g_velocity_direction == 0U);
    assert(g_velocity_speed_rpm == 100U);
    service(&test, now_ms + 4U, false, false, false, false);
    assert(PitchAxisVelocityTest_GetReport(&test, &report));
    assert(report.automatic_target_offset_raw == 500);

    g_position = 100;
    decision = automatic_decision(true, true, 1U, 100U, 2U);
    decision.outer_control_0_01rpm = 30000;
    PitchAxisVelocityTest_SubmitAutomaticDecision(
        &test, &decision, now_ms + 5U);
    service(&test, now_ms + 5U, false, false, false, false);
    service(&test, now_ms + 6U, false, false, false, false);
    assert(g_stop_write_count == 1U);
    assert(g_velocity_write_count == 1U);
    service(&test, now_ms + 7U, false, false, false, false);
    assert(g_velocity_write_count == 2U);
    assert(g_velocity_direction == 1U);
    assert(PitchAxisVelocityTest_GetReport(&test, &report));
    assert(report.automatic_target_offset_raw == -500);

    g_position = -490;
    service(&test, now_ms + 27U, false, false, false, false);
    service(&test, now_ms + 28U, false, false, false, false);
    assert(g_stop_write_count == 2U);
    service(&test, now_ms + 29U, false, false, false, false);
    assert(PitchAxisVelocityTest_GetReport(&test, &report));
    assert(report.state == PITCH_VELOCITY_TEST_STATE_ENABLED_STOPPED);
    assert(!report.automatic_motion_active);
    assert(report.automatic_position_error_raw == -10);
    assert(!report.fault_latched);
}

static void test_automatic_clamps_speed_and_stops_on_unsafe_vision(void)
{
    UART_HandleTypeDef uart;
    X42sDriver driver;
    PitchAxisVelocityTest test;
    PitchAxisVelocityTestReport report;
    PitchAxisAutomaticDecision decision;
    uint32_t now_ms = 0U;

    initialize(&driver, &uart, &test, now_ms);
    now_ms = enable_test(&test, now_ms);
    assert(PitchAxisVelocityTest_SetAutomaticArmed(
        &test, true, now_ms + 1U));

    decision = automatic_decision(true, false, 0U, 0U, 1U);
    PitchAxisVelocityTest_SubmitAutomaticDecision(
        &test, &decision, now_ms + 2U);
    service(&test, now_ms + 2U, false, false, false, false);
    assert(g_velocity_write_count == 0U);

    decision = automatic_decision(true, true, 0U, 5U, 2U);
    PitchAxisVelocityTest_SubmitAutomaticDecision(
        &test, &decision, now_ms + 3U);
    service(&test, now_ms + 3U, false, false, false, false);
    assert(g_velocity_write_count == 1U);
    assert(g_velocity_direction == 0U);
    assert(g_velocity_speed_rpm == 1U);
    service(&test, now_ms + 4U, false, false, false, false);

    decision = automatic_decision(true, true, 0U, 5U, 3U);
    PitchAxisVelocityTest_SubmitAutomaticDecision(
        &test, &decision, now_ms + 5U);
    service(&test, now_ms + 5U, false, false, false, false);
    assert(g_velocity_write_count == 1U);

    decision = automatic_decision(false, false, 0U, 0U, 4U);
    PitchAxisVelocityTest_SubmitAutomaticDecision(
        &test, &decision, now_ms + 6U);
    service(&test, now_ms + 6U, false, false, false, false);
    assert(g_stop_write_count == 1U);
    assert(PitchAxisVelocityTest_GetReport(&test, &report));
    assert(!report.automatic_armed);
    assert(report.automatic_disarm_reason ==
           PITCH_AUTOMATIC_DISARM_VISION_INVALID);
    service(&test, now_ms + 7U, false, false, false, false);
    assert(PitchAxisVelocityTest_GetReport(&test, &report));
    assert(report.state == PITCH_VELOCITY_TEST_STATE_ENABLED_STOPPED);
    assert(!report.velocity_command_active);
}

static void test_automatic_reversal_stops_before_new_direction(void)
{
    UART_HandleTypeDef uart;
    X42sDriver driver;
    PitchAxisVelocityTest test;
    PitchAxisAutomaticDecision decision;
    uint32_t now_ms = 0U;

    initialize(&driver, &uart, &test, now_ms);
    now_ms = enable_test(&test, now_ms);
    assert(PitchAxisVelocityTest_SetAutomaticArmed(
        &test, true, now_ms + 1U));

    decision = automatic_decision(true, true, 0U, 1U, 1U);
    PitchAxisVelocityTest_SubmitAutomaticDecision(
        &test, &decision, now_ms + 2U);
    service(&test, now_ms + 2U, false, false, false, false);
    service(&test, now_ms + 3U, false, false, false, false);
    assert(g_velocity_write_count == 1U);

    decision = automatic_decision(true, true, 1U, 1U, 2U);
    PitchAxisVelocityTest_SubmitAutomaticDecision(
        &test, &decision, now_ms + 4U);
    service(&test, now_ms + 4U, false, false, false, false);
    assert(g_stop_write_count == 1U);
    assert(g_velocity_write_count == 1U);

    service(&test, now_ms + 5U, false, false, false, false);
    assert(g_velocity_write_count == 2U);
    assert(g_velocity_direction == 1U);
}

static void test_same_direction_speed_update_does_not_stop(void)
{
    UART_HandleTypeDef uart;
    X42sDriver driver;
    PitchAxisVelocityTest test;
    PitchAxisVelocityTestConfig config;
    PitchAxisVelocityTestReport report;
    PitchAxisAutomaticDecision decision;
    uint32_t now_ms = 0U;

    initialize(&driver, &uart, &test, now_ms);
    now_ms = enable_test(&test, now_ms);
    assert(PitchAxisVelocityTest_GetConfig(&test, &config));
    config.automatic_max_speed_rpm = 5U;
    assert(PitchAxisVelocityTest_UpdateConfig(&test, &config));
    assert(PitchAxisVelocityTest_SetAutomaticArmed(
        &test, true, now_ms + 1U));

    decision = automatic_decision(true, true, 0U, 2U, 1U);
    PitchAxisVelocityTest_SubmitAutomaticDecision(
        &test, &decision, now_ms + 2U);
    service(&test, now_ms + 2U, false, false, false, false);
    service(&test, now_ms + 3U, false, false, false, false);
    assert(g_velocity_write_count == 1U);
    assert(g_velocity_speed_rpm == 2U);

    decision = automatic_decision(true, true, 0U, 4U, 2U);
    PitchAxisVelocityTest_SubmitAutomaticDecision(
        &test, &decision, now_ms + 4U);
    service(&test, now_ms + 4U, false, false, false, false);
    assert(g_velocity_write_count == 2U);
    assert(g_velocity_speed_rpm == 4U);
    assert(g_stop_write_count == 0U);
    service(&test, now_ms + 5U, false, false, false, false);
    assert(PitchAxisVelocityTest_GetReport(&test, &report));
    assert(report.state == PITCH_VELOCITY_TEST_STATE_RUNNING_AUTOMATIC);
    assert(report.automatic_budget_used_ms >= 1U);
}

static void test_automatic_motion_budget_disarms_and_stops(void)
{
    UART_HandleTypeDef uart;
    X42sDriver driver;
    PitchAxisVelocityTest test;
    PitchAxisVelocityTestReport report;
    PitchAxisAutomaticDecision decision;
    uint32_t now_ms = 0U;

    initialize(&driver, &uart, &test, now_ms);
    now_ms = enable_test(&test, now_ms);
    assert(PitchAxisVelocityTest_SetAutomaticArmed(
        &test, true, now_ms + 1U));
    decision = automatic_decision(true, true, 0U, 1U, 1U);
    PitchAxisVelocityTest_SubmitAutomaticDecision(
        &test, &decision, now_ms + 2U);
    service(&test, now_ms + 2U, false, false, false, false);
    service(&test, now_ms + 3U, false, false, false, false);

    /* Keep the vision watchdog fresh while the cumulative motion reaches the
     * independent 3 s physical-test budget. */
    decision.sequence = 2U;
    PitchAxisVelocityTest_SubmitAutomaticDecision(
        &test, &decision, now_ms + 2999U);
    service(&test, now_ms + 3003U, false, false, false, false);
    assert(g_stop_write_count == 1U);
    assert(PitchAxisVelocityTest_GetReport(&test, &report));
    assert(!report.automatic_armed);
    assert(report.automatic_disarm_reason == PITCH_AUTOMATIC_DISARM_BUDGET);
}

static void test_zero_motion_budget_keeps_automatic_control_running(void)
{
    UART_HandleTypeDef uart;
    X42sDriver driver;
    PitchAxisVelocityTest test;
    PitchAxisVelocityTestConfig config;
    PitchAxisVelocityTestReport report;
    PitchAxisAutomaticDecision decision;
    uint32_t now_ms = 0U;

    initialize(&driver, &uart, &test, now_ms);
    now_ms = enable_test(&test, now_ms);
    assert(PitchAxisVelocityTest_GetConfig(&test, &config));
    config.automatic_motion_budget_ms = 0U;
    assert(PitchAxisVelocityTest_UpdateConfig(&test, &config));

    decision = automatic_decision(true, true, 0U, 1U, 1U);
    PitchAxisVelocityTest_SubmitAutomaticDecision(
        &test, &decision, now_ms + 2U);
    service(&test, now_ms + 2U, false, false, false, false);
    service(&test, now_ms + 3U, false, false, false, false);

    decision.sequence = 2U;
    PitchAxisVelocityTest_SubmitAutomaticDecision(
        &test, &decision, now_ms + 5000U);
    service(&test, now_ms + 5000U, false, false, false, false);
    assert(PitchAxisVelocityTest_GetReport(&test, &report));
    assert(report.automatic_armed);
    assert(report.automatic_motion_active);
    assert(report.automatic_disarm_reason == PITCH_AUTOMATIC_DISARM_NONE);
    assert(g_stop_write_count == 0U);
}

static void test_automatic_without_vision_times_out_without_motion(void)
{
    UART_HandleTypeDef uart;
    X42sDriver driver;
    PitchAxisVelocityTest test;
    PitchAxisVelocityTestReport report;
    uint32_t now_ms = 0U;

    initialize(&driver, &uart, &test, now_ms);
    now_ms = enable_test(&test, now_ms);
    assert(PitchAxisVelocityTest_SetAutomaticArmed(
        &test, true, now_ms + 1U));
    service(&test, now_ms + 201U, false, false, false, false);

    assert(PitchAxisVelocityTest_GetReport(&test, &report));
    assert(!report.automatic_armed);
    assert(report.state == PITCH_VELOCITY_TEST_STATE_ENABLED_STOPPED);
    assert(report.automatic_disarm_reason ==
           PITCH_AUTOMATIC_DISARM_DECISION_TIMEOUT);
    assert(g_velocity_write_count == 0U);
    assert(g_stop_write_count == 0U);
}

static void test_pid_debug_key2_holds_after_ball_escape(void)
{
    UART_HandleTypeDef uart;
    X42sDriver driver;
    PitchAxisVelocityTest test;
    PitchAxisVelocityTestReport report;
    PitchAxisAutomaticDecision decision;
    uint32_t now_ms = 0U;

    initialize(&driver, &uart, &test, now_ms);
    now_ms = enable_test(&test, now_ms);
    assert(PitchAxisVelocityTest_SetPidDebugEnabled(&test, true, now_ms));
    assert(PitchAxisVelocityTest_SetAutomaticArmed(
        &test, true, now_ms + 1U));
    decision = automatic_decision(true, true, 0U, 1U, 1U);
    PitchAxisVelocityTest_SubmitAutomaticDecision(
        &test, &decision, now_ms + 2U);
    service(&test, now_ms + 2U, false, false, false, false);
    assert(g_velocity_write_count == 1U);
    service(&test, now_ms + 3U, false, false, false, false);

    now_ms = press_key(&test, now_ms + 4U, 2U);
    assert(PitchAxisVelocityTest_GetReport(&test, &report));
    assert(!report.automatic_armed);
    assert(report.automatic_hold);
    assert(report.automatic_disarm_reason ==
           PITCH_AUTOMATIC_DISARM_BALL_ESCAPE);
    assert(report.ball_escape_count == 1U);
    assert(report.state == PITCH_VELOCITY_TEST_STATE_ENABLED_STOPPED);
    assert(g_stop_write_count == 1U);
    assert(g_velocity_write_count == 1U);

    now_ms = release_all(&test, now_ms + 1U);
    now_ms = press_key(&test, now_ms + 1U, 2U);
    assert(PitchAxisVelocityTest_GetReport(&test, &report));
    assert(report.ball_escape_count == 1U);
    assert(g_stop_write_count == 1U);

    now_ms = release_all(&test, now_ms + 1U);
    now_ms = press_key(&test, now_ms + 1U, 3U);
    assert(PitchAxisVelocityTest_GetReport(&test, &report));
    assert(!report.automatic_hold);
    assert(g_velocity_write_count == 1U);
    assert(PitchAxisVelocityTest_SetAutomaticArmed(
        &test, true, now_ms + 1U));
}

static void test_automatic_limits_are_live_tunable(void)
{
    UART_HandleTypeDef uart;
    X42sDriver driver;
    PitchAxisVelocityTest test;
    PitchAxisVelocityTestConfig config;
    PitchAxisAutomaticDecision decision;
    uint32_t now_ms = 0U;

    initialize(&driver, &uart, &test, now_ms);
    now_ms = enable_test(&test, now_ms);
    assert(PitchAxisVelocityTest_SetAutomaticArmed(
        &test, true, now_ms + 1U));
    decision = automatic_decision(true, true, 0U, 1U, 1U);
    PitchAxisVelocityTest_SubmitAutomaticDecision(
        &test, &decision, now_ms + 2U);
    service(&test, now_ms + 2U, false, false, false, false);

    assert(PitchAxisVelocityTest_GetConfig(&test, &config));
    config.automatic_max_speed_rpm = 30U;
    config.automatic_decision_timeout_ms = 300U;
    config.automatic_vision_loss_grace_ms = 250U;
    config.automatic_motion_budget_ms = 12000U;
    config.acceleration = 80U;
    assert(PitchAxisVelocityTest_UpdateConfig(&test, &config));

    config.speed_rpm++;
    assert(!PitchAxisVelocityTest_UpdateConfig(&test, &config));
}

static void test_short_vision_loss_stops_and_recovers_while_armed(void)
{
    UART_HandleTypeDef uart;
    X42sDriver driver;
    PitchAxisVelocityTest test;
    PitchAxisVelocityTestConfig config;
    PitchAxisVelocityTestReport report;
    PitchAxisAutomaticDecision decision;
    uint32_t now_ms = 0U;

    initialize(&driver, &uart, &test, now_ms);
    now_ms = enable_test(&test, now_ms);
    assert(PitchAxisVelocityTest_GetConfig(&test, &config));
    config.automatic_decision_timeout_ms = 500U;
    config.automatic_vision_loss_grace_ms = 100U;
    assert(PitchAxisVelocityTest_UpdateConfig(&test, &config));

    decision = automatic_decision(true, true, 0U, 1U, 1U);
    PitchAxisVelocityTest_SubmitAutomaticDecision(
        &test, &decision, now_ms + 2U);
    service(&test, now_ms + 2U, false, false, false, false);
    service(&test, now_ms + 3U, false, false, false, false);
    assert(g_velocity_write_count == 1U);

    decision = automatic_decision(false, false, 0U, 0U, 2U);
    PitchAxisVelocityTest_SubmitAutomaticDecision(
        &test, &decision, now_ms + 4U);
    service(&test, now_ms + 4U, false, false, false, false);
    assert(g_stop_write_count == 1U);
    assert(PitchAxisVelocityTest_GetReport(&test, &report));
    assert(report.automatic_armed);
    assert(report.automatic_vision_loss_active);
    service(&test, now_ms + 5U, false, false, false, false);

    decision = automatic_decision(true, true, 0U, 1U, 3U);
    PitchAxisVelocityTest_SubmitAutomaticDecision(
        &test, &decision, now_ms + 50U);
    service(&test, now_ms + 50U, false, false, false, false);
    service(&test, now_ms + 51U, false, false, false, false);
    assert(PitchAxisVelocityTest_GetReport(&test, &report));
    assert(report.automatic_armed);
    assert(!report.automatic_vision_loss_active);
    assert(report.state == PITCH_VELOCITY_TEST_STATE_RUNNING_AUTOMATIC);
    assert(g_velocity_write_count == 2U);

    decision = automatic_decision(false, false, 0U, 0U, 4U);
    PitchAxisVelocityTest_SubmitAutomaticDecision(
        &test, &decision, now_ms + 60U);
    service(&test, now_ms + 60U, false, false, false, false);
    service(&test, now_ms + 61U, false, false, false, false);
    service(&test, now_ms + 161U, false, false, false, false);
    assert(PitchAxisVelocityTest_GetReport(&test, &report));
    assert(!report.automatic_armed);
    assert(report.automatic_disarm_reason ==
           PITCH_AUTOMATIC_DISARM_VISION_INVALID);
}

static void test_edge_loss_stops_then_recovers_inward(void)
{
    uint8_t direction;

    for (direction = 0U; direction <= 1U; ++direction)
    {
        UART_HandleTypeDef uart;
        X42sDriver driver;
        PitchAxisVelocityTest test;
        PitchAxisVelocityTestConfig config;
        PitchAxisVelocityTestReport report;
        PitchAxisAutomaticDecision decision;
        uint32_t now_ms = 0U;

        initialize(&driver, &uart, &test, now_ms);
        now_ms = enable_test(&test, now_ms);
        assert(PitchAxisVelocityTest_GetConfig(&test, &config));
        config.automatic_decision_timeout_ms = 500U;
        config.automatic_vision_loss_grace_ms = 1000U;
        config.automatic_edge_recovery_enabled = true;
        config.automatic_edge_recovery_speed_rpm = 1U;
        config.automatic_edge_recovery_max_ms = 200U;
        assert(PitchAxisVelocityTest_UpdateConfig(&test, &config));
        assert(PitchAxisVelocityTest_SetAutomaticArmed(
            &test, true, now_ms + 1U));

        decision = automatic_decision(true, true, direction, 1U, 1U);
        decision.edge_recovery_candidate = true;
        decision.edge_recovery_direction = direction;
        PitchAxisVelocityTest_SubmitAutomaticDecision(
            &test, &decision, now_ms + 2U);
        service(&test, now_ms + 2U, false, false, false, false);
        service(&test, now_ms + 3U, false, false, false, false);
        assert(g_velocity_write_count == 1U);

        decision = automatic_decision(false, false, 0U, 0U, 2U);
        PitchAxisVelocityTest_SubmitAutomaticDecision(
            &test, &decision, now_ms + 4U);
        service(&test, now_ms + 4U, false, false, false, false);
        assert(g_stop_write_count == 1U);
        service(&test, now_ms + 5U, false, false, false, false);
        assert(g_velocity_write_count == 2U);
        assert(g_velocity_direction == direction);
        assert(g_velocity_speed_rpm == 1U);
        assert(PitchAxisVelocityTest_GetReport(&test, &report));
        assert(report.automatic_armed);
        assert(report.automatic_edge_recovery_active);
        assert(report.automatic_edge_recovery_count == 1U);
        service(&test, now_ms + 6U, false, false, false, false);

        decision = automatic_decision(true, false, 0U, 0U, 3U);
        PitchAxisVelocityTest_SubmitAutomaticDecision(
            &test, &decision, now_ms + 20U);
        service(&test, now_ms + 20U, false, false, false, false);
        assert(g_stop_write_count == 2U);
        assert(PitchAxisVelocityTest_GetReport(&test, &report));
        assert(report.automatic_armed);
        assert(!report.automatic_vision_loss_active);
        assert(!report.automatic_edge_recovery_active);
    }
}

static void test_first_outside_limit_frame_starts_edge_recovery(void)
{
    UART_HandleTypeDef uart;
    X42sDriver driver;
    PitchAxisVelocityTest test;
    PitchAxisVelocityTestConfig config;
    PitchAxisVelocityTestReport report;
    PitchAxisAutomaticDecision decision;
    uint32_t now_ms = 0U;

    initialize(&driver, &uart, &test, now_ms);
    now_ms = enable_test(&test, now_ms);
    assert(PitchAxisVelocityTest_GetConfig(&test, &config));
    config.automatic_decision_timeout_ms = 500U;
    config.automatic_vision_loss_grace_ms = 1000U;
    config.automatic_edge_recovery_enabled = true;
    config.automatic_edge_recovery_speed_rpm = 20U;
    config.automatic_edge_recovery_max_ms = 200U;
    config.automatic_max_speed_rpm = 50U;
    assert(PitchAxisVelocityTest_UpdateConfig(&test, &config));
    assert(PitchAxisVelocityTest_SetAutomaticArmed(
        &test, true, now_ms + 1U));

    decision = automatic_decision(false, false, 0U, 0U, 1U);
    decision.unsafe_reason = PITCH_AUTOMATIC_DISARM_BALL_ESCAPE;
    decision.edge_recovery_candidate = true;
    decision.edge_recovery_direction = 1U;
    PitchAxisVelocityTest_SubmitAutomaticDecision(
        &test, &decision, now_ms + 2U);
    service(&test, now_ms + 2U, false, false, false, false);

    assert(g_velocity_write_count == 1U);
    assert(g_velocity_direction == 1U);
    assert(g_velocity_speed_rpm == 20U);
    assert(PitchAxisVelocityTest_GetReport(&test, &report));
    assert(report.automatic_armed);
    assert(report.automatic_vision_loss_active);
    assert(report.automatic_edge_recovery_active);

    service(&test, now_ms + 3U, false, false, false, false);
    decision = automatic_decision(true, false, 0U, 0U, 2U);
    PitchAxisVelocityTest_SubmitAutomaticDecision(
        &test, &decision, now_ms + 20U);
    service(&test, now_ms + 20U, false, false, false, false);
    assert(g_stop_write_count == 1U);
    assert(PitchAxisVelocityTest_GetReport(&test, &report));
    assert(report.automatic_armed);
    assert(!report.automatic_vision_loss_active);
    assert(!report.automatic_edge_recovery_active);
}

static void test_edge_recovery_timeout_stops_then_retries(void)
{
    UART_HandleTypeDef uart;
    X42sDriver driver;
    PitchAxisVelocityTest test;
    PitchAxisVelocityTestConfig config;
    PitchAxisVelocityTestReport report;
    PitchAxisAutomaticDecision decision;
    uint32_t now_ms = 0U;

    initialize(&driver, &uart, &test, now_ms);
    now_ms = enable_test(&test, now_ms);
    assert(PitchAxisVelocityTest_GetConfig(&test, &config));
    config.automatic_decision_timeout_ms = 500U;
    config.automatic_vision_loss_grace_ms = 50U;
    config.automatic_edge_recovery_enabled = true;
    config.automatic_edge_recovery_speed_rpm = 1U;
    config.automatic_edge_recovery_max_ms = 100U;
    assert(PitchAxisVelocityTest_UpdateConfig(&test, &config));
    assert(PitchAxisVelocityTest_SetAutomaticArmed(
        &test, true, now_ms + 1U));

    decision = automatic_decision(true, true, 0U, 1U, 1U);
    decision.edge_recovery_candidate = true;
    decision.edge_recovery_direction = 0U;
    PitchAxisVelocityTest_SubmitAutomaticDecision(
        &test, &decision, now_ms + 2U);
    service(&test, now_ms + 2U, false, false, false, false);
    service(&test, now_ms + 3U, false, false, false, false);

    decision = automatic_decision(false, false, 0U, 0U, 2U);
    PitchAxisVelocityTest_SubmitAutomaticDecision(
        &test, &decision, now_ms + 4U);
    service(&test, now_ms + 4U, false, false, false, false);
    service(&test, now_ms + 5U, false, false, false, false);
    service(&test, now_ms + 6U, false, false, false, false);
    service(&test, now_ms + 106U, false, false, false, false);
    service(&test, now_ms + 107U, false, false, false, false);

    assert(PitchAxisVelocityTest_GetReport(&test, &report));
    assert(report.automatic_armed);
    assert(report.automatic_vision_loss_active);
    assert(report.automatic_edge_recovery_active);
    assert(report.automatic_edge_recovery_count == 2U);
    assert(report.automatic_disarm_reason == PITCH_AUTOMATIC_DISARM_NONE);
    assert(g_stop_write_count == 2U);
    assert(g_velocity_write_count == 3U);
    assert(g_velocity_direction == 0U);
}

int main(void)
{
    test_self_test_pass_waits_for_explicit_start();
    test_gate_enable_and_timed_positive_motion();
    test_negative_motion_and_stop_button();
    test_rejected_velocity_ack_latches_stop();
    test_stop_button_when_disabled_does_not_wait_for_ack();
    test_automatic_clamps_speed_and_stops_on_unsafe_vision();
    test_automatic_reversal_stops_before_new_direction();
    test_same_direction_speed_update_does_not_stop();
    test_automatic_motion_budget_disarms_and_stops();
    test_zero_motion_budget_keeps_automatic_control_running();
    test_automatic_without_vision_times_out_without_motion();
    test_pid_debug_key2_holds_after_ball_escape();
    test_automatic_limits_are_live_tunable();
    test_short_vision_loss_stops_and_recovers_while_armed();
    test_edge_loss_stops_then_recovers_inward();
    test_first_outside_limit_frame_starts_edge_recovery();
    test_edge_recovery_timeout_stops_then_retries();
    test_position_loop_bounds_target_and_stops_before_reversal();
    puts("PITCH_AXIS_VELOCITY_TEST_TEST=PASS");
    return 0;
}
