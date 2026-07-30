#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "pitch_axis_self_test_telemetry.h"

static char g_output[4096];
static size_t g_output_length;
static bool g_output_ready;

size_t BspBluetooth_TxFree(const BspBluetooth *port)
{
    (void)port;
    return g_output_ready ? 511U : 0U;
}

BspBluetoothResult BspBluetooth_Write(
    BspBluetooth *port,
    const uint8_t *data,
    size_t length)
{
    (void)port;
    assert(data != NULL);
    assert((g_output_length + length) < sizeof(g_output));

    memcpy(&g_output[g_output_length], data, length);
    g_output_length += length;
    g_output[g_output_length] = '\0';
    return BSP_BLUETOOTH_OK;
}

PitchAxisSelfTestState PitchAxisSelfTest_GetState(
    const PitchAxisSelfTest *self_test)
{
    return self_test->state;
}

PitchAxisSelfTestFailure PitchAxisSelfTest_GetFailure(
    const PitchAxisSelfTest *self_test)
{
    return self_test->failure;
}

bool PitchAxisSelfTest_GetReport(
    const PitchAxisSelfTest *self_test,
    PitchAxisSelfTestReport *report)
{
    if ((self_test == NULL) || (report == NULL) ||
        !self_test->initialized)
    {
        return false;
    }

    *report = self_test->report;
    return true;
}

static void reset_output(void)
{
    memset(g_output, 0, sizeof(g_output));
    g_output_length = 0U;
    g_output_ready = true;
}

static void service_until_complete(PitchAxisSelfTestTelemetry *telemetry)
{
    uint32_t iteration;

    for (iteration = 0U; iteration < 30U; ++iteration)
    {
        PitchAxisSelfTestTelemetry_Service(telemetry);
    }
}

static void test_pass_summary_hands_off_communication_gate(void)
{
    PitchAxisSelfTest self_test;
    PitchAxisSelfTestTelemetry telemetry;
    BspBluetooth output;

    memset(&self_test, 0, sizeof(self_test));
    memset(&output, 0, sizeof(output));
    self_test.initialized = true;
    self_test.state = PITCH_AXIS_SELF_TEST_STATE_COMM_PASS;
    self_test.report.target_cycles = 1000U;
    self_test.report.completed_cycles = 1000U;
    self_test.report.address_echo_valid_count = 2000U;
    self_test.report.status_valid_count = 1000U;
    self_test.report.position_valid_count = 1000U;
    self_test.report.first_status = 0x03U;
    self_test.report.last_status = 0x03U;
    self_test.report.first_position_raw = -3;
    self_test.report.minimum_position_raw = -4;
    self_test.report.maximum_position_raw = 5;
    self_test.report.latest_position_raw = 2;
    self_test.report.maximum_position_step_raw = 3U;
    self_test.report.started_ms = 100U;
    self_test.report.completed_ms = 10100U;

    reset_output();
    assert(PitchAxisSelfTestTelemetry_Init(
        &telemetry,
        &self_test,
        &output));
    service_until_complete(&telemetry);

    assert(strstr(g_output, "PITCH_READ1000_START=1000\r\n") != NULL);
    assert(strstr(g_output, "PITCH_ADDRESS_ECHO_OK=2000\r\n") != NULL);
    assert(strstr(g_output, "PITCH_POSITION_MIN=-0x00000004\r\n") != NULL);
    assert(strstr(g_output, "PITCH_READ1000_RESULT=COMM_PASS\r\n") != NULL);
    assert(strstr(g_output, "PITCH_COMMUNICATION_GATE=PASS\r\n") != NULL);
}

static void test_failure_reason_is_reported(void)
{
    PitchAxisSelfTest self_test;
    PitchAxisSelfTestTelemetry telemetry;
    BspBluetooth output;

    memset(&self_test, 0, sizeof(self_test));
    memset(&output, 0, sizeof(output));
    self_test.initialized = true;
    self_test.state = PITCH_AXIS_SELF_TEST_STATE_FAILED;
    self_test.failure = PITCH_AXIS_SELF_TEST_FAILURE_STATUS_TIMEOUT;
    self_test.report.target_cycles = 1000U;

    reset_output();
    assert(PitchAxisSelfTestTelemetry_Init(
        &telemetry,
        &self_test,
        &output));
    service_until_complete(&telemetry);

    assert(strstr(g_output, "PITCH_READ1000_RESULT=FAIL\r\n") != NULL);
    assert(strstr(g_output, "PITCH_READ1000_FAILURE=STATUS_TIMEOUT\r\n") != NULL);
    assert(strstr(g_output, "PITCH_COMMUNICATION_GATE=FAIL\r\n") != NULL);
}

static void test_output_backpressure_retries(void)
{
    PitchAxisSelfTest self_test;
    PitchAxisSelfTestTelemetry telemetry;
    BspBluetooth output;

    memset(&self_test, 0, sizeof(self_test));
    memset(&output, 0, sizeof(output));
    self_test.initialized = true;
    self_test.state = PITCH_AXIS_SELF_TEST_STATE_WAITING_TO_START;
    self_test.report.target_cycles = 1000U;

    reset_output();
    g_output_ready = false;
    assert(PitchAxisSelfTestTelemetry_Init(
        &telemetry,
        &self_test,
        &output));
    PitchAxisSelfTestTelemetry_Service(&telemetry);
    assert(g_output_length == 0U);

    g_output_ready = true;
    PitchAxisSelfTestTelemetry_Service(&telemetry);
    assert(strstr(g_output, "PITCH_READ1000_START=1000\r\n") != NULL);
}

int main(void)
{
    test_pass_summary_hands_off_communication_gate();
    test_failure_reason_is_reported();
    test_output_backpressure_retries();
    return 0;
}
