#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "pitch_axis_angle_self_test_telemetry.h"

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

PitchAxisAngleSelfTestState PitchAxisAngleSelfTest_GetState(
    const PitchAxisAngleSelfTest *self_test)
{
    return self_test->state;
}

PitchAxisAngleSelfTestFailure PitchAxisAngleSelfTest_GetFailure(
    const PitchAxisAngleSelfTest *self_test)
{
    return self_test->failure;
}

bool PitchAxisAngleSelfTest_GetReport(
    const PitchAxisAngleSelfTest *self_test,
    PitchAxisAngleSelfTestReport *report)
{
    if ((self_test == NULL) || (report == NULL) || !self_test->initialized)
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

static void service_until_complete(PitchAxisAngleSelfTestTelemetry *telemetry)
{
    uint32_t iteration;

    for (iteration = 0U; iteration < 30U; ++iteration)
    {
        PitchAxisAngleSelfTestTelemetry_Service(telemetry);
    }
}

static void test_pass_summary_keeps_control_disabled(void)
{
    PitchAxisAngleSelfTest self_test;
    PitchAxisAngleSelfTestTelemetry telemetry;
    BspBluetooth output;

    memset(&self_test, 0, sizeof(self_test));
    memset(&output, 0, sizeof(output));
    self_test.initialized = true;
    self_test.state = PITCH_AXIS_ANGLE_SELF_TEST_STATE_PASS;
    self_test.report.target_samples = 500U;
    self_test.report.completed_samples = 500U;
    self_test.report.valid_sample_count = 500U;
    self_test.report.first_raw_count = 4090U;
    self_test.report.latest_raw_count = 5U;
    self_test.report.minimum_continuous_count = 4090;
    self_test.report.maximum_continuous_count = 4101;
    self_test.report.maximum_step_count = 11U;
    self_test.report.started_ms = 100U;
    self_test.report.completed_ms = 10100U;

    reset_output();
    assert(PitchAxisAngleSelfTestTelemetry_Init(
        &telemetry,
        &self_test,
        &output));
    service_until_complete(&telemetry);

    assert(strstr(g_output, "PITCH_ANGLE_TEST_START=500\r\n") != NULL);
    assert(strstr(g_output, "PITCH_ANGLE_VALID=500\r\n") != NULL);
    assert(strstr(g_output, "PITCH_ANGLE_MAX_STEP=0x0000000B\r\n") != NULL);
    assert(strstr(g_output, "PITCH_ANGLE_RESULT=PASS\r\n") != NULL);
    assert(strstr(g_output, "PITCH_ANGLE_ZERO=NOT_SET\r\n") != NULL);
    assert(strstr(g_output, "PITCH_ANGLE_CONTROL=DISABLED\r\n") != NULL);
}

static void test_failure_reason_is_reported(void)
{
    PitchAxisAngleSelfTest self_test;
    PitchAxisAngleSelfTestTelemetry telemetry;
    BspBluetooth output;

    memset(&self_test, 0, sizeof(self_test));
    memset(&output, 0, sizeof(output));
    self_test.initialized = true;
    self_test.state = PITCH_AXIS_ANGLE_SELF_TEST_STATE_FAILED;
    self_test.failure = PITCH_AXIS_ANGLE_SELF_TEST_FAILURE_MAGNET_WEAK;
    self_test.report.target_samples = 500U;

    reset_output();
    assert(PitchAxisAngleSelfTestTelemetry_Init(
        &telemetry,
        &self_test,
        &output));
    service_until_complete(&telemetry);

    assert(strstr(g_output, "PITCH_ANGLE_RESULT=FAIL\r\n") != NULL);
    assert(strstr(g_output, "PITCH_ANGLE_FAILURE=MAGNET_WEAK\r\n") != NULL);
    assert(strstr(g_output, "PITCH_ANGLE_CONTROL=DISABLED\r\n") != NULL);
}

static void test_output_backpressure_retries(void)
{
    PitchAxisAngleSelfTest self_test;
    PitchAxisAngleSelfTestTelemetry telemetry;
    BspBluetooth output;

    memset(&self_test, 0, sizeof(self_test));
    memset(&output, 0, sizeof(output));
    self_test.initialized = true;
    self_test.state = PITCH_AXIS_ANGLE_SELF_TEST_STATE_WAITING_TO_START;
    self_test.report.target_samples = 500U;

    reset_output();
    g_output_ready = false;
    assert(PitchAxisAngleSelfTestTelemetry_Init(
        &telemetry,
        &self_test,
        &output));
    PitchAxisAngleSelfTestTelemetry_Service(&telemetry);
    assert(g_output_length == 0U);

    g_output_ready = true;
    PitchAxisAngleSelfTestTelemetry_Service(&telemetry);
    assert(strstr(g_output, "PITCH_ANGLE_TEST_START=500\r\n") != NULL);
}

int main(void)
{
    test_pass_summary_keeps_control_disabled();
    test_failure_reason_is_reported();
    test_output_backpressure_retries();
    return 0;
}
