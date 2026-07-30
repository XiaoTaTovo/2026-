#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "pitch_axis_manual_telemetry.h"

static char g_output[8192];
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

bool PitchAxisManualControl_GetReport(
    const PitchAxisManualControl *control,
    PitchAxisManualReport *report)
{
    if ((control == NULL) || (report == NULL))
    {
        return false;
    }
    *report = control->report;
    return true;
}

bool PitchAxisManualControl_PeekEvent(
    const PitchAxisManualControl *control,
    PitchAxisManualEvent *event)
{
    if ((control == NULL) || (event == NULL) ||
        (control->event_tail == control->event_head))
    {
        return false;
    }
    *event = control->events[control->event_tail];
    return true;
}

void PitchAxisManualControl_DropEvent(PitchAxisManualControl *control)
{
    if (control->event_tail != control->event_head)
    {
        control->event_tail = (uint8_t)((control->event_tail + 1U) %
                                        PITCH_AXIS_MANUAL_EVENT_QUEUE_SIZE);
    }
}

static void initialize_control(PitchAxisManualControl *control)
{
    memset(control, 0, sizeof(*control));
    control->config.speed_rpm = 60U;
    control->config.acceleration = 100U;
    control->config.step_pulses = 32U;
    control->config.motion_mode = 0U;
    control->config.positive_direction = 0U;
    control->config.negative_direction = 1U;
    control->config.debounce_ms = 30U;
    control->config.settle_ms = 1200U;
    control->report.state = PITCH_MANUAL_STATE_ENABLED_IDLE;
    control->report.communication_ready = true;
    control->report.enabled = true;
    control->report.last_command = PITCH_MANUAL_COMMAND_MOVE_POSITIVE;
    control->report.last_ack_status = X42S_COMMAND_STATUS_ACCEPTED;
    control->report.position_before = 10;
    control->report.position_after = 42;
    control->report.position_delta = 32;
}

static void reset_output(void)
{
    memset(g_output, 0, sizeof(g_output));
    g_output_length = 0U;
    g_output_ready = true;
}

static void service_boot(
    PitchAxisManualTelemetry *telemetry,
    uint32_t now_ms)
{
    uint8_t index;
    for (index = 0U; index < 17U; ++index)
    {
        PitchAxisManualTelemetry_Service(telemetry, now_ms);
    }
}

static void test_boot_event_status_and_backpressure(void)
{
    PitchAxisManualControl control;
    PitchAxisManualTelemetry telemetry;
    BspBluetooth bluetooth;
    size_t before;

    initialize_control(&control);
    memset(&bluetooth, 0, sizeof(bluetooth));
    reset_output();
    assert(PitchAxisManualTelemetry_Init(
        &telemetry, &control, &bluetooth, 0U));

    service_boot(&telemetry, 0U);
    assert(strstr(g_output, "BT_CONTROL=TELEMETRY_ONLY\r\n") != NULL);
    assert(strstr(g_output, "PITCH_TEST_STEP_PULSES=32\r\n") != NULL);
    assert(strstr(
        g_output,
        "PITCH_TEST_MODE=RELATIVE_PREVIOUS_TARGET\r\n") != NULL);
    assert(strstr(g_output, "PITCH_STOP_IS_NOT_POWER_CUT\r\n") != NULL);

    control.events[0].type = PITCH_MANUAL_EVENT_POSITION_DELTA;
    control.events[0].command = PITCH_MANUAL_COMMAND_MOVE_POSITIVE;
    control.events[0].ack_status = X42S_COMMAND_STATUS_ACCEPTED;
    control.events[0].value = 32;
    control.event_head = 1U;
    control.event_tail = 0U;

    g_output_ready = false;
    before = g_output_length;
    PitchAxisManualTelemetry_Service(&telemetry, 1U);
    assert(g_output_length == before);
    assert(control.event_tail == 0U);

    g_output_ready = true;
    PitchAxisManualTelemetry_Service(&telemetry, 2U);
    assert(control.event_tail == 1U);
    assert(strstr(
        g_output,
        "PITCH_EVENT=POSITION_DELTA,key=0,cmd=MOVE_POS,ack=0x02,value=32\r\n") != NULL);

    PitchAxisManualTelemetry_Service(&telemetry, 500U);
    assert(strstr(g_output, "PITCH_MANUAL,state=ENABLED_IDLE") != NULL);
    assert(strstr(g_output, ",enabled=1,latched=0,failure=NONE") != NULL);
    assert(strstr(g_output, ",before=10,after=42,delta=32") != NULL);
}

int main(void)
{
    test_boot_event_status_and_backpressure();
    puts("PITCH_AXIS_MANUAL_TELEMETRY_TEST=PASS");
    return 0;
}
