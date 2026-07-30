#include "pitch_axis_manual_telemetry.h"

#include <stddef.h>
#include <string.h>

#define PITCH_MANUAL_STATUS_PERIOD_MS 500U

static size_t append_text(
    char *message,
    size_t capacity,
    size_t length,
    const char *text)
{
    size_t index = 0U;
    while ((text[index] != '\0') && (length < capacity))
    {
        message[length++] = text[index++];
    }
    return length;
}

static size_t append_u64(
    char *message,
    size_t capacity,
    size_t length,
    uint64_t value)
{
    char reversed[20];
    size_t digits = 0U;

    do
    {
        reversed[digits++] = (char)('0' + (value % 10U));
        value /= 10U;
    } while ((value != 0U) && (digits < sizeof(reversed)));

    while ((digits > 0U) && (length < capacity))
    {
        message[length++] = reversed[--digits];
    }
    return length;
}

static size_t append_i64(
    char *message,
    size_t capacity,
    size_t length,
    int64_t value)
{
    uint64_t magnitude;

    if (value < 0)
    {
        if (length < capacity)
        {
            message[length++] = '-';
        }
        magnitude = (uint64_t)(-(value + 1)) + 1U;
    }
    else
    {
        magnitude = (uint64_t)value;
    }
    return append_u64(message, capacity, length, magnitude);
}

static size_t append_hex8(
    char *message,
    size_t capacity,
    size_t length,
    uint8_t value)
{
    static const char hex[] = "0123456789ABCDEF";
    if (length < capacity)
    {
        message[length++] = hex[(value >> 4U) & 0x0FU];
    }
    if (length < capacity)
    {
        message[length++] = hex[value & 0x0FU];
    }
    return length;
}

static bool write_bytes(
    PitchAxisManualTelemetry *telemetry,
    const char *message,
    size_t length)
{
    if (BspBluetooth_TxFree(telemetry->output) < length)
    {
        return false;
    }
    return BspBluetooth_Write(
               telemetry->output,
               (const uint8_t *)message,
               length) == BSP_BLUETOOTH_OK;
}

static bool write_text(
    PitchAxisManualTelemetry *telemetry,
    const char *message)
{
    return write_bytes(telemetry, message, strlen(message));
}

static bool write_named_u32(
    PitchAxisManualTelemetry *telemetry,
    const char *name,
    uint32_t value)
{
    char message[80];
    size_t length = 0U;
    length = append_text(message, sizeof(message), length, name);
    length = append_u64(message, sizeof(message), length, value);
    length = append_text(message, sizeof(message), length, "\r\n");
    return write_bytes(telemetry, message, length);
}

static const char *state_name(PitchAxisManualState state)
{
    switch (state)
    {
        case PITCH_MANUAL_STATE_LOCKED_WAIT_SELF_TEST: return "LOCKED_SELF_TEST";
        case PITCH_MANUAL_STATE_DISABLED_READY: return "DISABLED_READY";
        case PITCH_MANUAL_STATE_WAIT_ENABLE_ACK: return "WAIT_ENABLE_ACK";
        case PITCH_MANUAL_STATE_ENABLED_IDLE: return "ENABLED_IDLE";
        case PITCH_MANUAL_STATE_WAIT_DISABLE_ACK: return "WAIT_DISABLE_ACK";
        case PITCH_MANUAL_STATE_WAIT_POSITION_BEFORE: return "WAIT_POS_BEFORE";
        case PITCH_MANUAL_STATE_WAIT_MOVE_ACK: return "WAIT_MOVE_ACK";
        case PITCH_MANUAL_STATE_WAIT_SETTLE: return "WAIT_SETTLE";
        case PITCH_MANUAL_STATE_WAIT_POSITION_AFTER: return "WAIT_POS_AFTER";
        case PITCH_MANUAL_STATE_STOPPING: return "STOPPING";
        case PITCH_MANUAL_STATE_FAULT_LATCHED: return "FAULT_LATCHED";
        default: return "UNINITIALIZED";
    }
}

static const char *command_name(PitchAxisManualCommand command)
{
    switch (command)
    {
        case PITCH_MANUAL_COMMAND_ENABLE: return "ENABLE";
        case PITCH_MANUAL_COMMAND_DISABLE: return "DISABLE";
        case PITCH_MANUAL_COMMAND_MOVE_POSITIVE: return "MOVE_POS";
        case PITCH_MANUAL_COMMAND_MOVE_NEGATIVE: return "MOVE_NEG";
        case PITCH_MANUAL_COMMAND_STOP: return "STOP";
        default: return "NONE";
    }
}

static const char *failure_name(PitchAxisManualFailure failure)
{
    switch (failure)
    {
        case PITCH_MANUAL_FAILURE_SELF_TEST: return "SELF_TEST";
        case PITCH_MANUAL_FAILURE_STOP_BUTTON: return "STOP_BUTTON";
        case PITCH_MANUAL_FAILURE_REQUEST: return "REQUEST";
        case PITCH_MANUAL_FAILURE_POSITION_TIMEOUT: return "POSITION_TIMEOUT";
        case PITCH_MANUAL_FAILURE_COMMAND_TIMEOUT: return "COMMAND_TIMEOUT";
        case PITCH_MANUAL_FAILURE_COMMAND_REJECTED: return "COMMAND_REJECTED";
        case PITCH_MANUAL_FAILURE_PROTOCOL: return "PROTOCOL";
        case PITCH_MANUAL_FAILURE_UART: return "UART";
        case PITCH_MANUAL_FAILURE_RX_OVERFLOW: return "RX_OVERFLOW";
        default: return "NONE";
    }
}

static const char *event_name(PitchAxisManualEventType type)
{
    switch (type)
    {
        case PITCH_MANUAL_EVENT_READY: return "READY";
        case PITCH_MANUAL_EVENT_KEY_PRESS: return "KEY_PRESS";
        case PITCH_MANUAL_EVENT_REJECT_LOCKED: return "REJECT_LOCKED";
        case PITCH_MANUAL_EVENT_REJECT_DISABLED: return "REJECT_DISABLED";
        case PITCH_MANUAL_EVENT_REJECT_BUSY: return "REJECT_BUSY";
        case PITCH_MANUAL_EVENT_REJECT_CONFLICT: return "REJECT_CONFLICT";
        case PITCH_MANUAL_EVENT_ENABLE_SENT: return "ENABLE_SENT";
        case PITCH_MANUAL_EVENT_DISABLE_SENT: return "DISABLE_SENT";
        case PITCH_MANUAL_EVENT_POSITION_BEFORE_REQUESTED: return "POS_BEFORE_REQ";
        case PITCH_MANUAL_EVENT_MOVE_SENT: return "MOVE_SENT";
        case PITCH_MANUAL_EVENT_POSITION_AFTER_REQUESTED: return "POS_AFTER_REQ";
        case PITCH_MANUAL_EVENT_COMMAND_ACK: return "COMMAND_ACK";
        case PITCH_MANUAL_EVENT_POSITION_BEFORE: return "POSITION_BEFORE";
        case PITCH_MANUAL_EVENT_POSITION_AFTER: return "POSITION_AFTER";
        case PITCH_MANUAL_EVENT_POSITION_DELTA: return "POSITION_DELTA";
        case PITCH_MANUAL_EVENT_STOP_SENT: return "STOP_SENT";
        case PITCH_MANUAL_EVENT_FAULT_LATCHED: return "FAULT_LATCHED";
        default: return "UNKNOWN";
    }
}

static bool service_boot(PitchAxisManualTelemetry *telemetry)
{
    bool sent = false;
    const PitchAxisManualConfig *config = &telemetry->control->config;

    switch (telemetry->boot_line)
    {
        case 0U: sent = write_text(telemetry, "PITCH_MANUAL_READY\r\n"); break;
        case 1U: sent = write_text(telemetry, "BT_CONTROL=TELEMETRY_ONLY\r\n"); break;
        case 2U: sent = write_text(telemetry, "KEY1=ENABLE_DISABLE\r\n"); break;
        case 3U: sent = write_text(telemetry, "KEY2=STEP_LOGICAL_POSITIVE\r\n"); break;
        case 4U: sent = write_text(telemetry, "KEY3=STEP_LOGICAL_NEGATIVE\r\n"); break;
        case 5U: sent = write_text(telemetry, "KEY4=STOP_AND_LATCH\r\n"); break;
        case 6U: sent = write_named_u32(telemetry, "PITCH_TEST_SPEED_RPM=", config->speed_rpm); break;
        case 7U: sent = write_named_u32(telemetry, "PITCH_TEST_ACCEL=", config->acceleration); break;
        case 8U: sent = write_named_u32(telemetry, "PITCH_TEST_STEP_PULSES=", config->step_pulses); break;
        case 9U: sent = write_text(telemetry, "PITCH_TEST_MODE=RELATIVE_PREVIOUS_TARGET\r\n"); break;
        case 10U: sent = write_named_u32(telemetry, "PITCH_LOGICAL_POS_DIRECTION=", config->positive_direction); break;
        case 11U: sent = write_named_u32(telemetry, "PITCH_LOGICAL_NEG_DIRECTION=", config->negative_direction); break;
        case 12U: sent = write_named_u32(telemetry, "PITCH_BUTTON_DEBOUNCE_MS=", config->debounce_ms); break;
        case 13U: sent = write_named_u32(telemetry, "PITCH_SETTLE_MS=", config->settle_ms); break;
        case 14U: sent = write_text(telemetry, "PITCH_ANGLE_CONTROL=DISABLED\r\n"); break;
        case 15U: sent = write_text(telemetry, "PITCH_VISION_PID=LOCKED\r\n"); break;
        case 16U: sent = write_text(telemetry, "PITCH_STOP_IS_NOT_POWER_CUT\r\n"); break;
        default: return true;
    }

    if (sent)
    {
        telemetry->boot_line++;
    }
    return telemetry->boot_line > 16U;
}

static bool write_event(
    PitchAxisManualTelemetry *telemetry,
    const PitchAxisManualEvent *event)
{
    char message[180];
    size_t length = 0U;

    length = append_text(message, sizeof(message), length, "PITCH_EVENT=");
    length = append_text(message, sizeof(message), length, event_name(event->type));
    length = append_text(message, sizeof(message), length, ",key=");
    length = append_u64(message, sizeof(message), length, event->key);
    length = append_text(message, sizeof(message), length, ",cmd=");
    length = append_text(message, sizeof(message), length, command_name(event->command));
    length = append_text(message, sizeof(message), length, ",ack=0x");
    length = append_hex8(message, sizeof(message), length, event->ack_status);
    length = append_text(message, sizeof(message), length, ",value=");
    length = append_i64(message, sizeof(message), length, event->value);
    length = append_text(message, sizeof(message), length, "\r\n");
    return write_bytes(telemetry, message, length);
}

static bool write_status(PitchAxisManualTelemetry *telemetry)
{
    PitchAxisManualReport report;
    char message[360];
    size_t length = 0U;

    if (!PitchAxisManualControl_GetReport(telemetry->control, &report))
    {
        return false;
    }

    length = append_text(message, sizeof(message), length, "PITCH_MANUAL,state=");
    length = append_text(message, sizeof(message), length, state_name(report.state));
    length = append_text(message, sizeof(message), length, ",comm=");
    length = append_u64(message, sizeof(message), length, report.communication_ready ? 1U : 0U);
    length = append_text(message, sizeof(message), length, ",enabled=");
    length = append_u64(message, sizeof(message), length, report.enabled ? 1U : 0U);
    length = append_text(message, sizeof(message), length, ",latched=");
    length = append_u64(message, sizeof(message), length, report.fault_latched ? 1U : 0U);
    length = append_text(message, sizeof(message), length, ",failure=");
    length = append_text(message, sizeof(message), length, failure_name(report.failure));
    length = append_text(message, sizeof(message), length, ",last_key=");
    length = append_u64(message, sizeof(message), length, report.last_key);
    length = append_text(message, sizeof(message), length, ",last_cmd=");
    length = append_text(message, sizeof(message), length, command_name(report.last_command));
    length = append_text(message, sizeof(message), length, ",last_ack=0x");
    length = append_hex8(message, sizeof(message), length, report.last_ack_status);
    length = append_text(message, sizeof(message), length, ",before=");
    length = append_i64(message, sizeof(message), length, report.position_before);
    length = append_text(message, sizeof(message), length, ",after=");
    length = append_i64(message, sizeof(message), length, report.position_after);
    length = append_text(message, sizeof(message), length, ",delta=");
    length = append_i64(message, sizeof(message), length, report.position_delta);
    length = append_text(message, sizeof(message), length, ",commands=");
    length = append_u64(message, sizeof(message), length, report.command_count);
    length = append_text(message, sizeof(message), length, ",rejects=");
    length = append_u64(message, sizeof(message), length, report.reject_count);
    length = append_text(message, sizeof(message), length, ",timeouts=");
    length = append_u64(message, sizeof(message), length, report.timeout_count);
    length = append_text(message, sizeof(message), length, ",errors=");
    length = append_u64(message, sizeof(message), length, report.error_count);
    length = append_text(message, sizeof(message), length, ",drops=");
    length = append_u64(message, sizeof(message), length, report.event_drop_count);
    length = append_text(message, sizeof(message), length, "\r\n");
    return write_bytes(telemetry, message, length);
}

bool PitchAxisManualTelemetry_Init(
    PitchAxisManualTelemetry *telemetry,
    PitchAxisManualControl *control,
    BspBluetooth *output,
    uint32_t now_ms)
{
    if ((telemetry == NULL) || (control == NULL) || (output == NULL))
    {
        return false;
    }

    memset(telemetry, 0, sizeof(*telemetry));
    telemetry->control = control;
    telemetry->output = output;
    telemetry->next_status_ms = now_ms + PITCH_MANUAL_STATUS_PERIOD_MS;
    telemetry->initialized = true;
    return true;
}

void PitchAxisManualTelemetry_Service(
    PitchAxisManualTelemetry *telemetry,
    uint32_t now_ms)
{
    PitchAxisManualEvent event;

    if ((telemetry == NULL) || !telemetry->initialized)
    {
        return;
    }

    if (!service_boot(telemetry))
    {
        return;
    }

    if (PitchAxisManualControl_PeekEvent(telemetry->control, &event))
    {
        if (write_event(telemetry, &event))
        {
            PitchAxisManualControl_DropEvent(telemetry->control);
        }
        return;
    }

    if ((int32_t)(now_ms - telemetry->next_status_ms) >= 0)
    {
        if (write_status(telemetry))
        {
            telemetry->next_status_ms = now_ms +
                PITCH_MANUAL_STATUS_PERIOD_MS;
        }
    }
}
