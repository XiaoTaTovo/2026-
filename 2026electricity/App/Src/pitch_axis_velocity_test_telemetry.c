#include "pitch_axis_velocity_test_telemetry.h"

#include <stddef.h>
#include <string.h>

#define PITCH_VELOCITY_TEST_STATUS_PERIOD_MS 1000U

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
    PitchAxisVelocityTestTelemetry *telemetry,
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
    PitchAxisVelocityTestTelemetry *telemetry,
    const char *message)
{
    return write_bytes(telemetry, message, strlen(message));
}

static bool write_named_u32(
    PitchAxisVelocityTestTelemetry *telemetry,
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

static const char *state_name(PitchAxisVelocityTestState state)
{
    switch (state)
    {
        case PITCH_VELOCITY_TEST_STATE_LOCKED_WAIT_SELF_TEST:
            return "LOCKED_SELF_TEST";
        case PITCH_VELOCITY_TEST_STATE_DISABLED_READY:
            return "DISABLED_READY";
        case PITCH_VELOCITY_TEST_STATE_WAIT_ENABLE_ACK:
            return "WAIT_ENABLE_ACK";
        case PITCH_VELOCITY_TEST_STATE_ENABLED_STOPPED:
            return "ENABLED_STOPPED";
        case PITCH_VELOCITY_TEST_STATE_WAIT_DISABLE_ACK:
            return "WAIT_DISABLE_ACK";
        case PITCH_VELOCITY_TEST_STATE_WAIT_POSITION_BEFORE:
            return "WAIT_POS_BEFORE";
        case PITCH_VELOCITY_TEST_STATE_WAIT_VELOCITY_ACK:
            return "WAIT_VELOCITY_ACK";
        case PITCH_VELOCITY_TEST_STATE_RUNNING_TIMED:
            return "RUNNING_TIMED";
        case PITCH_VELOCITY_TEST_STATE_WAIT_STOP_ACK:
            return "WAIT_STOP_ACK";
        case PITCH_VELOCITY_TEST_STATE_WAIT_POSITION_AFTER:
            return "WAIT_POS_AFTER";
        case PITCH_VELOCITY_TEST_STATE_WAIT_AUTOMATIC_VELOCITY_ACK:
            return "WAIT_AUTO_VELOCITY_ACK";
        case PITCH_VELOCITY_TEST_STATE_RUNNING_AUTOMATIC:
            return "RUNNING_AUTO";
        case PITCH_VELOCITY_TEST_STATE_WAIT_AUTOMATIC_STOP_ACK:
            return "WAIT_AUTO_STOP_ACK";
        case PITCH_VELOCITY_TEST_STATE_STOPPING:
            return "STOPPING";
        case PITCH_VELOCITY_TEST_STATE_FAULT_LATCHED:
            return "FAULT_LATCHED";
        default:
            return "UNINITIALIZED";
    }
}

static const char *command_name(PitchAxisVelocityTestCommand command)
{
    switch (command)
    {
        case PITCH_VELOCITY_TEST_COMMAND_ENABLE:
            return "ENABLE";
        case PITCH_VELOCITY_TEST_COMMAND_DISABLE:
            return "DISABLE";
        case PITCH_VELOCITY_TEST_COMMAND_RUN_POSITIVE:
            return "RUN_POS";
        case PITCH_VELOCITY_TEST_COMMAND_RUN_NEGATIVE:
            return "RUN_NEG";
        case PITCH_VELOCITY_TEST_COMMAND_STOP:
            return "STOP";
        case PITCH_VELOCITY_TEST_COMMAND_AUTOMATIC_VELOCITY:
            return "AUTO_VELOCITY";
        case PITCH_VELOCITY_TEST_COMMAND_AUTOMATIC_STOP:
            return "AUTO_STOP";
        default:
            return "NONE";
    }
}

static const char *failure_name(PitchAxisVelocityTestFailure failure)
{
    switch (failure)
    {
        case PITCH_VELOCITY_TEST_FAILURE_SELF_TEST:
            return "SELF_TEST";
        case PITCH_VELOCITY_TEST_FAILURE_STOP_BUTTON:
            return "STOP_BUTTON";
        case PITCH_VELOCITY_TEST_FAILURE_REQUEST:
            return "REQUEST";
        case PITCH_VELOCITY_TEST_FAILURE_POSITION_TIMEOUT:
            return "POSITION_TIMEOUT";
        case PITCH_VELOCITY_TEST_FAILURE_COMMAND_TIMEOUT:
            return "COMMAND_TIMEOUT";
        case PITCH_VELOCITY_TEST_FAILURE_STOP_TIMEOUT:
            return "STOP_TIMEOUT";
        case PITCH_VELOCITY_TEST_FAILURE_COMMAND_REJECTED:
            return "COMMAND_REJECTED";
        case PITCH_VELOCITY_TEST_FAILURE_PROTOCOL:
            return "PROTOCOL";
        case PITCH_VELOCITY_TEST_FAILURE_UART:
            return "UART";
        case PITCH_VELOCITY_TEST_FAILURE_RX_OVERFLOW:
            return "RX_OVERFLOW";
        default:
            return "NONE";
    }
}

static const char *event_name(PitchAxisVelocityTestEventType type)
{
    switch (type)
    {
        case PITCH_VELOCITY_TEST_EVENT_READY:
            return "READY";
        case PITCH_VELOCITY_TEST_EVENT_KEY_PRESS:
            return "KEY_PRESS";
        case PITCH_VELOCITY_TEST_EVENT_REJECT_LOCKED:
            return "REJECT_LOCKED";
        case PITCH_VELOCITY_TEST_EVENT_REJECT_DISABLED:
            return "REJECT_DISABLED";
        case PITCH_VELOCITY_TEST_EVENT_REJECT_BUSY:
            return "REJECT_BUSY";
        case PITCH_VELOCITY_TEST_EVENT_REJECT_CONFLICT:
            return "REJECT_CONFLICT";
        case PITCH_VELOCITY_TEST_EVENT_ENABLE_SENT:
            return "ENABLE_SENT";
        case PITCH_VELOCITY_TEST_EVENT_DISABLE_SENT:
            return "DISABLE_SENT";
        case PITCH_VELOCITY_TEST_EVENT_POSITION_BEFORE_REQUESTED:
            return "POS_BEFORE_REQ";
        case PITCH_VELOCITY_TEST_EVENT_POSITION_BEFORE:
            return "POSITION_BEFORE";
        case PITCH_VELOCITY_TEST_EVENT_VELOCITY_SENT:
            return "VELOCITY_SENT";
        case PITCH_VELOCITY_TEST_EVENT_COMMAND_ACK:
            return "COMMAND_ACK";
        case PITCH_VELOCITY_TEST_EVENT_AUTO_STOP_SENT:
            return "AUTO_STOP_SENT";
        case PITCH_VELOCITY_TEST_EVENT_POSITION_AFTER_REQUESTED:
            return "POS_AFTER_REQ";
        case PITCH_VELOCITY_TEST_EVENT_POSITION_AFTER:
            return "POSITION_AFTER";
        case PITCH_VELOCITY_TEST_EVENT_POSITION_DELTA:
            return "POSITION_DELTA";
        case PITCH_VELOCITY_TEST_EVENT_STOP_SENT:
            return "STOP_SENT";
        case PITCH_VELOCITY_TEST_EVENT_FAULT_LATCHED:
            return "FAULT_LATCHED";
        case PITCH_VELOCITY_TEST_EVENT_AUTOMATIC_ARMED:
            return "AUTO_ARMED";
        case PITCH_VELOCITY_TEST_EVENT_AUTOMATIC_DISARMED:
            return "AUTO_DISARMED";
        case PITCH_VELOCITY_TEST_EVENT_AUTOMATIC_VELOCITY_SENT:
            return "AUTO_VELOCITY_SENT";
        case PITCH_VELOCITY_TEST_EVENT_AUTOMATIC_STOP_SENT:
            return "AUTO_STOP_SENT";
        case PITCH_VELOCITY_TEST_EVENT_BALL_ESCAPE_HOLD:
            return "BALL_ESCAPE_HOLD";
        case PITCH_VELOCITY_TEST_EVENT_RESUME_READY:
            return "RESUME_READY";
        default:
            return "UNKNOWN";
    }
}

static const char *automatic_disarm_name(
    PitchAxisAutomaticDisarmReason reason)
{
    switch (reason)
    {
        case PITCH_AUTOMATIC_DISARM_USER:
            return "USER";
        case PITCH_AUTOMATIC_DISARM_BALL_ESCAPE:
            return "BALL_ESCAPE";
        case PITCH_AUTOMATIC_DISARM_VISION_INVALID:
            return "VISION_INVALID";
        case PITCH_AUTOMATIC_DISARM_VISION_LOW_CONFIDENCE:
            return "VISION_LOW_CONF";
        case PITCH_AUTOMATIC_DISARM_VISION_STALE:
            return "VISION_STALE";
        case PITCH_AUTOMATIC_DISARM_DECISION_TIMEOUT:
            return "DECISION_TIMEOUT";
        case PITCH_AUTOMATIC_DISARM_EDGE_RECOVERY_TIMEOUT:
            return "EDGE_TIMEOUT";
        case PITCH_AUTOMATIC_DISARM_BUDGET:
            return "BUDGET";
        case PITCH_AUTOMATIC_DISARM_FAULT:
            return "FAULT";
        default:
            return "NONE";
    }
}

static bool service_boot(PitchAxisVelocityTestTelemetry *telemetry)
{
    bool sent = false;
    const PitchAxisVelocityTestConfig *config = &telemetry->test->config;

    switch (telemetry->boot_line)
    {
        case 0U:
            sent = write_text(telemetry, "PITCH_VELOCITY_TEST_READY\r\n");
            break;
        case 1U:
            sent = write_text(
                telemetry,
                "PITCH_TEST_MODE=AUTO_START_AFTER_SELF_TEST\r\n");
            break;
        case 2U:
            sent = write_text(telemetry, "BT_CONTROL=AUTO_START_A_ARM_D_DISARM\r\n");
            break;
        case 3U:
            sent = write_text(telemetry, "KEY1=IGNORED_AUTO_START\r\n");
            break;
        case 4U:
            sent = write_text(telemetry, "KEY2=BALL_ESCAPE_HOLD\r\n");
            break;
        case 5U:
            sent = write_text(telemetry, "KEY3=RESUME_PERMISSION\r\n");
            break;
        case 6U:
            sent = write_text(telemetry, "KEY4=STOP_AND_LATCH\r\n");
            break;
        case 7U:
            sent = write_named_u32(
                telemetry, "PITCH_TEST_SPEED_RPM=", config->speed_rpm);
            break;
        case 8U:
            sent = write_named_u32(
                telemetry, "PITCH_TEST_ACCEL=", config->acceleration);
            break;
        case 9U:
            sent = write_named_u32(
                telemetry, "PITCH_TEST_RUN_MS=", config->run_ms);
            break;
        case 10U:
            sent = write_named_u32(
                telemetry,
                "PITCH_LOGICAL_POS_DIRECTION=",
                config->positive_direction);
            break;
        case 11U:
            sent = write_named_u32(
                telemetry,
                "PITCH_LOGICAL_NEG_DIRECTION=",
                config->negative_direction);
            break;
        case 12U:
            sent = write_named_u32(
                telemetry,
                "PITCH_BUTTON_DEBOUNCE_MS=",
                config->debounce_ms);
            break;
        case 13U:
            sent = write_text(telemetry, "PITCH_VISION_PID=AUTO_AFTER_SELF_TEST\r\n");
            break;
        case 14U:
            sent = write_text(telemetry, "PITCH_STOP_IS_NOT_POWER_CUT\r\n");
            break;
        case 15U:
            sent = write_named_u32(
                telemetry,
                "PITCH_AUTO_MAX_SPEED_RPM=",
                config->automatic_max_speed_rpm);
            break;
        case 16U:
            sent = write_named_u32(
                telemetry,
                "PITCH_AUTO_DECISION_TIMEOUT_MS=",
                config->automatic_decision_timeout_ms);
            break;
        case 17U:
            sent = write_named_u32(
                telemetry,
                "PITCH_AUTO_BUDGET_MS=",
                config->automatic_motion_budget_ms);
            break;
        case 18U:
            sent = write_named_u32(
                telemetry,
                "PITCH_AUTO_VISION_LOSS_GRACE_MS=",
                config->automatic_vision_loss_grace_ms);
            break;
        case 19U:
            sent = write_named_u32(
                telemetry,
                "PITCH_EDGE_RESCUE_ENABLED=",
                config->automatic_edge_recovery_enabled ? 1U : 0U);
            break;
        case 20U:
            sent = write_named_u32(
                telemetry,
                "PITCH_EDGE_RESCUE_RPM=",
                config->automatic_edge_recovery_speed_rpm);
            break;
        case 21U:
            sent = write_named_u32(
                telemetry,
                "PITCH_EDGE_RESCUE_MS=",
                config->automatic_edge_recovery_max_ms);
            break;
        default:
            return true;
    }

    if (sent)
    {
        telemetry->boot_line++;
    }
    return telemetry->boot_line > 21U;
}

static bool write_event(
    PitchAxisVelocityTestTelemetry *telemetry,
    const PitchAxisVelocityTestEvent *event)
{
    char message[190];
    size_t length = 0U;

    length = append_text(message, sizeof(message), length, "PITCH_VEL_EVENT=");
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

static bool write_status(PitchAxisVelocityTestTelemetry *telemetry)
{
    PitchAxisVelocityTestReport report;
    char message[700];
    size_t length = 0U;

    if (!PitchAxisVelocityTest_GetReport(telemetry->test, &report))
    {
        return false;
    }

    length = append_text(message, sizeof(message), length, "PITCH_VELOCITY,state=");
    length = append_text(message, sizeof(message), length, state_name(report.state));
    length = append_text(message, sizeof(message), length, ",comm=");
    length = append_u64(message, sizeof(message), length,
                        report.communication_ready ? 1U : 0U);
    length = append_text(message, sizeof(message), length, ",enabled=");
    length = append_u64(message, sizeof(message), length, report.enabled ? 1U : 0U);
    length = append_text(message, sizeof(message), length, ",velocity_cmd=");
    length = append_u64(message, sizeof(message), length,
                        report.velocity_command_active ? 1U : 0U);
    length = append_text(message, sizeof(message), length, ",auto=");
    length = append_u64(message, sizeof(message), length,
                         report.automatic_armed ? 1U : 0U);
    length = append_text(message, sizeof(message), length, ",pid_debug=");
    length = append_u64(message, sizeof(message), length,
                        report.pid_debug_enabled ? 1U : 0U);
    length = append_text(message, sizeof(message), length, ",hold=");
    length = append_u64(message, sizeof(message), length,
                        report.automatic_hold ? 1U : 0U);
    length = append_text(message, sizeof(message), length, ",escapes=");
    length = append_u64(message, sizeof(message), length,
                        report.ball_escape_count);
    length = append_text(message, sizeof(message), length, ",auto_motion=");
    length = append_u64(message, sizeof(message), length,
                        report.automatic_motion_active ? 1U : 0U);
    length = append_text(message, sizeof(message), length, ",auto_dir=");
    length = append_u64(message, sizeof(message), length,
                        report.automatic_direction);
    length = append_text(message, sizeof(message), length, ",auto_rpm=");
    length = append_u64(message, sizeof(message), length,
                        report.automatic_speed_rpm);
    length = append_text(message, sizeof(message), length, ",auto_budget_ms=");
    length = append_u64(message, sizeof(message), length,
                        report.automatic_budget_used_ms);
    length = append_text(message, sizeof(message), length, ",loss=");
    length = append_u64(message, sizeof(message), length,
                        report.automatic_vision_loss_active ? 1U : 0U);
    length = append_text(message, sizeof(message), length, ",loss_age_ms=");
    length = append_u64(message, sizeof(message), length,
                        report.automatic_vision_loss_age_ms);
    length = append_text(message, sizeof(message), length, ",loss_reason=");
    length = append_text(
        message,
        sizeof(message),
        length,
        automatic_disarm_name(report.automatic_vision_loss_reason));
    length = append_text(message, sizeof(message), length, ",rescue=");
    length = append_u64(
        message,
        sizeof(message),
        length,
        report.automatic_edge_recovery_active ? 1U : 0U);
    length = append_text(message, sizeof(message), length, ",rescue_age_ms=");
    length = append_u64(
        message,
        sizeof(message),
        length,
        report.automatic_edge_recovery_age_ms);
    length = append_text(message, sizeof(message), length, ",rescue_dir=");
    length = append_u64(
        message,
        sizeof(message),
        length,
        report.automatic_edge_recovery_direction);
    length = append_text(message, sizeof(message), length, ",rescue_rpm=");
    length = append_u64(
        message,
        sizeof(message),
        length,
        report.automatic_edge_recovery_speed_rpm);
    length = append_text(message, sizeof(message), length, ",rescues=");
    length = append_u64(
        message,
        sizeof(message),
        length,
        report.automatic_edge_recovery_count);
    length = append_text(message, sizeof(message), length, ",auto_reason=");
    length = append_text(
        message,
        sizeof(message),
        length,
        automatic_disarm_name(report.automatic_disarm_reason));
    length = append_text(message, sizeof(message), length, ",latched=");
    length = append_u64(message, sizeof(message), length,
                        report.fault_latched ? 1U : 0U);
    length = append_text(message, sizeof(message), length, ",failure=");
    length = append_text(message, sizeof(message), length, failure_name(report.failure));
    length = append_text(message, sizeof(message), length, ",last_key=");
    length = append_u64(message, sizeof(message), length, report.last_key);
    length = append_text(message, sizeof(message), length, ",last_cmd=");
    length = append_text(message, sizeof(message), length,
                         command_name(report.last_command));
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
    length = append_text(message, sizeof(message), length, ",x42_proto=");
    length = append_u64(
        message,
        sizeof(message),
        length,
        telemetry->test->driver->protocol_error_count);
    length = append_text(message, sizeof(message), length, ",x42_superseded=");
    length = append_u64(
        message,
        sizeof(message),
        length,
        telemetry->test->driver->superseded_response_count);
    length = append_text(message, sizeof(message), length, ",x42_uart=");
    length = append_u64(
        message,
        sizeof(message),
        length,
        telemetry->test->driver->transport.uart_error_count);
    length = append_text(message, sizeof(message), length, ",x42_rxovf=");
    length = append_u64(
        message,
        sizeof(message),
        length,
        telemetry->test->driver->transport.rx_overflow_count);
    length = append_text(message, sizeof(message), length, "\r\n");
    return write_bytes(telemetry, message, length);
}

bool PitchAxisVelocityTestTelemetry_Init(
    PitchAxisVelocityTestTelemetry *telemetry,
    PitchAxisVelocityTest *test,
    BspBluetooth *output,
    uint32_t now_ms)
{
    if ((telemetry == NULL) || (test == NULL) || (output == NULL))
    {
        return false;
    }

    memset(telemetry, 0, sizeof(*telemetry));
    telemetry->test = test;
    telemetry->output = output;
    telemetry->next_status_ms = now_ms + PITCH_VELOCITY_TEST_STATUS_PERIOD_MS;
    telemetry->initialized = true;
    return true;
}

void PitchAxisVelocityTestTelemetry_Service(
    PitchAxisVelocityTestTelemetry *telemetry,
    uint32_t now_ms)
{
    PitchAxisVelocityTestEvent event;

    if ((telemetry == NULL) || !telemetry->initialized)
    {
        return;
    }
    if (!service_boot(telemetry))
    {
        return;
    }
    if (PitchAxisVelocityTest_PeekEvent(telemetry->test, &event))
    {
        if (write_event(telemetry, &event))
        {
            PitchAxisVelocityTest_DropEvent(telemetry->test);
        }
        return;
    }
    if ((int32_t)(now_ms - telemetry->next_status_ms) >= 0)
    {
        if (write_status(telemetry))
        {
            telemetry->next_status_ms = now_ms +
                PITCH_VELOCITY_TEST_STATUS_PERIOD_MS;
        }
    }
}
