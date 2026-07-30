#include "pitch_axis_self_test_telemetry.h"

#include <stddef.h>
#include <string.h>

static bool write_bytes(
    PitchAxisSelfTestTelemetry *telemetry,
    const char *message,
    size_t length)
{
    if ((message == NULL) ||
        (BspBluetooth_TxFree(telemetry->output) < length))
    {
        return false;
    }

    return BspBluetooth_Write(
               telemetry->output,
               (const uint8_t *)message,
               length) == BSP_BLUETOOTH_OK;
}

static bool write_text(
    PitchAxisSelfTestTelemetry *telemetry,
    const char *message)
{
    return write_bytes(telemetry, message, strlen(message));
}

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

static size_t append_u32_decimal(
    char *message,
    size_t capacity,
    size_t length,
    uint32_t value)
{
    char reversed[10];
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

static size_t append_hex(
    char *message,
    size_t capacity,
    size_t length,
    uint64_t value,
    uint8_t digits)
{
    static const char hex[] = "0123456789ABCDEF";
    uint8_t index;

    for (index = 0U; (index < digits) && (length < capacity); ++index)
    {
        uint32_t shift = (uint32_t)(digits - 1U - index) * 4U;
        message[length++] = hex[(value >> shift) & 0x0FU];
    }
    return length;
}

static bool write_named_u32(
    PitchAxisSelfTestTelemetry *telemetry,
    const char *name,
    uint32_t value)
{
    char message[64];
    size_t length = 0U;

    length = append_text(message, sizeof(message), length, name);
    length = append_u32_decimal(message, sizeof(message), length, value);
    length = append_text(message, sizeof(message), length, "\r\n");
    return write_bytes(telemetry, message, length);
}

static bool write_named_hex8(
    PitchAxisSelfTestTelemetry *telemetry,
    const char *name,
    uint8_t value)
{
    char message[64];
    size_t length = 0U;

    length = append_text(message, sizeof(message), length, name);
    length = append_text(message, sizeof(message), length, "0x");
    length = append_hex(message, sizeof(message), length, value, 2U);
    length = append_text(message, sizeof(message), length, "\r\n");
    return write_bytes(telemetry, message, length);
}

static bool write_named_signed_raw(
    PitchAxisSelfTestTelemetry *telemetry,
    const char *name,
    int64_t value)
{
    char message[64];
    size_t length = 0U;
    uint64_t magnitude = (value < 0) ?
        (uint64_t)(-value) : (uint64_t)value;

    length = append_text(message, sizeof(message), length, name);
    if (length < sizeof(message))
    {
        message[length++] = (value < 0) ? '-' : '+';
    }
    length = append_text(message, sizeof(message), length, "0x");
    length = append_hex(message, sizeof(message), length, magnitude, 8U);
    length = append_text(message, sizeof(message), length, "\r\n");
    return write_bytes(telemetry, message, length);
}

static bool write_named_hex64(
    PitchAxisSelfTestTelemetry *telemetry,
    const char *name,
    uint64_t value)
{
    char message[64];
    size_t length = 0U;

    length = append_text(message, sizeof(message), length, name);
    length = append_text(message, sizeof(message), length, "0x");
    length = append_hex(message, sizeof(message), length, value, 16U);
    length = append_text(message, sizeof(message), length, "\r\n");
    return write_bytes(telemetry, message, length);
}

static const char *failure_name(PitchAxisSelfTestFailure failure)
{
    switch (failure)
    {
        case PITCH_AXIS_SELF_TEST_FAILURE_PROTOCOL:
            return "PROTOCOL";
        case PITCH_AXIS_SELF_TEST_FAILURE_STATUS_REQUEST:
            return "STATUS_REQUEST";
        case PITCH_AXIS_SELF_TEST_FAILURE_STATUS_TIMEOUT:
            return "STATUS_TIMEOUT";
        case PITCH_AXIS_SELF_TEST_FAILURE_POSITION_REQUEST:
            return "POSITION_REQUEST";
        case PITCH_AXIS_SELF_TEST_FAILURE_POSITION_TIMEOUT:
            return "POSITION_TIMEOUT";
        default:
            return "NONE";
    }
}

static bool write_failure(PitchAxisSelfTestTelemetry *telemetry)
{
    char message[64];
    size_t length = 0U;

    length = append_text(
        message,
        sizeof(message),
        length,
        "PITCH_READ1000_FAILURE=");
    length = append_text(
        message,
        sizeof(message),
        length,
        failure_name(PitchAxisSelfTest_GetFailure(telemetry->self_test)));
    length = append_text(message, sizeof(message), length, "\r\n");
    return write_bytes(telemetry, message, length);
}

static void service_summary(PitchAxisSelfTestTelemetry *telemetry)
{
    PitchAxisSelfTestReport report;
    PitchAxisSelfTestState state;
    bool sent = false;

    if (!telemetry->summary_active ||
        !PitchAxisSelfTest_GetReport(telemetry->self_test, &report))
    {
        return;
    }

    state = PitchAxisSelfTest_GetState(telemetry->self_test);

    switch (telemetry->summary_line)
    {
        case 0U:
            sent = write_text(telemetry, "PITCH_READ1000_DONE\r\n");
            break;
        case 1U:
            sent = write_named_u32(telemetry, "PITCH_READ1000_CYCLES=", report.completed_cycles);
            break;
        case 2U:
            sent = write_named_u32(
                telemetry,
                "PITCH_ADDRESS_ECHO_OK=",
                report.address_echo_valid_count);
            break;
        case 3U:
            sent = write_named_u32(telemetry, "PITCH_READ1000_STATUS_OK=", report.status_valid_count);
            break;
        case 4U:
            sent = write_named_u32(telemetry, "PITCH_READ1000_POSITION_OK=", report.position_valid_count);
            break;
        case 5U:
            sent = write_named_u32(telemetry, "PITCH_READ1000_REQUEST_ERRORS=", report.request_error_count);
            break;
        case 6U:
            sent = write_named_u32(telemetry, "PITCH_READ1000_PROTOCOL_ERRORS=", report.protocol_error_count);
            break;
        case 7U:
            sent = write_named_hex8(telemetry, "PITCH_STATUS_FIRST=", report.first_status);
            break;
        case 8U:
            sent = write_named_hex8(telemetry, "PITCH_STATUS_LAST=", report.last_status);
            break;
        case 9U:
            sent = write_named_u32(telemetry, "PITCH_STATUS_CHANGES=", report.status_change_count);
            break;
        case 10U:
            sent = write_named_signed_raw(telemetry, "PITCH_POSITION_FIRST=", report.first_position_raw);
            break;
        case 11U:
            sent = write_named_signed_raw(telemetry, "PITCH_POSITION_MIN=", report.minimum_position_raw);
            break;
        case 12U:
            sent = write_named_signed_raw(telemetry, "PITCH_POSITION_MAX=", report.maximum_position_raw);
            break;
        case 13U:
            sent = write_named_signed_raw(telemetry, "PITCH_POSITION_LAST=", report.latest_position_raw);
            break;
        case 14U:
            sent = write_named_hex64(telemetry, "PITCH_POSITION_MAX_STEP=", report.maximum_position_step_raw);
            break;
        case 15U:
            sent = write_named_u32(
                telemetry,
                "PITCH_READ1000_DURATION_MS=",
                report.completed_ms - report.started_ms);
            break;
        case 16U:
            sent = (state == PITCH_AXIS_SELF_TEST_STATE_COMM_PASS) ?
                write_text(telemetry, "PITCH_READ1000_RESULT=COMM_PASS\r\n") :
                write_text(telemetry, "PITCH_READ1000_RESULT=FAIL\r\n");
            break;
        case 17U:
            sent = (state == PITCH_AXIS_SELF_TEST_STATE_FAILED) ?
                write_failure(telemetry) : true;
            break;
        case 18U:
            sent = (state == PITCH_AXIS_SELF_TEST_STATE_COMM_PASS) ?
                write_text(telemetry, "PITCH_COMMUNICATION_GATE=PASS\r\n") :
                write_text(telemetry, "PITCH_COMMUNICATION_GATE=FAIL\r\n");
            break;
        default:
            telemetry->summary_active = false;
            break;
    }

    if (sent)
    {
        telemetry->summary_line++;
    }
}

bool PitchAxisSelfTestTelemetry_Init(
    PitchAxisSelfTestTelemetry *telemetry,
    const PitchAxisSelfTest *self_test,
    BspBluetooth *output)
{
    if ((telemetry == NULL) || (self_test == NULL) || (output == NULL))
    {
        return false;
    }

    memset(telemetry, 0, sizeof(*telemetry));
    telemetry->self_test = self_test;
    telemetry->output = output;
    telemetry->initialized = true;
    return true;
}

void PitchAxisSelfTestTelemetry_Service(
    PitchAxisSelfTestTelemetry *telemetry)
{
    PitchAxisSelfTestReport report;
    PitchAxisSelfTestState state;

    if ((telemetry == NULL) || !telemetry->initialized ||
        !PitchAxisSelfTest_GetReport(telemetry->self_test, &report))
    {
        return;
    }

    if (!telemetry->start_announced)
    {
        telemetry->start_announced = write_named_u32(
            telemetry,
            "PITCH_READ1000_START=",
            report.target_cycles);
    }

    state = PitchAxisSelfTest_GetState(telemetry->self_test);
    if (!telemetry->summary_started &&
        ((state == PITCH_AXIS_SELF_TEST_STATE_COMM_PASS) ||
         (state == PITCH_AXIS_SELF_TEST_STATE_FAILED)))
    {
        telemetry->summary_started = true;
        telemetry->summary_active = true;
        telemetry->summary_line = 0U;
    }

    if (!telemetry->summary_started &&
        (report.completed_cycles >= (telemetry->progress_reported + 100U)) &&
        write_named_u32(
            telemetry,
            "PITCH_READ1000_PROGRESS=",
            report.completed_cycles))
    {
        telemetry->progress_reported = report.completed_cycles;
    }

    service_summary(telemetry);
}
