#include "pitch_axis_angle_self_test_telemetry.h"

#include <stddef.h>
#include <string.h>

static bool write_bytes(
    PitchAxisAngleSelfTestTelemetry *telemetry,
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
    uint32_t value,
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

static bool write_text(
    PitchAxisAngleSelfTestTelemetry *telemetry,
    const char *message)
{
    return write_bytes(telemetry, message, strlen(message));
}

static bool write_named_u32(
    PitchAxisAngleSelfTestTelemetry *telemetry,
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

static bool write_named_hex(
    PitchAxisAngleSelfTestTelemetry *telemetry,
    const char *name,
    uint32_t value,
    uint8_t digits)
{
    char message[64];
    size_t length = 0U;

    length = append_text(message, sizeof(message), length, name);
    length = append_text(message, sizeof(message), length, "0x");
    length = append_hex(message, sizeof(message), length, value, digits);
    length = append_text(message, sizeof(message), length, "\r\n");
    return write_bytes(telemetry, message, length);
}

static bool write_named_signed_raw(
    PitchAxisAngleSelfTestTelemetry *telemetry,
    const char *name,
    int32_t value)
{
    char message[64];
    size_t length = 0U;
    uint32_t magnitude = (value < 0) ?
        (uint32_t)(-(int64_t)value) : (uint32_t)value;

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

static const char *failure_name(PitchAxisAngleSelfTestFailure failure)
{
    switch (failure)
    {
        case PITCH_AXIS_ANGLE_SELF_TEST_FAILURE_I2C_NACK:
            return "I2C_NACK";
        case PITCH_AXIS_ANGLE_SELF_TEST_FAILURE_I2C_TIMEOUT:
            return "I2C_TIMEOUT";
        case PITCH_AXIS_ANGLE_SELF_TEST_FAILURE_MAGNET_MISSING:
            return "MAGNET_MISSING";
        case PITCH_AXIS_ANGLE_SELF_TEST_FAILURE_MAGNET_WEAK:
            return "MAGNET_WEAK";
        case PITCH_AXIS_ANGLE_SELF_TEST_FAILURE_MAGNET_STRONG:
            return "MAGNET_STRONG";
        default:
            return "I2C_TRANSPORT";
    }
}

static bool write_failure(PitchAxisAngleSelfTestTelemetry *telemetry)
{
    char message[64];
    size_t length = 0U;

    length = append_text(
        message,
        sizeof(message),
        length,
        "PITCH_ANGLE_FAILURE=");
    length = append_text(
        message,
        sizeof(message),
        length,
        failure_name(PitchAxisAngleSelfTest_GetFailure(telemetry->self_test)));
    length = append_text(message, sizeof(message), length, "\r\n");
    return write_bytes(telemetry, message, length);
}

static void service_summary(PitchAxisAngleSelfTestTelemetry *telemetry)
{
    PitchAxisAngleSelfTestReport report;
    PitchAxisAngleSelfTestState state;
    bool sent = false;

    if (!telemetry->summary_active ||
        !PitchAxisAngleSelfTest_GetReport(telemetry->self_test, &report))
    {
        return;
    }

    state = PitchAxisAngleSelfTest_GetState(telemetry->self_test);
    switch (telemetry->summary_line)
    {
        case 0U:
            sent = write_text(telemetry, "PITCH_ANGLE_TEST_DONE\r\n");
            break;
        case 1U:
            sent = write_named_u32(
                telemetry,
                "PITCH_ANGLE_SAMPLES=",
                report.completed_samples);
            break;
        case 2U:
            sent = write_named_u32(
                telemetry,
                "PITCH_ANGLE_VALID=",
                report.valid_sample_count);
            break;
        case 3U:
            sent = write_named_u32(
                telemetry,
                "PITCH_ANGLE_I2C_ERRORS=",
                report.i2c_error_count);
            break;
        case 4U:
            sent = write_named_u32(
                telemetry,
                "PITCH_ANGLE_MAGNET_ERRORS=",
                report.magnet_error_count);
            break;
        case 5U:
            sent = write_named_hex(
                telemetry,
                "PITCH_ANGLE_FIRST_RAW=",
                report.first_raw_count,
                4U);
            break;
        case 6U:
            sent = write_named_hex(
                telemetry,
                "PITCH_ANGLE_LAST_RAW=",
                report.latest_raw_count,
                4U);
            break;
        case 7U:
            sent = write_named_signed_raw(
                telemetry,
                "PITCH_ANGLE_MIN_CONTINUOUS=",
                report.minimum_continuous_count);
            break;
        case 8U:
            sent = write_named_signed_raw(
                telemetry,
                "PITCH_ANGLE_MAX_CONTINUOUS=",
                report.maximum_continuous_count);
            break;
        case 9U:
            sent = write_named_hex(
                telemetry,
                "PITCH_ANGLE_MAX_STEP=",
                report.maximum_step_count,
                8U);
            break;
        case 10U:
            sent = write_named_u32(
                telemetry,
                "PITCH_ANGLE_DURATION_MS=",
                report.completed_ms - report.started_ms);
            break;
        case 11U:
            sent = (state == PITCH_AXIS_ANGLE_SELF_TEST_STATE_PASS) ?
                write_text(telemetry, "PITCH_ANGLE_RESULT=PASS\r\n") :
                write_text(telemetry, "PITCH_ANGLE_RESULT=FAIL\r\n");
            break;
        case 12U:
            sent = (state == PITCH_AXIS_ANGLE_SELF_TEST_STATE_FAILED) ?
                write_failure(telemetry) : true;
            break;
        case 13U:
            sent = write_text(telemetry, "PITCH_ANGLE_ZERO=NOT_SET\r\n");
            break;
        case 14U:
            sent = write_text(telemetry, "PITCH_ANGLE_CONTROL=DISABLED\r\n");
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

bool PitchAxisAngleSelfTestTelemetry_Init(
    PitchAxisAngleSelfTestTelemetry *telemetry,
    const PitchAxisAngleSelfTest *self_test,
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

void PitchAxisAngleSelfTestTelemetry_Service(
    PitchAxisAngleSelfTestTelemetry *telemetry)
{
    PitchAxisAngleSelfTestReport report;
    PitchAxisAngleSelfTestState state;

    if ((telemetry == NULL) || !telemetry->initialized ||
        !PitchAxisAngleSelfTest_GetReport(telemetry->self_test, &report))
    {
        return;
    }

    if (!telemetry->start_announced)
    {
        telemetry->start_announced = write_named_u32(
            telemetry,
            "PITCH_ANGLE_TEST_START=",
            report.target_samples);
    }

    state = PitchAxisAngleSelfTest_GetState(telemetry->self_test);
    if (!telemetry->summary_started &&
        ((state == PITCH_AXIS_ANGLE_SELF_TEST_STATE_PASS) ||
         (state == PITCH_AXIS_ANGLE_SELF_TEST_STATE_FAILED)))
    {
        telemetry->summary_started = true;
        telemetry->summary_active = true;
        telemetry->summary_line = 0U;
    }

    service_summary(telemetry);
}
