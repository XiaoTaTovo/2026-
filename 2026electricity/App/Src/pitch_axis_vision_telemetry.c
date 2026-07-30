#include "pitch_axis_vision_telemetry.h"

#include <stddef.h>
#include <string.h>

#define PITCH_VISION_STATUS_PERIOD_MS 100U

static size_t append_text(char *message, size_t capacity, size_t length, const char *text)
{
    size_t index = 0U;
    while ((text[index] != '\0') && (length < capacity))
    {
        message[length++] = text[index++];
    }
    return length;
}

static size_t append_u32(char *message, size_t capacity, size_t length, uint32_t value)
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

static size_t append_i16(char *message, size_t capacity, size_t length, int16_t value)
{
    uint16_t magnitude;

    if (value < 0)
    {
        if (length < capacity)
        {
            message[length++] = '-';
        }
        magnitude = (uint16_t)(-(value + 1)) + 1U;
    }
    else
    {
        magnitude = (uint16_t)value;
    }
    return append_u32(message, capacity, length, magnitude);
}

static const char *state_name(PitchAxisVisionState state)
{
    switch (state)
    {
        case PITCH_VISION_STATE_WAITING_FOR_FRAME: return "WAIT_FRAME";
        case PITCH_VISION_STATE_TRACKING: return "TRACKING";
        case PITCH_VISION_STATE_REJECT_INVALID: return "INVALID";
        case PITCH_VISION_STATE_REJECT_STALE: return "STALE";
        case PITCH_VISION_STATE_REJECT_LOW_CONFIDENCE: return "LOW_CONF";
        default: return "UNKNOWN";
    }
}

static bool write_bytes(
    PitchAxisVisionTelemetry *telemetry,
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

static bool write_text(PitchAxisVisionTelemetry *telemetry, const char *text)
{
    return write_bytes(telemetry, text, strlen(text));
}

static bool service_boot(PitchAxisVisionTelemetry *telemetry)
{
    bool sent;

    switch (telemetry->boot_line)
    {
        case 0U: sent = write_text(telemetry, "PITCH_VISION_READY\r\n"); break;
        case 1U: sent = write_text(telemetry, "PITCH_VISION_MOTION=DRY_RUN\r\n"); break;
        case 2U: sent = write_text(telemetry, "PITCH_VISION_CAL_PULSES_PER_MM=960\r\n"); break;
        case 3U: sent = write_text(telemetry, "PITCH_VISION_CONTROL_PERIOD_MS=50\r\n"); break;
        case 4U: sent = write_text(telemetry, "PITCH_VISION_STALE_LIMIT_MS=150\r\n"); break;
        default: return true;
    }

    if (sent)
    {
        telemetry->boot_line++;
    }
    return telemetry->boot_line > 4U;
}

static bool write_status(PitchAxisVisionTelemetry *telemetry)
{
    PitchAxisVisionReport report;
    char message[460];
    size_t length = 0U;

    if (!PitchAxisVisionControl_GetReport(telemetry->control, &report))
    {
        return false;
    }

    length = append_text(message, sizeof(message), length, "PITCH_VISION,state=");
    length = append_text(message, sizeof(message), length, state_name(report.state));
    length = append_text(message, sizeof(message), length, ",motion=DRY_RUN,valid=");
    length = append_u32(message, sizeof(message), length, report.observation.valid ? 1U : 0U);
    length = append_text(message, sizeof(message), length, ",fresh=");
    length = append_u32(message, sizeof(message), length, report.observation_fresh ? 1U : 0U);
    length = append_text(message, sizeof(message), length, ",seq=");
    length = append_u32(message, sizeof(message), length, report.observation.sequence);
    length = append_text(message, sizeof(message), length, ",x_0p1mm=");
    length = append_i16(message, sizeof(message), length, report.observation.x_0_1mm);
    length = append_text(message, sizeof(message), length, ",conf=");
    length = append_u32(message, sizeof(message), length, report.observation.confidence_permille);
    length = append_text(message, sizeof(message), length, ",age_ms=");
    length = append_u32(message, sizeof(message), length, report.observation_age_ms);
    length = append_text(message, sizeof(message), length, ",err_0p1mm=");
    length = append_i16(message, sizeof(message), length, report.error_0_1mm);
    length = append_text(message, sizeof(message), length, ",u_0p001mm=");
    length = append_i16(message, sizeof(message), length, report.output_0_001mm);
    length = append_text(message, sizeof(message), length, ",dir=");
    length = append_u32(message, sizeof(message), length, report.command_positive_direction ? 1U : 0U);
    length = append_text(message, sizeof(message), length, ",pulses=");
    length = append_u32(message, sizeof(message), length, report.command_pulses);
    length = append_text(message, sizeof(message), length, ",frames=");
    length = append_u32(message, sizeof(message), length, report.accepted_observation_count);
    length = append_text(message, sizeof(message), length, ",invalid=");
    length = append_u32(message, sizeof(message), length, report.invalid_observation_count);
    length = append_text(message, sizeof(message), length, ",stale=");
    length = append_u32(message, sizeof(message), length, report.stale_observation_count);
    length = append_text(message, sizeof(message), length, ",duplicate=");
    length = append_u32(message, sizeof(message), length, report.duplicate_sequence_count);
    length = append_text(message, sizeof(message), length, ",candidates=");
    length = append_u32(message, sizeof(message), length, report.command_candidate_count);
    length = append_text(message, sizeof(message), length, ",crc=");
    length = append_u32(message, sizeof(message), length, telemetry->parser->crc_error_count);
    length = append_text(message, sizeof(message), length, ",format=");
    length = append_u32(message, sizeof(message), length, telemetry->parser->format_error_count);
    length = append_text(message, sizeof(message), length, ",semantic=");
    length = append_u32(message, sizeof(message), length, telemetry->parser->semantic_error_count);
    length = append_text(message, sizeof(message), length, ",rx_overflow=");
    length = append_u32(message, sizeof(message), length, telemetry->vision_port->rx_overflow_count);
    length = append_text(message, sizeof(message), length, ",uart_error=");
    length = append_u32(message, sizeof(message), length, telemetry->vision_port->uart_error_count);
    length = append_text(message, sizeof(message), length, "\r\n");
    return write_bytes(telemetry, message, length);
}

bool PitchAxisVisionTelemetry_Init(
    PitchAxisVisionTelemetry *telemetry,
    BspBluetooth *output,
    const BspUartDmaPort *vision_port,
    const BallObservationParser *parser,
    const PitchAxisVisionControl *control,
    uint32_t now_ms)
{
    if ((telemetry == NULL) || (output == NULL) || (vision_port == NULL) ||
        (parser == NULL) || (control == NULL))
    {
        return false;
    }

    memset(telemetry, 0, sizeof(*telemetry));
    telemetry->output = output;
    telemetry->vision_port = vision_port;
    telemetry->parser = parser;
    telemetry->control = control;
    telemetry->next_status_ms = now_ms + PITCH_VISION_STATUS_PERIOD_MS;
    telemetry->initialized = true;
    return true;
}

void PitchAxisVisionTelemetry_Service(
    PitchAxisVisionTelemetry *telemetry,
    uint32_t now_ms)
{
    if ((telemetry == NULL) || !telemetry->initialized)
    {
        return;
    }
    if (!service_boot(telemetry))
    {
        return;
    }
    if ((int32_t)(now_ms - telemetry->next_status_ms) >= 0)
    {
        if (write_status(telemetry))
        {
            telemetry->next_status_ms = now_ms + PITCH_VISION_STATUS_PERIOD_MS;
        }
    }
}
