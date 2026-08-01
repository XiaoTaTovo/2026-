#include "pitch_pid_debug.h"

#include <string.h>

#define PITCH_PID_DEBUG_MAX_KP_MILLI 10000U
#define PITCH_PID_DEBUG_MAX_KI_MILLI 5000U
#define PITCH_PID_DEBUG_MAX_KD_MILLI 10000U
#define PITCH_PID_DEBUG_MAX_ILIM_MILLI 30000U
#define PITCH_PID_DEBUG_ABSOLUTE_POSITION_0_1MM 1250
#define PITCH_PID_DEBUG_MAX_TARGET_0_1MM \
    PITCH_PID_DEBUG_ABSOLUTE_POSITION_0_1MM
#define PITCH_PID_DEBUG_MAX_EDGE_MARGIN_0_1MM 1000
#define PITCH_PID_DEBUG_MAX_VISION_RPM 50U
#define PITCH_PID_DEBUG_MIN_CONTROL_PERIOD_MS 20U
#define PITCH_PID_DEBUG_MAX_CONTROL_PERIOD_MS 1000U
#define PITCH_PID_DEBUG_MIN_OBSERVATION_AGE_MS 50U
#define PITCH_PID_DEBUG_MAX_OBSERVATION_AGE_MS 1000U
#define PITCH_PID_DEBUG_MAX_AUTO_TIMEOUT_MS 1000U
#define PITCH_PID_DEBUG_MAX_AUTO_BUDGET_MS 600000U
#define PITCH_PID_DEBUG_MAX_MANUAL_RPM 50U
#define PITCH_PID_DEBUG_MAX_RUN_MS 1000U
#define PITCH_PID_DEBUG_MAX_RAW_PER_MM 1000000U
#define PITCH_PID_DEBUG_MAX_TILT_UM 10000U
#define PITCH_PID_DEBUG_MAX_POSITION_POLL_MS 200U
#define PITCH_PID_DEBUG_MAX_FF_MILLI 500U
#define PITCH_PID_DEBUG_MAX_FF_LIMIT_MILLI 50000U
#define PITCH_PID_DEBUG_MAX_FF_DEADBAND_MM_S2 5000U

static bool time_elapsed(uint32_t now_ms, uint32_t start_ms, uint32_t delay_ms)
{
    return (uint32_t)(now_ms - start_ms) >= delay_ms;
}

static void handle_single_character(
    PitchPidDebug *debug,
    char value,
    uint32_t now_ms);

static char upper_char(char value)
{
    if ((value >= 'a') && (value <= 'z'))
    {
        return (char)(value - ('a' - 'A'));
    }
    return value;
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

static size_t append_u32(
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

static size_t append_i32(
    char *message,
    size_t capacity,
    size_t length,
    int32_t value)
{
    uint32_t magnitude;

    if (value < 0)
    {
        if (length < capacity)
        {
            message[length++] = '-';
        }
        magnitude = (uint32_t)(-(value + 1)) + 1U;
    }
    else
    {
        magnitude = (uint32_t)value;
    }
    return append_u32(message, capacity, length, magnitude);
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

static size_t append_fixed3(
    char *message,
    size_t capacity,
    size_t length,
    float value)
{
    uint32_t milli;

    if (value < 0.0f)
    {
        value = 0.0f;
    }
    milli = (uint32_t)(value * 1000.0f + 0.5f);
    length = append_u32(message, capacity, length, milli / 1000U);
    if (length < capacity)
    {
        message[length++] = '.';
    }
    if (length < capacity)
    {
        message[length++] = (char)('0' + ((milli / 100U) % 10U));
    }
    if (length < capacity)
    {
        message[length++] = (char)('0' + ((milli / 10U) % 10U));
    }
    if (length < capacity)
    {
        message[length++] = (char)('0' + (milli % 10U));
    }
    return length;
}

/* Signed fixed-point for the plot stream. append_fixed3 clamps negatives to
 * zero, which would hide the sign of err, ballv and the PID terms. */
static size_t append_scaled(
    char *message,
    size_t capacity,
    size_t length,
    int32_t value,
    uint32_t divisor,
    uint8_t decimals)
{
    uint32_t magnitude;
    uint32_t fraction;
    uint32_t scale = 1U;
    uint8_t index;

    if (value < 0)
    {
        if (length < capacity)
        {
            message[length++] = '-';
        }
        magnitude = (uint32_t)(-(value + 1)) + 1U;
    }
    else
    {
        magnitude = (uint32_t)value;
    }

    length = append_u32(message, capacity, length, magnitude / divisor);
    if (decimals == 0U)
    {
        return length;
    }
    if (length < capacity)
    {
        message[length++] = '.';
    }
    fraction = magnitude % divisor;
    for (index = 0U; index < decimals; ++index)
    {
        scale *= 10U;
    }
    fraction = (fraction * scale) / divisor;
    while (scale > 1U)
    {
        scale /= 10U;
        if (length < capacity)
        {
            message[length++] = (char)('0' + ((fraction / scale) % 10U));
        }
    }
    return length;
}

static bool write_bytes(
    PitchPidDebug *debug,
    const char *message,
    size_t length)
{
    if (BspBluetooth_TxFree(debug->bluetooth) < length)
    {
        return false;
    }
    return BspBluetooth_Write(
               debug->bluetooth,
               (const uint8_t *)message,
               length) == BSP_BLUETOOTH_OK;
}

static bool write_text(PitchPidDebug *debug, const char *text)
{
    return write_bytes(debug, text, strlen(text));
}

static void service_config_response(PitchPidDebug *debug)
{
    size_t remaining;
    size_t available;
    size_t chunk_length;

    if (!debug->config_response_pending)
    {
        return;
    }

    remaining = debug->config_response_length - debug->config_response_offset;
    available = BspBluetooth_TxFree(debug->bluetooth);
    chunk_length = remaining;
    if (chunk_length > PITCH_PID_DEBUG_TX_CHUNK_SIZE)
    {
        chunk_length = PITCH_PID_DEBUG_TX_CHUNK_SIZE;
    }
    if (chunk_length > available)
    {
        chunk_length = available;
    }
    if ((chunk_length == 0U) ||
        !write_bytes(
            debug,
            &debug->config_response[debug->config_response_offset],
            chunk_length))
    {
        return;
    }

    debug->config_response_offset += chunk_length;
    if (debug->config_response_offset >= debug->config_response_length)
    {
        debug->config_response_length = 0U;
        debug->config_response_offset = 0U;
        debug->config_response_pending = false;
    }
}

static bool text_equal(const char *left, const char *right)
{
    size_t index = 0U;

    while ((left[index] != '\0') && (right[index] != '\0'))
    {
        if (upper_char(left[index]) != upper_char(right[index]))
        {
            return false;
        }
        index++;
    }
    return (left[index] == '\0') && (right[index] == '\0');
}

static char *skip_spaces(char *text)
{
    while ((*text == ' ') || (*text == '\t'))
    {
        text++;
    }
    return text;
}

static void trim_end(char *text)
{
    size_t length = strlen(text);

    while ((length > 0U) &&
           ((text[length - 1U] == ' ') || (text[length - 1U] == '\t')))
    {
        text[--length] = '\0';
    }
}

static bool parse_u32(const char *text, uint32_t *value)
{
    uint32_t result = 0U;
    size_t index = 0U;

    if ((text == NULL) || (text[0] == '\0'))
    {
        return false;
    }
    while (text[index] != '\0')
    {
        uint32_t digit;

        if ((text[index] < '0') || (text[index] > '9'))
        {
            return false;
        }
        digit = (uint32_t)(text[index] - '0');
        if (result > (UINT32_MAX - digit) / 10U)
        {
            return false;
        }
        result = result * 10U + digit;
        index++;
    }
    *value = result;
    return true;
}

static bool parse_i32(const char *text, int32_t *value)
{
    bool negative = false;
    uint32_t magnitude;
    const char *digits = text;

    if (digits == NULL)
    {
        return false;
    }
    if ((*digits == '-') || (*digits == '+'))
    {
        negative = (*digits == '-');
        digits++;
    }
    if (!parse_u32(digits, &magnitude))
    {
        return false;
    }
    if (negative)
    {
        if (magnitude > 0x80000000UL)
        {
            return false;
        }
        *value = (magnitude == 0x80000000UL) ? INT32_MIN :
            -(int32_t)magnitude;
    }
    else
    {
        if (magnitude > 0x7FFFFFFFUL)
        {
            return false;
        }
        *value = (int32_t)magnitude;
    }
    return true;
}

static bool parse_decimal_milli(const char *text, int32_t *value)
{
    bool negative = false;
    uint32_t whole = 0U;
    uint32_t fraction = 0U;
    uint8_t fraction_digits = 0U;
    size_t index = 0U;
    bool have_digit = false;

    if (text == NULL)
    {
        return false;
    }
    if ((text[index] == '-') || (text[index] == '+'))
    {
        negative = (text[index] == '-');
        index++;
    }
    while ((text[index] >= '0') && (text[index] <= '9'))
    {
        uint32_t digit = (uint32_t)(text[index] - '0');
        if (whole > (UINT32_MAX - digit) / 10U)
        {
            return false;
        }
        whole = whole * 10U + digit;
        have_digit = true;
        index++;
    }
    if (text[index] == '.')
    {
        index++;
        while ((text[index] >= '0') && (text[index] <= '9'))
        {
            if (fraction_digits >= 3U)
            {
                return false;
            }
            fraction = fraction * 10U + (uint32_t)(text[index] - '0');
            fraction_digits++;
            have_digit = true;
            index++;
        }
    }
    while (fraction_digits < 3U)
    {
        fraction *= 10U;
        fraction_digits++;
    }
    if (!have_digit || (text[index] != '\0') ||
        (whole > (INT32_MAX - fraction) / 1000U))
    {
        return false;
    }
    *value = (int32_t)(whole * 1000U + fraction);
    if (negative)
    {
        *value = -*value;
    }
    return true;
}

static bool split_name_value(
    char *line,
    char **name,
    char **value)
{
    char *cursor = skip_spaces(line);
    char *separator;

    if ((cursor[0] == 'S') && (cursor[1] == 'E') &&
        (cursor[2] == 'T') &&
        ((cursor[3] == ' ') || (cursor[3] == '\t')))
    {
        cursor = skip_spaces(cursor + 3);
    }
    separator = cursor;
    while ((*separator != '\0') && (*separator != '=') &&
           (*separator != ' ') && (*separator != '\t'))
    {
        separator++;
    }
    if (*separator == '\0')
    {
        return false;
    }
    *separator = '\0';
    separator++;
    separator = skip_spaces(separator);
    if (*separator == '=')
    {
        separator = skip_spaces(separator + 1);
    }
    trim_end(separator);
    if (*separator == '\0')
    {
        return false;
    }
    *name = cursor;
    *value = separator;
    return true;
}

static const char *velocity_state_name(PitchAxisVelocityTestState state)
{
    switch (state)
    {
        case PITCH_VELOCITY_TEST_STATE_LOCKED_WAIT_SELF_TEST:
            return "LOCKED";
        case PITCH_VELOCITY_TEST_STATE_DISABLED_READY:
            return "DISABLED_READY";
        case PITCH_VELOCITY_TEST_STATE_ENABLED_STOPPED:
            return "ENABLED_STOPPED";
        case PITCH_VELOCITY_TEST_STATE_RUNNING_AUTOMATIC:
            return "RUNNING_AUTO";
        case PITCH_VELOCITY_TEST_STATE_WAIT_AUTOMATIC_STOP_ACK:
            return "WAIT_AUTO_STOP";
        case PITCH_VELOCITY_TEST_STATE_FAULT_LATCHED:
            return "FAULT_LATCHED";
        default:
            return "BUSY";
    }
}

static const char *vision_state_name(PitchAxisVisionState state)
{
    switch (state)
    {
        case PITCH_VISION_STATE_WAITING_FOR_FRAME:
            return "WAIT_FRAME";
        case PITCH_VISION_STATE_TRACKING:
            return "TRACKING";
        case PITCH_VISION_STATE_REJECT_INVALID:
            return "INVALID";
        case PITCH_VISION_STATE_REJECT_STALE:
            return "STALE";
        case PITCH_VISION_STATE_REJECT_LOW_CONFIDENCE:
            return "LOW_CONF";
        default:
            return "UNKNOWN";
    }
}

static const char *automatic_reason_name(
    PitchAxisAutomaticDisarmReason reason)
{
    switch (reason)
    {
        case PITCH_AUTOMATIC_DISARM_USER: return "USER";
        case PITCH_AUTOMATIC_DISARM_BALL_ESCAPE: return "BALL_ESCAPE";
        case PITCH_AUTOMATIC_DISARM_VISION_INVALID: return "INVALID";
        case PITCH_AUTOMATIC_DISARM_VISION_LOW_CONFIDENCE: return "LOW_CONF";
        case PITCH_AUTOMATIC_DISARM_VISION_STALE: return "STALE";
        case PITCH_AUTOMATIC_DISARM_DECISION_TIMEOUT: return "TIMEOUT";
        case PITCH_AUTOMATIC_DISARM_EDGE_RECOVERY_TIMEOUT:
            return "EDGE_TIMEOUT";
        case PITCH_AUTOMATIC_DISARM_BUDGET: return "BUDGET";
        case PITCH_AUTOMATIC_DISARM_FAULT: return "FAULT";
        default: return "NONE";
    }
}

static bool write_config(PitchPidDebug *debug)
{
    PitchAxisVisionConfig vision_config;
    PitchAxisVelocityTestConfig velocity_config;
    PitchAxisVelocityTestReport velocity_report;
    PitchTaskControllerConfig task_config;
    PitchTaskControllerReport task_report;
    char message[896];
    size_t length = 0U;

    if (debug->config_response_pending)
    {
        return false;
    }

    if (!PitchAxisVisionControl_GetConfig(debug->vision, &vision_config) ||
        !PitchAxisVelocityTest_GetConfig(debug->velocity, &velocity_config) ||
        !PitchAxisVelocityTest_GetReport(debug->velocity, &velocity_report) ||
        ((debug->tasks != NULL) &&
         (!PitchTaskController_GetConfig(debug->tasks, &task_config) ||
          !PitchTaskController_GetReport(debug->tasks, &task_report))))
    {
        return false;
    }

    length = append_text(message, sizeof(message), length, "PID_CONFIG,kp=");
    length = append_fixed3(message, sizeof(message), length,
                           vision_config.kp_rpm_per_mm);
    length = append_text(message, sizeof(message), length, ",ki=");
    length = append_fixed3(message, sizeof(message), length,
                           vision_config.ki_rpm_per_mm_s);
    length = append_text(message, sizeof(message), length, ",kd=");
    length = append_fixed3(message, sizeof(message), length,
                           vision_config.kd_rpm_per_mm_s);
    length = append_text(message, sizeof(message), length, ",ilim=");
    length = append_fixed3(message, sizeof(message), length,
                           vision_config.integral_limit_rpm);
    length = append_text(message, sizeof(message), length, ",iband=");
    length = append_i32(message, sizeof(message), length,
                        vision_config.integral_separation_band_0_1mm);
    length = append_text(message, sizeof(message), length, ",appband=");
    length = append_i32(message, sizeof(message), length,
                        vision_config.approach_band_0_1mm);
    length = append_text(message, sizeof(message), length, ",appmax=");
    length = append_u32(message, sizeof(message), length,
                        vision_config.approach_speed_limit_rpm);
    length = append_text(message, sizeof(message), length, ",target=");
    length = append_i32(message, sizeof(message), length,
                        vision_config.target_position_0_1mm);
    length = append_text(message, sizeof(message), length, ",ballmin=");
    length = append_i32(message, sizeof(message), length,
                        vision_config.minimum_safe_position_0_1mm);
    length = append_text(message, sizeof(message), length, ",ballmax=");
    length = append_i32(message, sizeof(message), length,
                        vision_config.maximum_safe_position_0_1mm);
    length = append_text(message, sizeof(message), length, ",edgemargin=");
    length = append_i32(message, sizeof(message), length,
                        vision_config.edge_recovery_margin_0_1mm);
    length = append_text(message, sizeof(message), length, ",db=");
    length = append_i32(message, sizeof(message), length,
                        vision_config.deadband_0_1mm);
    length = append_text(message, sizeof(message), length, ",vdb=");
    length = append_i32(message, sizeof(message), length,
                        vision_config.velocity_deadband_0_1mm_s);
    length = append_text(message, sizeof(message), length, ",conf=");
    length = append_u32(message, sizeof(message), length,
                        vision_config.minimum_confidence_permille);
    length = append_text(message, sizeof(message), length, ",age=");
    length = append_u32(message, sizeof(message), length,
                        vision_config.maximum_observation_age_ms);
    length = append_text(message, sizeof(message), length, ",period=");
    length = append_u32(message, sizeof(message), length,
                        vision_config.control_period_ms);
    length = append_text(message, sizeof(message), length, ",minrpm=");
    length = append_u32(message, sizeof(message), length,
                        vision_config.minimum_speed_rpm);
    length = append_text(message, sizeof(message), length, ",maxrpm=");
    length = append_u32(message, sizeof(message), length,
                        vision_config.maximum_speed_rpm);
    length = append_text(message, sizeof(message), length, ",alpha=");
    length = append_fixed3(message, sizeof(message), length,
                           vision_config.velocity_filter_alpha);
    length = append_text(message, sizeof(message), length, ",sign=");
    length = append_u32(message, sizeof(message), length,
                        vision_config.positive_error_uses_positive_direction ?
                            1U : 0U);
    length = append_text(message, sizeof(message), length, ",ffen=");
    length = append_u32(message, sizeof(message), length,
                        vision_config.feedforward_enabled ? 1U : 0U);
    length = append_text(message, sizeof(message), length, ",ff=");
    length = append_fixed3(message, sizeof(message), length,
                           vision_config.feedforward_gain_rpm_per_mm_s2);
    length = append_text(message, sizeof(message), length, ",ffsign=");
    length = append_i32(message, sizeof(message), length,
                        vision_config.feedforward_sign);
    length = append_text(message, sizeof(message), length, ",fflim=");
    length = append_fixed3(message, sizeof(message), length,
                           vision_config.feedforward_limit_rpm);
    length = append_text(message, sizeof(message), length, ",ffdb=");
    length = append_fixed3(message, sizeof(message), length,
                           vision_config.feedforward_deadband_mm_s2);
    length = append_text(message, sizeof(message), length, ",automax=");
    length = append_u32(message, sizeof(message), length,
                         velocity_config.automatic_max_speed_rpm);
    length = append_text(message, sizeof(message), length, ",postrack=");
    length = append_u32(
        message, sizeof(message), length,
        velocity_config.automatic_position_tracking_enabled ? 1U : 0U);
    length = append_text(message, sizeof(message), length, ",rawsign=");
    length = append_u32(
        message, sizeof(message), length,
        velocity_config.automatic_direction0_increases_raw ? 1U : 0U);
    length = append_text(message, sizeof(message), length, ",rawpmm=");
    length = append_u32(message, sizeof(message), length,
                        velocity_config.automatic_position_raw_per_mm);
    length = append_text(message, sizeof(message), length, ",tiltscale=");
    length = append_u32(
        message, sizeof(message), length,
        velocity_config.automatic_tilt_scale_um_per_outer_rpm);
    length = append_text(message, sizeof(message), length, ",tiltlim=");
    length = append_u32(message, sizeof(message), length,
                        velocity_config.automatic_tilt_limit_um);
    length = append_text(message, sizeof(message), length, ",posdb=");
    length = append_u32(message, sizeof(message), length,
                        velocity_config.automatic_position_deadband_um);
    length = append_text(message, sizeof(message), length, ",slowum=");
    length = append_u32(message, sizeof(message), length,
                        velocity_config.automatic_position_slow_zone_um);
    length = append_text(message, sizeof(message), length, ",innermin=");
    length = append_u32(message, sizeof(message), length,
                        velocity_config.automatic_position_min_speed_rpm);
    length = append_text(message, sizeof(message), length, ",pospoll=");
    length = append_u32(message, sizeof(message), length,
                        velocity_config.automatic_position_poll_period_ms);
    length = append_text(message, sizeof(message), length, ",timeout=");
    length = append_u32(message, sizeof(message), length,
                        velocity_config.automatic_decision_timeout_ms);
    length = append_text(message, sizeof(message), length, ",lossms=");
    length = append_u32(message, sizeof(message), length,
                        velocity_config.automatic_vision_loss_grace_ms);
    length = append_text(message, sizeof(message), length, ",rescue=");
    length = append_u32(
        message,
        sizeof(message),
        length,
        velocity_config.automatic_edge_recovery_enabled ? 1U : 0U);
    length = append_text(message, sizeof(message), length, ",rescuerpm=");
    length = append_u32(
        message,
        sizeof(message),
        length,
        velocity_config.automatic_edge_recovery_speed_rpm);
    length = append_text(message, sizeof(message), length, ",rescuems=");
    length = append_u32(
        message,
        sizeof(message),
        length,
        velocity_config.automatic_edge_recovery_max_ms);
    length = append_text(message, sizeof(message), length, ",budget=");
    length = append_u32(message, sizeof(message), length,
                        velocity_config.automatic_motion_budget_ms);
    length = append_text(message, sizeof(message), length, ",accel=");
    length = append_u32(message, sizeof(message), length,
                         velocity_config.acceleration);
    length = append_text(message, sizeof(message), length, ",manualrpm=");
    length = append_u32(message, sizeof(message), length,
                        velocity_config.speed_rpm);
    length = append_text(message, sizeof(message), length, ",runms=");
    length = append_u32(message, sizeof(message), length,
                        velocity_config.run_ms);
    length = append_text(message, sizeof(message), length, ",posdir=");
    length = append_u32(message, sizeof(message), length,
                        velocity_config.positive_direction);
    length = append_text(message, sizeof(message), length, ",negdir=");
    length = append_u32(message, sizeof(message), length,
                        velocity_config.negative_direction);
    length = append_text(message, sizeof(message), length, ",sync=");
    length = append_u32(message, sizeof(message), length,
                        velocity_config.synchronize ? 1U : 0U);
    length = append_text(message, sizeof(message), length, ",sample=");
    length = append_u32(message, sizeof(message), length,
                        debug->sample_period_ms);
    length = append_text(message, sizeof(message), length, ",enabled=");
    length = append_u32(message, sizeof(message), length,
                        debug->enabled ? 1U : 0U);
    length = append_text(message, sizeof(message), length, ",pid_debug=");
    length = append_u32(message, sizeof(message), length,
                        velocity_report.pid_debug_enabled ? 1U : 0U);
    if (debug->tasks != NULL)
    {
        length = append_text(message, sizeof(message), length, ",task=");
        length = append_u32(message, sizeof(message), length,
                            task_report.selected_task);
        length = append_text(message, sizeof(message), length, ",task_state=");
        length = append_text(
            message, sizeof(message), length,
            PitchTaskController_StateName(task_report.state));
        length = append_text(message, sizeof(message), length, ",pid_profile=");
        length = append_u32(message, sizeof(message), length,
                            task_report.pid_profile);
        length = append_text(message, sizeof(message), length, ",center=");
        length = append_i32(message, sizeof(message), length,
                            task_config.center_position_0_1mm);
        length = append_text(message, sizeof(message), length, ",t3offset=");
        length = append_u32(message, sizeof(message), length,
                            task_config.task3_offset_0_1mm);
        length = append_text(message, sizeof(message), length, ",t3tol=");
        length = append_u32(message, sizeof(message), length,
                            task_config.task3_tolerance_0_1mm);
        length = append_text(message, sizeof(message), length, ",t3vmax=");
        length = append_u32(
            message,
            sizeof(message),
            length,
            task_config.task3_velocity_limit_0_1mm_s);
        length = append_text(message, sizeof(message), length, ",t3dwell=");
        length = append_u32(message, sizeof(message), length,
                            task_config.task3_turnaround_dwell_ms);
        length = append_text(message, sizeof(message), length, ",holdtilt=");
        length = append_u32(
            message, sizeof(message), length,
            task_config.position_hold_tilt_limit_um);
        length = append_text(message, sizeof(message), length, ",t3tilt=");
        length = append_u32(
            message, sizeof(message), length,
            task_config.task3_tilt_limit_um);
    }
    length = append_text(message, sizeof(message), length, "\r\n");
    if (length > sizeof(debug->config_response))
    {
        return false;
    }
    memcpy(debug->config_response, message, length);
    debug->config_response_length = length;
    debug->config_response_offset = 0U;
    debug->config_response_pending = true;
    return true;
}

static bool write_sample(PitchPidDebug *debug, uint32_t now_ms)
{
    PitchAxisVisionConfig vision_config;
    PitchAxisVisionReport vision_report;
    PitchAxisVelocityTestReport velocity_report;
    PitchTaskControllerReport task_report;
    char message[896];
    size_t length = 0U;

    if (!PitchAxisVisionControl_GetConfig(debug->vision, &vision_config) ||
        !PitchAxisVisionControl_GetReport(debug->vision, &vision_report) ||
        !PitchAxisVelocityTest_GetReport(debug->velocity, &velocity_report) ||
        ((debug->tasks != NULL) &&
         !PitchTaskController_GetReport(debug->tasks, &task_report)))
    {
        return false;
    }

    length = append_text(message, sizeof(message), length, "PID_SAMPLE,t=");
    length = append_u32(message, sizeof(message), length, now_ms);
    length = append_text(message, sizeof(message), length, ",vstate=");
    length = append_text(message, sizeof(message), length,
                         vision_state_name(vision_report.state));
    length = append_text(message, sizeof(message), length, ",valid=");
    length = append_u32(message, sizeof(message), length,
                        vision_report.observation.valid ? 1U : 0U);
    length = append_text(message, sizeof(message), length, ",reason=");
    length = append_u32(message, sizeof(message), length,
                        vision_report.observation.reason);
    length = append_text(message, sizeof(message), length, ",conf=");
    length = append_u32(message, sizeof(message), length,
                        vision_report.observation.confidence_permille);
    length = append_text(message, sizeof(message), length, ",motor=");
    length = append_text(message, sizeof(message), length,
                         velocity_state_name(velocity_report.state));
    if (debug->tasks != NULL)
    {
        length = append_text(message, sizeof(message), length, ",task=");
        length = append_u32(message, sizeof(message), length,
                            task_report.selected_task);
        length = append_text(message, sizeof(message), length, ",tstate=");
        length = append_text(
            message, sizeof(message), length,
            PitchTaskController_StateName(task_report.state));
        length = append_text(message, sizeof(message), length, ",ttarget=");
        length = append_i32(message, sizeof(message), length,
                            task_report.target_position_0_1mm);
        length = append_text(message, sizeof(message), length, ",tcapture=");
        length = append_i32(message, sizeof(message), length,
                            task_report.captured_position_0_1mm);
        length = append_text(message, sizeof(message), length, ",tcapok=");
        length = append_u32(
            message, sizeof(message), length,
            task_report.captured_position_valid ? 1U : 0U);
    }
    length = append_text(message, sizeof(message), length, ",x=");
    length = append_i32(message, sizeof(message), length,
                        vision_report.observation.x_0_1mm);
    length = append_text(message, sizeof(message), length, ",target=");
    length = append_i32(message, sizeof(message), length,
                         vision_config.target_position_0_1mm);
    length = append_text(message, sizeof(message), length, ",escape=");
    length = append_u32(message, sizeof(message), length,
                        vision_report.ball_position_outside_limits ? 1U : 0U);
    length = append_text(message, sizeof(message), length, ",err=");
    length = append_i32(message, sizeof(message), length,
                        vision_report.error_0_1mm);
    length = append_text(message, sizeof(message), length, ",ballv=");
    length = append_i32(message, sizeof(message), length,
                        vision_report.ball_velocity_0_1mm_s);
    length = append_text(message, sizeof(message), length, ",p=");
    length = append_i32(message, sizeof(message), length,
                        vision_report.p_term_0_01rpm);
    length = append_text(message, sizeof(message), length, ",i=");
    length = append_i32(message, sizeof(message), length,
                        vision_report.i_term_0_01rpm);
    length = append_text(message, sizeof(message), length, ",ien=");
    length = append_u32(message, sizeof(message), length,
                        vision_report.integral_active ? 1U : 0U);
    length = append_text(message, sizeof(message), length, ",alim=");
    length = append_u32(message, sizeof(message), length,
                        vision_report.approach_limited ? 1U : 0U);
    length = append_text(message, sizeof(message), length, ",d=");
    length = append_i32(message, sizeof(message), length,
                        vision_report.d_term_0_01rpm);
    length = append_text(message, sizeof(message), length, ",ff=");
    length = append_i32(message, sizeof(message), length,
                        vision_report.feedforward_0_01rpm);
    length = append_text(message, sizeof(message), length, ",ff_in=");
    length = append_i32(message, sizeof(message), length,
                        vision_report.feedforward_input_mm_s2);
    length = append_text(message, sizeof(message), length, ",ff_valid=");
    length = append_u32(message, sizeof(message), length,
                        vision_report.feedforward_valid ? 1U : 0U);
    length = append_text(message, sizeof(message), length, ",ff_sat=");
    length = append_u32(message, sizeof(message), length,
                        vision_report.feedforward_saturated ? 1U : 0U);
    length = append_text(message, sizeof(message), length, ",u=");
    length = append_i32(message, sizeof(message), length,
                        vision_report.unsaturated_output_0_01rpm);
    length = append_text(message, sizeof(message), length, ",out=");
    length = append_i32(message, sizeof(message), length,
                        vision_report.control_output_0_01rpm);
    length = append_text(message, sizeof(message), length, ",sat=");
    length = append_u32(message, sizeof(message), length,
                        vision_report.output_saturated ? 1U : 0U);
    length = append_text(message, sizeof(message), length, ",cmd=");
    length = append_u32(message, sizeof(message), length,
                        vision_report.command_speed_rpm);
    length = append_text(message, sizeof(message), length, ",dir=");
    length = append_u32(message, sizeof(message), length,
                         vision_report.command_positive_direction ? 1U : 0U);
    length = append_text(message, sizeof(message), length, ",posok=");
    length = append_u32(
        message, sizeof(message), length,
        velocity_report.automatic_position_valid ? 1U : 0U);
    length = append_text(message, sizeof(message), length, ",zraw=");
    length = append_i64(message, sizeof(message), length,
                        velocity_report.automatic_zero_position_raw);
    length = append_text(message, sizeof(message), length, ",praw=");
    length = append_i64(message, sizeof(message), length,
                        velocity_report.automatic_position_raw);
    length = append_text(message, sizeof(message), length, ",ptgt=");
    length = append_i64(message, sizeof(message), length,
                        velocity_report.automatic_target_position_raw);
    length = append_text(message, sizeof(message), length, ",perr=");
    length = append_i64(message, sizeof(message), length,
                        velocity_report.automatic_position_error_raw);
    length = append_text(message, sizeof(message), length, ",page=");
    length = append_u32(message, sizeof(message), length,
                        velocity_report.automatic_position_age_ms);
    length = append_text(message, sizeof(message), length, ",fresh=");
    length = append_u32(message, sizeof(message), length,
                        vision_report.observation_fresh ? 1U : 0U);
    length = append_text(message, sizeof(message), length, ",age=");
    length = append_u32(message, sizeof(message), length,
                        vision_report.observation_age_ms);
    length = append_text(message, sizeof(message), length, ",auto=");
    length = append_u32(message, sizeof(message), length,
                        velocity_report.automatic_armed ? 1U : 0U);
    length = append_text(message, sizeof(message), length, ",motion=");
    length = append_u32(message, sizeof(message), length,
                        velocity_report.automatic_motion_active ? 1U : 0U);
    length = append_text(message, sizeof(message), length, ",hold=");
    length = append_u32(message, sizeof(message), length,
                        velocity_report.automatic_hold ? 1U : 0U);
    length = append_text(message, sizeof(message), length, ",loss=");
    length = append_u32(message, sizeof(message), length,
                        velocity_report.automatic_vision_loss_active ? 1U : 0U);
    length = append_text(message, sizeof(message), length, ",loss_age=");
    length = append_u32(message, sizeof(message), length,
                        velocity_report.automatic_vision_loss_age_ms);
    length = append_text(message, sizeof(message), length, ",loss_reason=");
    length = append_text(
        message,
        sizeof(message),
        length,
        automatic_reason_name(velocity_report.automatic_vision_loss_reason));
    length = append_text(message, sizeof(message), length, ",edge=");
    length = append_u32(
        message,
        sizeof(message),
        length,
        vision_report.edge_recovery_candidate ? 1U : 0U);
    length = append_text(message, sizeof(message), length, ",rescue=");
    length = append_u32(
        message,
        sizeof(message),
        length,
        velocity_report.automatic_edge_recovery_active ? 1U : 0U);
    length = append_text(message, sizeof(message), length, ",rescue_age=");
    length = append_u32(
        message,
        sizeof(message),
        length,
        velocity_report.automatic_edge_recovery_age_ms);
    length = append_text(message, sizeof(message), length, ",rescue_dir=");
    length = append_u32(
        message,
        sizeof(message),
        length,
        velocity_report.automatic_edge_recovery_direction);
    length = append_text(message, sizeof(message), length, ",rescues=");
    length = append_u32(
        message,
        sizeof(message),
        length,
        velocity_report.automatic_edge_recovery_count);
    length = append_text(message, sizeof(message), length, ",auto_reason=");
    length = append_text(
        message,
        sizeof(message),
        length,
        automatic_reason_name(velocity_report.automatic_disarm_reason));
    length = append_text(message, sizeof(message), length, ",budget_ms=");
    length = append_u32(message, sizeof(message), length,
                        velocity_report.automatic_budget_used_ms);
    length = append_text(message, sizeof(message), length, ",fault=");
    length = append_u32(message, sizeof(message), length,
                        velocity_report.fault_latched ? 1U : 0U);
    length = append_text(message, sizeof(message), length, ",seq=");
    length = append_u32(message, sizeof(message), length,
                        vision_report.decision_sequence);
    length = append_text(message, sizeof(message), length, "\r\n");
    return write_bytes(debug, message, length);
}

/* VOFA+ FireWater: "name1:value1,name2:value2\n". The header line names the
 * channels once so the plot legend is readable after a fresh connect. */
static bool write_plot_header(PitchPidDebug *debug)
{
    return write_text(
        debug,
        "x:0,target:0,err:0,ballv:0,p:0,i:0,d:0,out:0,"
        "perr_um:0,cmd:0,conf:0,track:0\n");
}

/* Scaled to millimetres so every position channel shares one Y axis:
 * 0.1 mm fields are divided by 10, raw counts by RAWPMM. */
static bool write_plot_sample(PitchPidDebug *debug)
{
    PitchAxisVisionConfig vision_config;
    PitchAxisVisionReport vision_report;
    PitchAxisVelocityTestConfig velocity_config;
    PitchAxisVelocityTestReport velocity_report;
    char message[256];
    size_t length = 0U;
    int32_t position_error_um = 0;

    if (!PitchAxisVisionControl_GetConfig(debug->vision, &vision_config) ||
        !PitchAxisVisionControl_GetReport(debug->vision, &vision_report) ||
        !PitchAxisVelocityTest_GetConfig(debug->velocity, &velocity_config) ||
        !PitchAxisVelocityTest_GetReport(debug->velocity, &velocity_report))
    {
        return false;
    }

    if (velocity_config.automatic_position_raw_per_mm != 0U)
    {
        int64_t error_raw = velocity_report.automatic_position_error_raw;
        int64_t scaled = (error_raw * 1000) /
            (int64_t)velocity_config.automatic_position_raw_per_mm;

        if (scaled > INT32_MAX)
        {
            scaled = INT32_MAX;
        }
        else if (scaled < INT32_MIN)
        {
            scaled = INT32_MIN;
        }
        position_error_um = (int32_t)scaled;
    }

    length = append_text(message, sizeof(message), length, "x:");
    length = append_scaled(message, sizeof(message), length,
                           vision_report.observation.x_0_1mm, 10U, 1U);
    length = append_text(message, sizeof(message), length, ",target:");
    length = append_scaled(message, sizeof(message), length,
                           vision_config.target_position_0_1mm, 10U, 1U);
    length = append_text(message, sizeof(message), length, ",err:");
    length = append_scaled(message, sizeof(message), length,
                           vision_report.error_0_1mm, 10U, 1U);
    length = append_text(message, sizeof(message), length, ",ballv:");
    length = append_scaled(message, sizeof(message), length,
                           vision_report.ball_velocity_0_1mm_s, 10U, 1U);
    length = append_text(message, sizeof(message), length, ",p:");
    length = append_scaled(message, sizeof(message), length,
                           vision_report.p_term_0_01rpm, 100U, 2U);
    length = append_text(message, sizeof(message), length, ",i:");
    length = append_scaled(message, sizeof(message), length,
                           vision_report.i_term_0_01rpm, 100U, 2U);
    length = append_text(message, sizeof(message), length, ",d:");
    length = append_scaled(message, sizeof(message), length,
                           vision_report.d_term_0_01rpm, 100U, 2U);
    length = append_text(message, sizeof(message), length, ",out:");
    length = append_scaled(message, sizeof(message), length,
                           vision_report.control_output_0_01rpm, 100U, 2U);
    length = append_text(message, sizeof(message), length, ",perr_um:");
    length = append_i32(message, sizeof(message), length, position_error_um);
    length = append_text(message, sizeof(message), length, ",cmd:");
    length = append_u32(message, sizeof(message), length,
                        vision_report.command_speed_rpm);
    length = append_text(message, sizeof(message), length, ",conf:");
    length = append_u32(message, sizeof(message), length,
                        vision_report.observation.confidence_permille);
    /* Plot-friendly state flag: 1 while tracking, 0 on any reject path. */
    length = append_text(message, sizeof(message), length, ",track:");
    length = append_u32(
        message, sizeof(message), length,
        (vision_report.state == PITCH_VISION_STATE_TRACKING) ? 1U : 0U);
    length = append_text(message, sizeof(message), length, "\n");
    return write_bytes(debug, message, length);
}

static bool write_task_status(PitchPidDebug *debug)
{
    PitchTaskControllerReport report;
    char message[220];
    size_t length = 0U;

    if ((debug->tasks == NULL) ||
        !PitchTaskController_GetReport(debug->tasks, &report))
    {
        return false;
    }
    length = append_text(message, sizeof(message), length, "PITCH_TASK,task=");
    length = append_u32(message, sizeof(message), length,
                        report.selected_task);
    length = append_text(message, sizeof(message), length, ",state=");
    length = append_text(
        message, sizeof(message), length,
        PitchTaskController_StateName(report.state));
    length = append_text(message, sizeof(message), length, ",pid_profile=");
    length = append_u32(message, sizeof(message), length,
                        report.pid_profile);
    length = append_text(message, sizeof(message), length, ",target=");
    length = append_i32(message, sizeof(message), length,
                        report.target_position_0_1mm);
    length = append_text(message, sizeof(message), length, ",capture=");
    length = append_i32(message, sizeof(message), length,
                        report.captured_position_0_1mm);
    length = append_text(message, sizeof(message), length, ",capture_valid=");
    length = append_u32(
        message, sizeof(message), length,
        report.captured_position_valid ? 1U : 0U);
    length = append_text(message, sizeof(message), length, ",armed=");
    length = append_u32(message, sizeof(message), length,
                        report.automatic_armed ? 1U : 0U);
    length = append_text(message, sizeof(message), length, ",enabled=");
    length = append_u32(message, sizeof(message), length,
                        report.motor_enabled ? 1U : 0U);
    length = append_text(message, sizeof(message), length, "\r\n");
    return write_bytes(debug, message, length);
}

static bool service_boot(PitchPidDebug *debug)
{
    const char *text = NULL;

    if (!debug->boot_report_pending)
    {
        return true;
    }
    switch (debug->boot_line)
    {
        case 0U: text = "PID_DEBUG_READY\r\n"; break;
        case 1U: text = "PID_DEBUG_BUILD=pid-live-r20-control-continuity\r\n"; break;
        case 2U:
            text = (debug->tasks != NULL) ?
                "PID_COMMANDS=PING|PID ON|PID OFF|PID?|PID HELP|PLOT ON|PLOT OFF|SET NAME=VALUE|ZERO|RESUME\r\n" :
                "PID_COMMANDS=PING|PID ON|PID OFF|PID?|PID HELP|PLOT ON|PLOT OFF|SET NAME=VALUE|ZERO|A|D|RESUME\r\n";
            break;
        case 3U: text = "PID_SAMPLE_DEFAULT_MS=100\r\n"; break;
        case 4U:
            text = "PID_RUNTIME_SET=KP,KI,KD,ILIM,IBAND,APPBAND,APPMAX,TARGET,BALLMIN,BALLMAX,EDGEMARGIN,DB,VDB,CONF,AGE,PERIOD,MINRPM,MAXRPM,ALPHA,SIGN,AUTOMAX,POSTRACK,RAWSIGN,RAWPMM,TILTSCALE,TILTLIM,POSDB,SLOWUM,INNERMIN,POSPOLL,TIMEOUT,LOSSMS,RESCUE,RESCUERPM,RESCUEMS,BUDGET,ACCEL,CENTER,T3OFFSET,T3TOL,T3VMAX,T3DWELL,HOLDTILT,T3TILT,SAMPLE\r\n";
            break;
        case 5U:
            text = "PID_IDLE_ONLY_SET=MANUALRPM,RUNMS,POSDIR,NEGDIR,SYNC,POSTRACK,RAWSIGN,RAWPMM,ZERO\r\n";
            break;
        case 6U: text = "PID_HARD_AUTO_MAX_RPM=300\r\n"; break;
        case 7U:
            text = "PID_HARD_POSITION_0P1MM=-1250..1250\r\n";
            break;
        case 8U:
            text = (debug->tasks != NULL) ?
                "PID_TASK_CONTROL=KEYS_ONLY_BLUETOOTH_A_D_REJECTED\r\n" :
                "PID_AUTO_START=AFTER_COMM_ENABLE_ACK\r\n";
            break;
        case 9U:
            text = (debug->tasks != NULL) ?
                "PID_KEY1=START_SELECTED_TASK\r\n" :
                "PID_KEY1=IGNORED_AUTO_START\r\n";
            break;
        case 10U:
            text = (debug->tasks != NULL) ?
                "PID_KEY2=NEXT_TASK\r\n" :
                "PID_KEY2=BALL_ESCAPE_HOLD\r\n";
            break;
        case 11U:
            text = (debug->tasks != NULL) ?
                "PID_KEY3=NEXT_PID_PROFILE_OR_TASK6_CAPTURE\r\n" :
                "PID_KEY3=RESUME_PERMISSION\r\n";
            break;
        case 12U: text = "PID_KEY4=STOP_AND_LATCH\r\n"; break;
        default:
            debug->boot_report_pending = false;
            return true;
    }
    if (write_text(debug, text))
    {
        debug->boot_line++;
    }
    return false;
}

static void write_command_result(
    PitchPidDebug *debug,
    bool accepted,
    const char *name,
    const char *value)
{
    char message[180];
    size_t length = 0U;

    length = append_text(
        message,
        sizeof(message),
        length,
        accepted ? "PID_SET_OK,name=" : "PID_SET_REJECTED,name=");
    length = append_text(message, sizeof(message), length, name);
    if (value != NULL)
    {
        length = append_text(message, sizeof(message), length, ",value=");
        length = append_text(message, sizeof(message), length, value);
    }
    length = append_text(message, sizeof(message), length, "\r\n");
    (void)write_bytes(debug, message, length);
}

static bool apply_setting(
    PitchPidDebug *debug,
    char *name,
    char *value,
    uint32_t now_ms)
{
    PitchAxisVisionConfig vision_config;
    PitchAxisVelocityTestConfig velocity_config;
    PitchTaskControllerConfig task_config;
    int32_t signed_value;
    uint32_t unsigned_value;
    bool vision_changed = false;
    bool velocity_changed = false;
    bool task_changed = false;

    if (text_equal(name, "SAMPLE"))
    {
        if (!parse_u32(value, &unsigned_value) ||
            (unsigned_value < PITCH_PID_DEBUG_MIN_SAMPLE_PERIOD_MS) ||
            (unsigned_value > PITCH_PID_DEBUG_MAX_SAMPLE_PERIOD_MS))
        {
            return false;
        }
        debug->sample_period_ms = unsigned_value;
        debug->next_sample_ms = now_ms + debug->sample_period_ms;
        return true;
    }

    if (text_equal(name, "PLOT"))
    {
        if (!parse_u32(value, &unsigned_value) ||
            (unsigned_value < PITCH_PID_DEBUG_MIN_PLOT_PERIOD_MS) ||
            (unsigned_value > PITCH_PID_DEBUG_MAX_PLOT_PERIOD_MS))
        {
            return false;
        }
        debug->plot_period_ms = unsigned_value;
        debug->next_plot_ms = now_ms + debug->plot_period_ms;
        return true;
    }

    if (!PitchAxisVisionControl_GetConfig(debug->vision, &vision_config) ||
        !PitchAxisVelocityTest_GetConfig(debug->velocity, &velocity_config))
    {
        return false;
    }

    if ((debug->tasks != NULL) &&
        !PitchTaskController_GetConfig(debug->tasks, &task_config))
    {
        return false;
    }

    if (text_equal(name, "KP") || text_equal(name, "KI") ||
        text_equal(name, "KD") || text_equal(name, "ILIM") ||
        text_equal(name, "ALPHA"))
    {
        if (!parse_decimal_milli(value, &signed_value) || (signed_value < 0))
        {
            return false;
        }
        if (text_equal(name, "KP"))
        {
            if ((uint32_t)signed_value > PITCH_PID_DEBUG_MAX_KP_MILLI)
            {
                return false;
            }
            vision_config.kp_rpm_per_mm = (float)signed_value / 1000.0f;
        }
        else if (text_equal(name, "KI"))
        {
            if ((uint32_t)signed_value > PITCH_PID_DEBUG_MAX_KI_MILLI)
            {
                return false;
            }
            vision_config.ki_rpm_per_mm_s =
                (float)signed_value / 1000.0f;
        }
        else if (text_equal(name, "KD"))
        {
            if ((uint32_t)signed_value > PITCH_PID_DEBUG_MAX_KD_MILLI)
            {
                return false;
            }
            vision_config.kd_rpm_per_mm_s =
                (float)signed_value / 1000.0f;
        }
        else if (text_equal(name, "ILIM"))
        {
            if ((uint32_t)signed_value > PITCH_PID_DEBUG_MAX_ILIM_MILLI)
            {
                return false;
            }
            vision_config.integral_limit_rpm =
                (float)signed_value / 1000.0f;
        }
        else
        {
            if ((signed_value <= 0) || (signed_value > 1000))
            {
                return false;
            }
            vision_config.velocity_filter_alpha =
                (float)signed_value / 1000.0f;
        }
        vision_changed = true;
    }
    else if (text_equal(name, "FF") || text_equal(name, "FFLIM") ||
             text_equal(name, "FFDB"))
    {
        if (!parse_decimal_milli(value, &signed_value) || (signed_value < 0))
        {
            return false;
        }
        if (text_equal(name, "FF"))
        {
            if ((uint32_t)signed_value > PITCH_PID_DEBUG_MAX_FF_MILLI)
            {
                return false;
            }
            vision_config.feedforward_gain_rpm_per_mm_s2 =
                (float)signed_value / 1000.0f;
        }
        else if (text_equal(name, "FFLIM"))
        {
            if ((uint32_t)signed_value > PITCH_PID_DEBUG_MAX_FF_LIMIT_MILLI)
            {
                return false;
            }
            vision_config.feedforward_limit_rpm =
                (float)signed_value / 1000.0f;
        }
        else
        {
            if ((uint32_t)signed_value >
                (PITCH_PID_DEBUG_MAX_FF_DEADBAND_MM_S2 * 1000U))
            {
                return false;
            }
            vision_config.feedforward_deadband_mm_s2 =
                (float)signed_value / 1000.0f;
        }
        vision_changed = true;
    }
    else if (text_equal(name, "FFSIGN"))
    {
        if (!parse_i32(value, &signed_value) ||
            ((signed_value != -1) && (signed_value != 1)))
        {
            return false;
        }
        vision_config.feedforward_sign = (int8_t)signed_value;
        vision_changed = true;
    }
    else if (text_equal(name, "FFEN"))
    {
        if (!parse_u32(value, &unsigned_value) || (unsigned_value > 1U))
        {
            return false;
        }
        vision_config.feedforward_enabled = unsigned_value != 0U;
        vision_changed = true;
    }
    else if (text_equal(name, "IBAND"))
    {
        if (!parse_i32(value, &signed_value) ||
            (signed_value < 0) || (signed_value > 5000))
        {
            return false;
        }
        vision_config.integral_separation_band_0_1mm = (int16_t)signed_value;
        vision_changed = true;
    }
    else if (text_equal(name, "APPBAND") || text_equal(name, "APPMAX"))
    {
        if (!parse_u32(value, &unsigned_value))
        {
            return false;
        }
        if (text_equal(name, "APPBAND"))
        {
            if (unsigned_value > 1000U)
            {
                return false;
            }
            vision_config.approach_band_0_1mm = (int16_t)unsigned_value;
        }
        else
        {
            if ((unsigned_value == 0U) ||
                (unsigned_value < vision_config.minimum_speed_rpm) ||
                (unsigned_value > vision_config.maximum_speed_rpm))
            {
                return false;
            }
            vision_config.approach_speed_limit_rpm = (uint16_t)unsigned_value;
        }
        vision_changed = true;
    }
    else if (text_equal(name, "TARGET") || text_equal(name, "BALLMIN") ||
             text_equal(name, "BALLMAX") ||
             text_equal(name, "EDGEMARGIN") || text_equal(name, "DB") ||
             text_equal(name, "VDB"))
    {
        if (!parse_i32(value, &signed_value))
        {
            return false;
        }
        if (text_equal(name, "TARGET"))
        {
            if ((signed_value < -PITCH_PID_DEBUG_MAX_TARGET_0_1MM) ||
                (signed_value > PITCH_PID_DEBUG_MAX_TARGET_0_1MM) ||
                (signed_value <
                 vision_config.minimum_safe_position_0_1mm) ||
                (signed_value >
                 vision_config.maximum_safe_position_0_1mm))
            {
                return false;
            }
            vision_config.target_position_0_1mm = (int16_t)signed_value;
        }
        else if (text_equal(name, "BALLMIN"))
        {
            if ((signed_value <
                 -PITCH_PID_DEBUG_ABSOLUTE_POSITION_0_1MM) ||
                (signed_value >
                 PITCH_PID_DEBUG_ABSOLUTE_POSITION_0_1MM) ||
                (signed_value >=
                 vision_config.maximum_safe_position_0_1mm) ||
                (signed_value > vision_config.target_position_0_1mm))
            {
                return false;
            }
            vision_config.minimum_safe_position_0_1mm =
                (int16_t)signed_value;
        }
        else if (text_equal(name, "BALLMAX"))
        {
            if ((signed_value <
                 -PITCH_PID_DEBUG_ABSOLUTE_POSITION_0_1MM) ||
                (signed_value >
                 PITCH_PID_DEBUG_ABSOLUTE_POSITION_0_1MM) ||
                (signed_value <=
                 vision_config.minimum_safe_position_0_1mm) ||
                (signed_value < vision_config.target_position_0_1mm))
            {
                return false;
            }
            vision_config.maximum_safe_position_0_1mm =
                (int16_t)signed_value;
        }
        else if (text_equal(name, "EDGEMARGIN"))
        {
            if ((signed_value < 0) ||
                (signed_value > PITCH_PID_DEBUG_MAX_EDGE_MARGIN_0_1MM))
            {
                return false;
            }
            vision_config.edge_recovery_margin_0_1mm =
                (int16_t)signed_value;
        }
        else if (text_equal(name, "DB"))
        {
            if ((signed_value < 0) || (signed_value > 1000))
            {
                return false;
            }
            vision_config.deadband_0_1mm = (int16_t)signed_value;
        }
        else
        {
            if ((signed_value < 0) || (signed_value > 5000))
            {
                return false;
            }
            vision_config.velocity_deadband_0_1mm_s = (int16_t)signed_value;
        }
        vision_changed = true;
    }
    else if (text_equal(name, "CONF") || text_equal(name, "AGE") ||
             text_equal(name, "PERIOD") || text_equal(name, "MINRPM") ||
             text_equal(name, "MAXRPM") || text_equal(name, "SIGN"))
    {
        if (!parse_u32(value, &unsigned_value))
        {
            return false;
        }
        if (text_equal(name, "CONF"))
        {
            if (unsigned_value > 1000U)
            {
                return false;
            }
            vision_config.minimum_confidence_permille =
                (uint16_t)unsigned_value;
        }
        else if (text_equal(name, "AGE"))
        {
            if ((unsigned_value < PITCH_PID_DEBUG_MIN_OBSERVATION_AGE_MS) ||
                (unsigned_value > PITCH_PID_DEBUG_MAX_OBSERVATION_AGE_MS))
            {
                return false;
            }
            vision_config.maximum_observation_age_ms = unsigned_value;
        }
        else if (text_equal(name, "PERIOD"))
        {
            if ((unsigned_value < PITCH_PID_DEBUG_MIN_CONTROL_PERIOD_MS) ||
                (unsigned_value > PITCH_PID_DEBUG_MAX_CONTROL_PERIOD_MS))
            {
                return false;
            }
            vision_config.control_period_ms = unsigned_value;
        }
        else if (text_equal(name, "MINRPM"))
        {
            if ((unsigned_value == 0U) ||
                (unsigned_value > PITCH_PID_DEBUG_MAX_VISION_RPM))
            {
                return false;
            }
            vision_config.minimum_speed_rpm = (uint16_t)unsigned_value;
        }
        else if (text_equal(name, "MAXRPM"))
        {
            if ((unsigned_value == 0U) ||
                (unsigned_value > PITCH_PID_DEBUG_MAX_VISION_RPM))
            {
                return false;
            }
            vision_config.maximum_speed_rpm = (uint16_t)unsigned_value;
        }
        else
        {
            if (unsigned_value > 1U)
            {
                return false;
            }
            vision_config.positive_error_uses_positive_direction =
                unsigned_value != 0U;
        }
        vision_changed = true;
    }
    else if (text_equal(name, "CENTER") ||
             text_equal(name, "T3OFFSET") ||
             text_equal(name, "T3TOL") ||
             text_equal(name, "T3VMAX") ||
             text_equal(name, "T3DWELL") ||
             text_equal(name, "HOLDTILT") ||
             text_equal(name, "T3TILT"))
    {
        if (debug->tasks == NULL)
        {
            return false;
        }
        if (text_equal(name, "CENTER"))
        {
            if (!parse_i32(value, &signed_value) ||
                (signed_value < vision_config.minimum_safe_position_0_1mm) ||
                (signed_value > vision_config.maximum_safe_position_0_1mm))
            {
                return false;
            }
            task_config.center_position_0_1mm = (int16_t)signed_value;
        }
        else
        {
            if (!parse_u32(value, &unsigned_value))
            {
                return false;
            }
            if (text_equal(name, "T3OFFSET"))
            {
                if ((unsigned_value == 0U) || (unsigned_value > 1250U))
                {
                    return false;
                }
                task_config.task3_offset_0_1mm = (uint16_t)unsigned_value;
            }
            else if (text_equal(name, "T3TOL"))
            {
                if ((unsigned_value == 0U) || (unsigned_value > 500U))
                {
                    return false;
                }
                task_config.task3_tolerance_0_1mm = (uint16_t)unsigned_value;
            }
            else if (text_equal(name, "T3VMAX"))
            {
                if ((unsigned_value == 0U) ||
                    (unsigned_value > UINT16_MAX))
                {
                    return false;
                }
                task_config.task3_velocity_limit_0_1mm_s =
                    (uint16_t)unsigned_value;
            }
            else if (text_equal(name, "T3DWELL"))
            {
                if (unsigned_value > 5000U)
                {
                    return false;
                }
                task_config.task3_turnaround_dwell_ms = unsigned_value;
            }
            else if (text_equal(name, "HOLDTILT"))
            {
                if ((unsigned_value < 100U) ||
                    (unsigned_value > PITCH_PID_DEBUG_MAX_TILT_UM))
                {
                    return false;
                }
                task_config.position_hold_tilt_limit_um =
                    (uint16_t)unsigned_value;
            }
            else
            {
                if ((unsigned_value < 100U) ||
                    (unsigned_value > PITCH_PID_DEBUG_MAX_TILT_UM))
                {
                    return false;
                }
                task_config.task3_tilt_limit_um = (uint16_t)unsigned_value;
            }
        }
        task_changed = true;
    }
    else if (text_equal(name, "AUTOMAX") || text_equal(name, "POSTRACK") ||
              text_equal(name, "RAWSIGN") || text_equal(name, "RAWPMM") ||
              text_equal(name, "TILTSCALE") || text_equal(name, "TILTLIM") ||
              text_equal(name, "POSDB") || text_equal(name, "SLOWUM") ||
              text_equal(name, "INNERMIN") || text_equal(name, "POSPOLL") ||
              text_equal(name, "TIMEOUT") ||
             text_equal(name, "LOSSMS") || text_equal(name, "RESCUE") ||
             text_equal(name, "RESCUERPM") || text_equal(name, "RESCUEMS") ||
             text_equal(name, "BUDGET") || text_equal(name, "ACCEL") ||
             text_equal(name, "MANUALRPM") || text_equal(name, "RUNMS") ||
             text_equal(name, "POSDIR") || text_equal(name, "NEGDIR") ||
             text_equal(name, "SYNC"))
    {
        if (!parse_u32(value, &unsigned_value))
        {
            return false;
        }
        if (text_equal(name, "AUTOMAX"))
        {
            if ((unsigned_value == 0U) ||
                (unsigned_value > PITCH_AXIS_VELOCITY_TEST_HARD_AUTO_MAX_RPM))
            {
                return false;
            }
            velocity_config.automatic_max_speed_rpm =
                (uint16_t)unsigned_value;
        }
        else if (text_equal(name, "POSTRACK"))
        {
            if (unsigned_value > 1U)
            {
                return false;
            }
            velocity_config.automatic_position_tracking_enabled =
                unsigned_value != 0U;
        }
        else if (text_equal(name, "RAWSIGN"))
        {
            if (unsigned_value > 1U)
            {
                return false;
            }
            velocity_config.automatic_direction0_increases_raw =
                unsigned_value != 0U;
        }
        else if (text_equal(name, "RAWPMM"))
        {
            if ((unsigned_value < 1000U) ||
                (unsigned_value > PITCH_PID_DEBUG_MAX_RAW_PER_MM))
            {
                return false;
            }
            velocity_config.automatic_position_raw_per_mm = unsigned_value;
        }
        else if (text_equal(name, "TILTSCALE"))
        {
            if ((unsigned_value == 0U) ||
                (unsigned_value > 1000U))
            {
                return false;
            }
            velocity_config.automatic_tilt_scale_um_per_outer_rpm =
                (uint16_t)unsigned_value;
        }
        else if (text_equal(name, "TILTLIM"))
        {
            if ((unsigned_value < 100U) ||
                (unsigned_value > PITCH_PID_DEBUG_MAX_TILT_UM))
            {
                return false;
            }
            velocity_config.automatic_tilt_limit_um =
                (uint16_t)unsigned_value;
        }
        else if (text_equal(name, "POSDB"))
        {
            if ((unsigned_value == 0U) ||
                (unsigned_value >= velocity_config.automatic_tilt_limit_um))
            {
                return false;
            }
            velocity_config.automatic_position_deadband_um =
                (uint16_t)unsigned_value;
        }
        else if (text_equal(name, "SLOWUM"))
        {
            if ((unsigned_value <=
                 velocity_config.automatic_position_deadband_um) ||
                (unsigned_value > velocity_config.automatic_tilt_limit_um))
            {
                return false;
            }
            velocity_config.automatic_position_slow_zone_um =
                (uint16_t)unsigned_value;
        }
        else if (text_equal(name, "INNERMIN"))
        {
            if ((unsigned_value == 0U) ||
                (unsigned_value > velocity_config.automatic_max_speed_rpm))
            {
                return false;
            }
            velocity_config.automatic_position_min_speed_rpm =
                (uint16_t)unsigned_value;
        }
        else if (text_equal(name, "POSPOLL"))
        {
            if ((unsigned_value < 5U) ||
                (unsigned_value > PITCH_PID_DEBUG_MAX_POSITION_POLL_MS))
            {
                return false;
            }
            velocity_config.automatic_position_poll_period_ms = unsigned_value;
        }
        else if (text_equal(name, "TIMEOUT"))
        {
            if ((unsigned_value < 100U) ||
                (unsigned_value > PITCH_PID_DEBUG_MAX_AUTO_TIMEOUT_MS))
            {
                return false;
            }
            velocity_config.automatic_decision_timeout_ms = unsigned_value;
        }
        else if (text_equal(name, "LOSSMS"))
        {
            if (unsigned_value >
                PITCH_AXIS_VELOCITY_TEST_HARD_MAX_VISION_LOSS_GRACE_MS)
            {
                return false;
            }
            velocity_config.automatic_vision_loss_grace_ms = unsigned_value;
        }
        else if (text_equal(name, "RESCUE"))
        {
            if (unsigned_value > 1U)
            {
                return false;
            }
            velocity_config.automatic_edge_recovery_enabled =
                unsigned_value != 0U;
        }
        else if (text_equal(name, "RESCUERPM"))
        {
            if ((unsigned_value == 0U) ||
                (unsigned_value >
                 PITCH_AXIS_VELOCITY_TEST_HARD_EDGE_RECOVERY_MAX_RPM))
            {
                return false;
            }
            velocity_config.automatic_edge_recovery_speed_rpm =
                (uint16_t)unsigned_value;
        }
        else if (text_equal(name, "RESCUEMS"))
        {
            if ((unsigned_value < 50U) ||
                (unsigned_value >
                 PITCH_AXIS_VELOCITY_TEST_HARD_MAX_EDGE_RECOVERY_MS))
            {
                return false;
            }
            velocity_config.automatic_edge_recovery_max_ms = unsigned_value;
        }
        else if (text_equal(name, "BUDGET"))
        {
            if (unsigned_value > PITCH_PID_DEBUG_MAX_AUTO_BUDGET_MS)
            {
                return false;
            }
            velocity_config.automatic_motion_budget_ms = unsigned_value;
        }
        else if (text_equal(name, "ACCEL"))
        {
            /* ZDT speed mode reserves zero for immediate acceleration. */
            if (unsigned_value > 255U)
            {
                return false;
            }
            velocity_config.acceleration = (uint8_t)unsigned_value;
        }
        else if (text_equal(name, "MANUALRPM"))
        {
            if ((unsigned_value == 0U) ||
                (unsigned_value > PITCH_PID_DEBUG_MAX_MANUAL_RPM))
            {
                return false;
            }
            velocity_config.speed_rpm = (uint16_t)unsigned_value;
        }
        else if (text_equal(name, "RUNMS"))
        {
            if ((unsigned_value == 0U) ||
                (unsigned_value > PITCH_PID_DEBUG_MAX_RUN_MS))
            {
                return false;
            }
            velocity_config.run_ms = unsigned_value;
        }
        else if (text_equal(name, "POSDIR"))
        {
            if (unsigned_value > 1U)
            {
                return false;
            }
            velocity_config.positive_direction = (uint8_t)unsigned_value;
        }
        else if (text_equal(name, "NEGDIR"))
        {
            if (unsigned_value > 1U)
            {
                return false;
            }
            velocity_config.negative_direction = (uint8_t)unsigned_value;
        }
        else
        {
            if (unsigned_value > 1U)
            {
                return false;
            }
            velocity_config.synchronize = unsigned_value != 0U;
        }
        velocity_changed = true;
    }
    else
    {
        return false;
    }

    /* PID and automatic safety settings are live. The velocity layer permits
     * those fields while running and still rejects manual transport changes. */
    if (vision_changed &&
        ((debug->tasks != NULL) ?
         !PitchTaskController_UpdateActivePidConfig(
             debug->tasks, &vision_config, now_ms) :
         !PitchAxisVisionControl_UpdateConfig(
             debug->vision, &vision_config, now_ms)))
    {
        return false;
    }
    if (velocity_changed &&
        !PitchAxisVelocityTest_UpdateConfig(debug->velocity, &velocity_config))
    {
        return false;
    }
    if (task_changed &&
        !PitchTaskController_UpdateConfig(debug->tasks, &task_config, now_ms))
    {
        return false;
    }
    return true;
}

static void handle_command(
    PitchPidDebug *debug,
    char *line,
    uint32_t now_ms)
{
    char *name;
    char *value;

    trim_end(line);
    if (line[0] == '\0')
    {
        return;
    }
    if (text_equal(line, "PING"))
    {
        debug->accepted_command_count++;
        (void)write_text(debug, "PONG\r\n");
        return;
    }
    if (text_equal(line, "PID HELP"))
    {
        debug->accepted_command_count++;
        (void)write_text(
            debug,
             "PID_HELP=outer:p/i/d/u/out=0.01virtual_rpm;position:zraw/praw/ptgt/perr=raw_counts;ZERO=recapture_zero,A=ARM,D=DISARM\r\n");
        return;
    }
    if (text_equal(line, "A"))
    {
        handle_single_character(debug, 'A', now_ms);
        return;
    }
    if (text_equal(line, "D"))
    {
        handle_single_character(debug, 'D', now_ms);
        return;
    }
    if (text_equal(line, "ZERO"))
    {
        if (PitchAxisVelocityTest_CaptureAutomaticZero(
                debug->velocity,
                now_ms))
        {
            debug->accepted_command_count++;
            (void)write_text(debug, "POSITION_ZERO_REQUESTED\r\n");
        }
        else
        {
            debug->rejected_command_count++;
            (void)write_text(debug, "POSITION_ZERO_REJECTED\r\n");
        }
        return;
    }
    if (text_equal(line, "PID ON"))
    {
        if (PitchAxisVelocityTest_SetPidDebugEnabled(
                debug->velocity, true, now_ms))
        {
            debug->enabled = true;
            debug->next_sample_ms = now_ms + debug->sample_period_ms;
            (void)write_text(debug, "PID_ON_OK\r\n");
        }
        else
        {
            debug->rejected_command_count++;
            (void)write_text(debug, "PID_ON_REJECTED\r\n");
        }
        return;
    }
    if (text_equal(line, "PID OFF"))
    {
        if (PitchAxisVelocityTest_SetPidDebugEnabled(
                debug->velocity, false, now_ms))
        {
            debug->enabled = false;
            (void)write_text(debug, "PID_OFF_OK\r\n");
        }
        else
        {
            debug->rejected_command_count++;
            (void)write_text(debug, "PID_OFF_REJECTED\r\n");
        }
        return;
    }
    if (text_equal(line, "PLOT ON"))
    {
        debug->accepted_command_count++;
        debug->plot_enabled = true;
        debug->plot_header_pending = true;
        debug->next_plot_ms = now_ms;
        (void)write_text(debug, "PLOT_ON_OK\r\n");
        return;
    }
    if (text_equal(line, "PLOT OFF"))
    {
        debug->accepted_command_count++;
        debug->plot_enabled = false;
        (void)write_text(debug, "PLOT_OFF_OK\r\n");
        return;
    }
    if (text_equal(line, "PID?") || text_equal(line, "PID STATUS"))
    {
        (void)write_config(debug);
        return;
    }
    if (text_equal(line, "PID RESET"))
    {
        PitchAxisVisionControl_ResetController(debug->vision, now_ms);
        (void)write_text(debug, "PID_RESET_OK\r\n");
        return;
    }
    if (text_equal(line, "RESUME"))
    {
        if (PitchAxisVelocityTest_ClearAutomaticHold(debug->velocity))
        {
            PitchAxisVisionControl_ResetController(debug->vision, now_ms);
            (void)write_text(debug, "PID_RESUME_READY\r\n");
        }
        else
        {
            debug->rejected_command_count++;
            (void)write_text(debug, "PID_RESUME_REJECTED\r\n");
        }
        return;
    }
    if (!split_name_value(line, &name, &value))
    {
        debug->rejected_command_count++;
        (void)write_text(debug, "PID_COMMAND_ERROR\r\n");
        return;
    }
    if (apply_setting(debug, name, value, now_ms))
    {
        debug->accepted_command_count++;
        write_command_result(debug, true, name, value);
    }
    else
    {
        debug->rejected_command_count++;
        write_command_result(debug, false, name, value);
    }
}

static void handle_single_character(
    PitchPidDebug *debug,
    char value,
    uint32_t now_ms)
{
    if ((value == 'A') || (value == 'a'))
    {
        if (debug->tasks != NULL)
        {
            debug->rejected_command_count++;
            (void)write_text(debug, "AUTO_ARM_REJECTED_USE_KEY1\r\n");
            return;
        }
        PitchAxisVisionControl_ResetController(debug->vision, now_ms);
        if (PitchAxisVelocityTest_SetAutomaticArmed(
                debug->velocity, true, now_ms))
        {
            debug->accepted_command_count++;
            (void)write_text(debug, "AUTO_ARM_OK\r\n");
        }
        else
        {
            debug->rejected_command_count++;
            (void)write_text(debug, "AUTO_ARM_REJECTED\r\n");
        }
    }
    else if ((value == 'D') || (value == 'd'))
    {
        if (debug->tasks != NULL)
        {
            debug->rejected_command_count++;
            (void)write_text(debug, "AUTO_DISARM_REJECTED_USE_KEY4\r\n");
            return;
        }
        if (PitchAxisVelocityTest_SetAutomaticArmed(
                debug->velocity, false, now_ms))
        {
            debug->accepted_command_count++;
            (void)write_text(debug, "AUTO_DISARM_OK\r\n");
        }
        else
        {
            debug->rejected_command_count++;
            (void)write_text(debug, "AUTO_DISARM_REJECTED\r\n");
        }
    }
}

static void read_commands(PitchPidDebug *debug, uint32_t now_ms)
{
    uint8_t data[64];
    size_t available;
    size_t read_length;
    size_t index;

    available = BspBluetooth_Available(debug->bluetooth);
    if (available > sizeof(data))
    {
        available = sizeof(data);
    }
    if ((available == 0U) ||
        (BspBluetooth_Read(
             debug->bluetooth,
             data,
             available,
             &read_length) != BSP_BLUETOOTH_OK))
    {
        if (debug->pending_single_active &&
            time_elapsed(
                now_ms,
                debug->pending_single_since_ms,
                PITCH_PID_DEBUG_SINGLE_COMMAND_TIMEOUT_MS))
        {
            handle_single_character(
                debug,
                debug->pending_single_command,
                now_ms);
            debug->pending_single_active = false;
        }
        return;
    }

    if (debug->pending_single_active)
    {
        if ((data[0] == '\r') || (data[0] == '\n'))
        {
            handle_single_character(
                debug,
                debug->pending_single_command,
                now_ms);
            debug->pending_single_active = false;
        }
        else
        {
            debug->line[0] = debug->pending_single_command;
            debug->line_length = 1U;
            debug->pending_single_active = false;
        }
    }

    for (index = 0U; index < read_length; ++index)
    {
        char value = (char)data[index];

        if ((debug->line_length == 0U) && !debug->pending_single_active &&
            ((value == 'A') || (value == 'a') ||
             (value == 'D') || (value == 'd')))
        {
            if ((index + 1U < read_length) &&
                ((data[index + 1U] == '\r') ||
                 (data[index + 1U] == '\n')))
            {
                handle_single_character(debug, value, now_ms);
            }
            else if (index + 1U < read_length)
            {
                debug->line[debug->line_length++] = upper_char(value);
            }
            else
            {
                debug->pending_single_command = upper_char(value);
                debug->pending_single_since_ms = now_ms;
                debug->pending_single_active = true;
            }
            continue;
        }
        if ((value == '\r') || (value == '\n'))
        {
            if (debug->line_length != 0U)
            {
                debug->line[debug->line_length] = '\0';
                handle_command(debug, debug->line, now_ms);
                debug->line_length = 0U;
            }
            continue;
        }
        if (debug->line_length >= (PITCH_PID_DEBUG_LINE_SIZE - 1U))
        {
            debug->line_length = 0U;
            debug->rejected_command_count++;
            (void)write_text(debug, "PID_COMMAND_ERROR=TOO_LONG\r\n");
            continue;
        }
        debug->line[debug->line_length++] = upper_char(value);
    }
}

bool PitchPidDebug_Init(
    PitchPidDebug *debug,
    BspBluetooth *bluetooth,
    PitchAxisVisionControl *vision,
    PitchAxisVelocityTest *velocity,
    PitchTaskController *tasks,
    uint32_t now_ms)
{
    if ((debug == NULL) || (bluetooth == NULL) || (vision == NULL) ||
        (velocity == NULL))
    {
        return false;
    }

    memset(debug, 0, sizeof(*debug));
    debug->bluetooth = bluetooth;
    debug->vision = vision;
    debug->velocity = velocity;
    debug->tasks = tasks;
    debug->sample_period_ms = PITCH_PID_DEBUG_DEFAULT_SAMPLE_PERIOD_MS;
    debug->next_sample_ms = now_ms + debug->sample_period_ms;
    debug->plot_period_ms = PITCH_PID_DEBUG_DEFAULT_PLOT_PERIOD_MS;
    debug->next_plot_ms = now_ms + debug->plot_period_ms;
    debug->last_task_transition_count = UINT32_MAX;
    debug->boot_report_pending = true;
    debug->initialized = true;
    return true;
}

void PitchPidDebug_Service(
    PitchPidDebug *debug,
    uint32_t now_ms)
{
    if ((debug == NULL) || !debug->initialized)
    {
        return;
    }

    if (debug->config_response_pending)
    {
        service_config_response(debug);
        return;
    }

    read_commands(debug, now_ms);
    if (debug->config_response_pending)
    {
        service_config_response(debug);
        return;
    }
    if (!service_boot(debug))
    {
        return;
    }
    if (debug->tasks != NULL)
    {
        PitchTaskControllerReport task_report;

        if (PitchTaskController_GetReport(debug->tasks, &task_report) &&
            (task_report.transition_count !=
             debug->last_task_transition_count) &&
            write_task_status(debug))
        {
            debug->last_task_transition_count =
                task_report.transition_count;
        }
    }
    if (debug->enabled &&
        ((int32_t)(now_ms - debug->next_sample_ms) >= 0))
    {
        if (write_sample(debug, now_ms))
        {
            debug->next_sample_ms = now_ms + debug->sample_period_ms;
        }
    }
    if (debug->plot_enabled &&
        ((int32_t)(now_ms - debug->next_plot_ms) >= 0))
    {
        if (debug->plot_header_pending)
        {
            if (write_plot_header(debug))
            {
                debug->plot_header_pending = false;
            }
        }
        else if (write_plot_sample(debug))
        {
            debug->next_plot_ms = now_ms + debug->plot_period_ms;
        }
    }
}

bool PitchPidDebug_IsEnabled(const PitchPidDebug *debug)
{
    return (debug != NULL) && debug->initialized && debug->enabled;
}
