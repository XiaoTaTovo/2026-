#include "vofa_telemetry.h"

#include <stdlib.h>
#include <string.h>

#include "tb6612.h"
#include "ti_msp_dl_config.h"

#define VOFA_RX_RING_SIZE (128U)
#define VOFA_RX_RING_MASK (VOFA_RX_RING_SIZE - 1U)
#define VOFA_COMMAND_SIZE (64U)
#define VOFA_COMMAND_IDLE_MS (100U)
#define VOFA_TX_RING_SIZE (1024U)
#define VOFA_TX_RING_MASK (VOFA_TX_RING_SIZE - 1U)
#define VOFA_FRAME_SIZE (512U)

static volatile uint8_t g_rx_ring[VOFA_RX_RING_SIZE];
static volatile uint8_t g_rx_head;
static volatile uint8_t g_rx_tail;
static volatile bool g_rx_overflow;
static char g_command[VOFA_COMMAND_SIZE];
static uint8_t g_command_length;
static uint32_t g_last_command_byte_ms;
static bool g_discard_command;

static volatile uint8_t g_tx_ring[VOFA_TX_RING_SIZE];
static volatile uint16_t g_tx_head;
static volatile uint16_t g_tx_tail;
static volatile uint32_t g_tx_drop_count;
static char g_frame[VOFA_FRAME_SIZE];

static void VofaTelemetry_FillTxFifo(void)
{
    while ((g_tx_tail != g_tx_head) &&
           !DL_UART_Main_isTXFIFOFull(UART_VOFA_INST)) {
        DL_UART_Main_transmitData(UART_VOFA_INST,
                                  g_tx_ring[g_tx_tail]);
        g_tx_tail = (uint16_t)((g_tx_tail + 1U) & VOFA_TX_RING_MASK);
    }
    if (g_tx_tail == g_tx_head) {
        DL_UART_Main_disableInterrupt(UART_VOFA_INST,
                                      DL_UART_MAIN_INTERRUPT_TX);
    } else {
        DL_UART_Main_enableInterrupt(UART_VOFA_INST,
                                     DL_UART_MAIN_INTERRUPT_TX);
    }
}

static bool VofaTelemetry_Enqueue(const char *data, uint16_t length)
{
    uint16_t head;
    uint16_t free_bytes;
    uint32_t interrupt_state;

    if ((data == 0) || (length == 0U)) {
        return false;
    }
    head = g_tx_head;
    free_bytes = (uint16_t)((g_tx_tail - head - 1U) &
                            VOFA_TX_RING_MASK);
    if (length > free_bytes) {
        g_tx_drop_count++;
        return false;
    }
    for (uint16_t i = 0U; i < length; i++) {
        g_tx_ring[head] = (uint8_t)data[i];
        head = (uint16_t)((head + 1U) & VOFA_TX_RING_MASK);
    }
    __DMB();
    g_tx_head = head;

    interrupt_state = __get_PRIMASK();
    __disable_irq();
    VofaTelemetry_FillTxFifo();
    if (interrupt_state == 0U) {
        __enable_irq();
    }
    return true;
}

static void VofaTelemetry_SendText(const char *text)
{
    if (text != 0) {
        (void)VofaTelemetry_Enqueue(text, (uint16_t)strlen(text));
    }
}

static char *VofaTelemetry_AppendUnsigned(char *out, uint32_t value)
{
    char reversed[10];
    uint8_t count = 0U;

    do {
        reversed[count++] = (char)('0' + (value % 10U));
        value /= 10U;
    } while (value != 0U);
    while (count > 0U) {
        *out++ = reversed[--count];
    }
    return out;
}

static char *VofaTelemetry_AppendSigned(char *out, int32_t value)
{
    uint32_t magnitude;

    if (value < 0) {
        *out++ = '-';
        magnitude = (uint32_t)(-(value + 1)) + 1U;
    } else {
        magnitude = (uint32_t)value;
    }
    return VofaTelemetry_AppendUnsigned(out, magnitude);
}

static char *VofaTelemetry_AppendText(char *out, const char *text)
{
    while ((text != 0) && (*text != '\0')) {
        *out++ = *text++;
    }
    return out;
}

static char *VofaTelemetry_AppendU32Field(char *out,
                                         uint32_t value,
                                         bool *first)
{
    if (!*first) {
        *out++ = ',';
    }
    *first = false;
    return VofaTelemetry_AppendUnsigned(out, value);
}

static char *VofaTelemetry_AppendI32Field(char *out,
                                         int32_t value,
                                         bool *first)
{
    if (!*first) {
        *out++ = ',';
    }
    *first = false;
    return VofaTelemetry_AppendSigned(out, value);
}

static int32_t VofaTelemetry_RoundFloat(float value)
{
    return (int32_t)(value + ((value >= 0.0f) ? 0.5f : -0.5f));
}

static void VofaTelemetry_UppercaseAscii(char *text)
{
    while ((text != 0) && (*text != '\0')) {
        if ((*text >= 'a') && (*text <= 'z')) {
            *text = (char)(*text - ('a' - 'A'));
        }
        text++;
    }
}

static char *VofaTelemetry_SplitCommand(char *text)
{
    char *argument;

    while ((*text == ' ') || (*text == '\t')) {
        text++;
    }
    argument = text;
    while ((*argument != '\0') && (*argument != ' ') &&
           (*argument != '\t') && (*argument != '=') &&
           (*argument != ',')) {
        argument++;
    }
    if (*argument == '\0') {
        VofaTelemetry_UppercaseAscii(text);
        return 0;
    }
    *argument++ = '\0';
    while ((*argument == ' ') || (*argument == '\t') ||
           (*argument == '=') || (*argument == ',')) {
        argument++;
    }
    VofaTelemetry_UppercaseAscii(text);
    return (*argument == '\0') ? 0 : argument;
}

static bool VofaTelemetry_ParseU32(const char *text,
                                   uint32_t minimum,
                                   uint32_t maximum,
                                   uint32_t *value)
{
    char *end;
    unsigned long parsed;

    if ((text == 0) || (value == 0)) {
        return false;
    }
    parsed = strtoul(text, &end, 10);
    if (end == text) {
        return false;
    }
    while ((*end == ' ') || (*end == '\t')) {
        end++;
    }
    if ((*end != '\0') || (parsed < minimum) || (parsed > maximum)) {
        return false;
    }
    *value = (uint32_t)parsed;
    return true;
}

static bool VofaTelemetry_ParseFloat(const char *text,
                                     float minimum,
                                     float maximum,
                                     float *value)
{
    char *end;
    float parsed;

    if ((text == 0) || (value == 0)) {
        return false;
    }
    parsed = strtof(text, &end);
    if (end == text) {
        return false;
    }
    while ((*end == ' ') || (*end == '\t')) {
        end++;
    }
    if ((*end != '\0') || !(parsed >= minimum) ||
        !(parsed <= maximum)) {
        return false;
    }
    *value = parsed;
    return true;
}

static TB6612Drive *VofaTelemetry_GetDrive(CarFirmware *firmware)
{
    if ((firmware == 0) || (firmware->config.drive.context == 0)) {
        return 0;
    }
    return (TB6612Drive *)firmware->config.drive.context;
}

static const TB6612Drive *VofaTelemetry_GetDriveConst(
    const CarFirmware *firmware)
{
    if ((firmware == 0) || (firmware->config.drive.context == 0)) {
        return 0;
    }
    return (const TB6612Drive *)firmware->config.drive.context;
}

static bool VofaTelemetry_IsStopped(const CarFirmware *firmware)
{
    return (firmware != 0) && !firmware->app.executor.running &&
           !firmware->drive_active && !firmware->output.motor.enable;
}

static void VofaTelemetry_SendError(const char *reason)
{
    char line[96];
    char *out = line;

    out = VofaTelemetry_AppendText(out, "#ERR ");
    out = VofaTelemetry_AppendText(out, reason);
    *out++ = '\r';
    *out++ = '\n';
    (void)VofaTelemetry_Enqueue(line, (uint16_t)(out - line));
}

static void VofaTelemetry_SendOk(const char *name)
{
    char line[96];
    char *out = line;

    out = VofaTelemetry_AppendText(out, "#OK ");
    out = VofaTelemetry_AppendText(out, name);
    out = VofaTelemetry_AppendText(out, " RAM_ONLY\r\n");
    (void)VofaTelemetry_Enqueue(line, (uint16_t)(out - line));
}

static void VofaTelemetry_SendParams(const CarFirmware *firmware)
{
    TB6612SpeedLoopConfig speed = {0};
    char line[192];
    char *out = line;

    if (!TB6612_DriveGetSpeedLoopConfig(
            VofaTelemetry_GetDriveConst(firmware), &speed)) {
        VofaTelemetry_SendError("TB6612_NOT_READY");
        return;
    }
    out = VofaTelemetry_AppendText(out, "#PARAMS SPDKP_MILLI=");
    out = VofaTelemetry_AppendUnsigned(out, speed.kp_milli);
    out = VofaTelemetry_AppendText(out, " SPDKI_MILLI=");
    out = VofaTelemetry_AppendUnsigned(out, speed.ki_milli);
    out = VofaTelemetry_AppendText(out, " SPDKD_MILLI=");
    out = VofaTelemetry_AppendUnsigned(out, speed.kd_milli);
    out = VofaTelemetry_AppendText(out, " SPDLIMIT_PCT=");
    out = VofaTelemetry_AppendUnsigned(out, speed.output_limit_percent);
    out = VofaTelemetry_AppendText(out, " LINEKP_X1E6=");
    out = VofaTelemetry_AppendSigned(
        out, VofaTelemetry_RoundFloat(
                 firmware->app.config.line_kp * 1000000.0f));
    out = VofaTelemetry_AppendText(out, " SEARCHMM_X10=");
    out = VofaTelemetry_AppendSigned(
        out, VofaTelemetry_RoundFloat(
                 firmware->app.config.required_line_search_mm * 10.0f));
    *out++ = '\r';
    *out++ = '\n';
    (void)VofaTelemetry_Enqueue(line, (uint16_t)(out - line));
}

static void VofaTelemetry_ResetLineController(CarFirmware *firmware,
                                               uint32_t now_ms)
{
    firmware->app.executor.line_integral = 0.0f;
    firmware->app.executor.line_previous_error = 0.0f;
    firmware->app.executor.line_previous_ms = now_ms;
    firmware->app.executor.line_correction_mm_s = 0.0f;
}

static bool VofaTelemetry_UpdateSpeedParameter(CarFirmware *firmware,
                                               const char *command,
                                               const char *argument)
{
    TB6612Drive *drive = VofaTelemetry_GetDrive(firmware);
    TB6612SpeedLoopConfig speed;
    uint32_t value;

    if ((drive == 0) ||
        !TB6612_DriveGetSpeedLoopConfig(drive, &speed)) {
        VofaTelemetry_SendError("TB6612_NOT_READY");
        return true;
    }
    if (!VofaTelemetry_ParseU32(argument, 0U, 10000U, &value)) {
        VofaTelemetry_SendError("BAD_INTEGER");
        return true;
    }
    if (strcmp(command, "SPDKP") == 0) {
        speed.kp_milli = value;
    } else if (strcmp(command, "SPDKI") == 0) {
        speed.ki_milli = value;
    } else if (strcmp(command, "SPDKD") == 0) {
        speed.kd_milli = value;
    } else if (strcmp(command, "SPDLIMIT") == 0) {
        if ((value == 0U) || (value > TB6612_MAX_DUTY_PERCENT)) {
            VofaTelemetry_SendError("LIMIT_RANGE_1_TO_80");
            return true;
        }
        speed.output_limit_percent = (uint8_t)value;
    } else {
        return false;
    }
    if (!TB6612_DriveUpdateSpeedLoopTuning(
            drive, speed.kp_milli, speed.ki_milli, speed.kd_milli,
            speed.output_limit_percent)) {
        VofaTelemetry_SendError("TB6612_UPDATE_FAILED");
        return true;
    }
    VofaTelemetry_SendOk(command);
    return true;
}

static bool VofaTelemetry_ExecuteCommand(CarFirmware *firmware,
                                         uint32_t now_ms)
{
    char *command = g_command;
    char *argument;
    float value;

    while ((*command == ' ') || (*command == '\t')) {
        command++;
    }
    if (*command == '\0') {
        return false;
    }
    argument = VofaTelemetry_SplitCommand(command);

    if (strcmp(command, "STOP") == 0) {
        CarFirmware_ForceStop(firmware, CAR_FAULT_EMERGENCY_STOP);
        VofaTelemetry_SendOk("STOP_LATCHED");
        return true;
    }
    if (strcmp(command, "PARAMS") == 0) {
        VofaTelemetry_SendParams(firmware);
        return true;
    }
    if (strcmp(command, "HELP") == 0) {
        VofaTelemetry_SendText(
            "#HELP STOP PARAMS SPDKP SPDKI SPDKD SPDLIMIT LINEKP "
            "SEARCHMM HELP\r\n");
        return true;
    }
    if (!VofaTelemetry_IsStopped(firmware)) {
        VofaTelemetry_SendError("STOP_REQUIRED");
        return true;
    }
    if ((strcmp(command, "SPDKP") == 0) ||
        (strcmp(command, "SPDKI") == 0) ||
        (strcmp(command, "SPDKD") == 0) ||
        (strcmp(command, "SPDLIMIT") == 0)) {
        return VofaTelemetry_UpdateSpeedParameter(firmware, command,
                                                  argument);
    }
    if (strcmp(command, "LINEKP") == 0) {
        if (!VofaTelemetry_ParseFloat(argument, 0.0f, 0.5f, &value)) {
            VofaTelemetry_SendError("LINEKP_RANGE_0_TO_0.5");
            return true;
        }
        firmware->config.car.line_kp = value;
        firmware->app.config.line_kp = value;
        VofaTelemetry_ResetLineController(firmware, now_ms);
        VofaTelemetry_SendOk("LINEKP");
        return true;
    }
    if (strcmp(command, "SEARCHMM") == 0) {
        if (!VofaTelemetry_ParseFloat(argument, 10.0f, 500.0f, &value)) {
            VofaTelemetry_SendError("SEARCHMM_RANGE_10_TO_500");
            return true;
        }
        firmware->config.car.required_line_search_mm = value;
        firmware->app.config.required_line_search_mm = value;
        VofaTelemetry_SendOk("SEARCHMM_NEXT_ARM");
        return true;
    }

    VofaTelemetry_SendError("UNKNOWN_COMMAND");
    return true;
}

void VofaTelemetry_Init(void)
{
    g_rx_head = 0U;
    g_rx_tail = 0U;
    g_rx_overflow = false;
    g_command_length = 0U;
    g_last_command_byte_ms = 0U;
    g_discard_command = false;
    g_tx_head = 0U;
    g_tx_tail = 0U;
    g_tx_drop_count = 0U;
    DL_UART_Main_disableInterrupt(UART_VOFA_INST,
                                  DL_UART_MAIN_INTERRUPT_TX);
}

void VofaTelemetry_TxIrqHandler(void)
{
    VofaTelemetry_FillTxFifo();
}

void VofaTelemetry_PushRxFromIsr(uint8_t byte)
{
    uint8_t next = (uint8_t)((g_rx_head + 1U) & VOFA_RX_RING_MASK);

    if (next == g_rx_tail) {
        g_rx_overflow = true;
        return;
    }
    g_rx_ring[g_rx_head] = byte;
    __DMB();
    g_rx_head = next;
}

bool VofaTelemetry_ProcessCommands(CarFirmware *firmware,
                                   uint32_t uptime_ms)
{
    bool handled = false;

    if (firmware == 0) {
        return false;
    }
    if (g_rx_overflow) {
        g_rx_tail = g_rx_head;
        g_rx_overflow = false;
        g_command_length = 0U;
        g_discard_command = false;
        VofaTelemetry_SendError("RX_OVERFLOW");
        handled = true;
    }
    while (g_rx_tail != g_rx_head) {
        uint8_t byte = g_rx_ring[g_rx_tail];

        g_rx_tail = (uint8_t)((g_rx_tail + 1U) & VOFA_RX_RING_MASK);
        g_last_command_byte_ms = uptime_ms;
        if ((byte == '\r') || (byte == '\n')) {
            if (!g_discard_command && (g_command_length > 0U)) {
                g_command[g_command_length] = '\0';
                handled = VofaTelemetry_ExecuteCommand(firmware,
                                                       uptime_ms) || handled;
            }
            g_command_length = 0U;
            g_discard_command = false;
        } else if ((byte >= 0x20U) && (byte <= 0x7EU)) {
            if (g_command_length + 1U < VOFA_COMMAND_SIZE) {
                if (!g_discard_command) {
                    g_command[g_command_length++] = (char)byte;
                }
            } else {
                g_discard_command = true;
            }
        }
    }
    if (g_discard_command &&
        ((uint32_t)(uptime_ms - g_last_command_byte_ms) >=
         VOFA_COMMAND_IDLE_MS)) {
        g_command_length = 0U;
        g_discard_command = false;
        VofaTelemetry_SendError("COMMAND_TOO_LONG");
        handled = true;
    } else if (!g_discard_command && (g_command_length > 0U) &&
               ((uint32_t)(uptime_ms - g_last_command_byte_ms) >=
                VOFA_COMMAND_IDLE_MS)) {
        g_command[g_command_length] = '\0';
        handled = VofaTelemetry_ExecuteCommand(firmware, uptime_ms) ||
                  handled;
        g_command_length = 0U;
    }
    return handled;
}

void VofaTelemetry_SendBanner(void)
{
    VofaTelemetry_SendText(
        "#H2026_TB6612_FIREWATER_V1 fields=48 period_ms=100 "
        "baud=115200\r\n");
    VofaTelemetry_SendText(
        "#Commands are RAM-only; tuning writes require a stopped chassis. "
        "Send HELP for names.\r\n");
}

void VofaTelemetry_SendFrame(const CarFirmware *firmware,
                             uint32_t uptime_ms)
{
    TB6612SpeedLoopStatus speed = {0};
    char *out = g_frame;
    bool first = true;
    uint32_t faults;

    if (firmware == 0) {
        return;
    }
    TB6612_DriveGetSpeedLoopStatus(
        VofaTelemetry_GetDriveConst(firmware), &speed);
    faults = firmware->hardware_faults | firmware->output.faults;

    out = VofaTelemetry_AppendU32Field(out, uptime_ms, &first);                 /* I0 */
    out = VofaTelemetry_AppendU32Field(out, firmware->config.mode, &first);    /* I1 */
    out = VofaTelemetry_AppendU32Field(out,
        firmware->app.executor.running ? 1U : 0U, &first);                    /* I2 */
    out = VofaTelemetry_AppendU32Field(out,
        firmware->app.executor.finished ? 1U : 0U, &first);                   /* I3 */
    out = VofaTelemetry_AppendU32Field(out,
        firmware->app.executor.index, &first);                                /* I4 */
    out = VofaTelemetry_AppendU32Field(out,
        firmware->app.executor.track_phase, &first);                          /* I5 */
    out = VofaTelemetry_AppendI32Field(out, VofaTelemetry_RoundFloat(
        firmware->app.executor.segment_progress_mm * 10.0f), &first);         /* I6 */
    out = VofaTelemetry_AppendI32Field(out, VofaTelemetry_RoundFloat(
        firmware->app.odometry.center_distance_mm * 10.0f), &first);          /* I7 */
    out = VofaTelemetry_AppendU32Field(out,
        firmware->app.line.valid ? 1U : 0U, &first);                          /* I8 */
    out = VofaTelemetry_AppendI32Field(out,
        firmware->app.line.track_position, &first);                           /* I9 */
    out = VofaTelemetry_AppendU32Field(out,
        firmware->app.line.track_confidence, &first);                         /* I10 */
    out = VofaTelemetry_AppendU32Field(out,
        firmware->app.line.track_active_count, &first);                       /* I11 */
    out = VofaTelemetry_AppendU32Field(out,
        firmware->app.line.track_active_mask, &first);                        /* I12 */
    out = VofaTelemetry_AppendU32Field(out,
        firmware->app.line.pattern, &first);                                  /* I13 */
    out = VofaTelemetry_AppendI32Field(out, VofaTelemetry_RoundFloat(
        firmware->app.executor.line_correction_mm_s * 10.0f), &first);        /* I14 */
    out = VofaTelemetry_AppendI32Field(out, VofaTelemetry_RoundFloat(
        firmware->output.motor.left_mm_s * 10.0f), &first);                   /* I15 */
    out = VofaTelemetry_AppendI32Field(out, VofaTelemetry_RoundFloat(
        firmware->output.motor.right_mm_s * 10.0f), &first);                  /* I16 */
    out = VofaTelemetry_AppendI32Field(out, speed.target_left_rpm, &first);    /* I17 */
    out = VofaTelemetry_AppendI32Field(out, speed.target_right_rpm, &first);   /* I18 */
    out = VofaTelemetry_AppendI32Field(out, speed.measured_left_rpm, &first);  /* I19 */
    out = VofaTelemetry_AppendI32Field(out, speed.measured_right_rpm, &first); /* I20 */
    out = VofaTelemetry_AppendI32Field(out, speed.left_output_percent, &first);/* I21 */
    out = VofaTelemetry_AppendI32Field(out, speed.right_output_percent, &first);/* I22 */
    out = VofaTelemetry_AppendI32Field(out, speed.left_delta_count, &first);   /* I23 */
    out = VofaTelemetry_AppendI32Field(out, speed.right_delta_count, &first);  /* I24 */
    out = VofaTelemetry_AppendU32Field(out, speed.sample_elapsed_ms, &first);  /* I25 */
    out = VofaTelemetry_AppendU32Field(out, speed.update_count, &first);       /* I26 */
    out = VofaTelemetry_AppendI32Field(out, VofaTelemetry_RoundFloat(
        firmware->imu_sample.yaw_deg * 10.0f), &first);                       /* I27 */
    out = VofaTelemetry_AppendU32Field(out, faults, &first);                   /* I28 */
    out = VofaTelemetry_AppendU32Field(out,
        firmware->app.executor.last_exit_reason, &first);                     /* I29 */
    out = VofaTelemetry_AppendI32Field(out, VofaTelemetry_RoundFloat(
        firmware->app.executor.track_line_gap_mm * 10.0f), &first);           /* I30 */
    out = VofaTelemetry_AppendU32Field(out,
        firmware->app.executor.marker_streak, &first);                        /* I31 */
    out = VofaTelemetry_AppendU32Field(out,
        firmware->app.executor.last_marker_confidence, &first);               /* I32 */
    out = VofaTelemetry_AppendU32Field(out,
        firmware->app.executor.last_marker_active_count, &first);             /* I33 */
    out = VofaTelemetry_AppendI32Field(out, VofaTelemetry_RoundFloat(
        firmware->app.executor.finish_offset_progress_mm * 10.0f), &first);   /* I34 */
    out = VofaTelemetry_AppendU32Field(out,
        firmware->output.result_time_ms, &first);                             /* I35 */
    out = VofaTelemetry_AppendU32Field(out,
        firmware->output.result_valid ? 1U : 0U, &first);                     /* I36 */
    out = VofaTelemetry_AppendU32Field(out,
        firmware->encoder_valid_current ? 1U : 0U, &first);                   /* I37 */
    out = VofaTelemetry_AppendU32Field(out,
        firmware->drive_active ? 1U : 0U, &first);                            /* I38 */
    for (uint8_t i = 0U; i < GRAY_ARRAY_CHANNELS; i++) {
        out = VofaTelemetry_AppendU32Field(
            out, firmware->gray_sample.normalized[i], &first);                /* I39-I46 */
    }
    out = VofaTelemetry_AppendU32Field(out, g_tx_drop_count, &first);          /* I47 */
    *out++ = '\r';
    *out++ = '\n';
    (void)VofaTelemetry_Enqueue(g_frame, (uint16_t)(out - g_frame));
}
