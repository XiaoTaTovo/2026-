#include "vofa_telemetry.h"

#include <stddef.h>
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

typedef struct {
    char *begin;
    char *out;
    char *end;
    bool overflow;
} VofaTelemetryWriter;

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

static VofaTelemetryWriter VofaTelemetry_BeginWrite(char *buffer,
                                                    uint16_t capacity)
{
    VofaTelemetryWriter writer = {
        .begin = buffer,
        .out = buffer,
        .end = buffer + capacity,
        .overflow = false
    };

    return writer;
}

static void VofaTelemetry_AppendChar(VofaTelemetryWriter *writer, char value)
{
    if (writer->out < writer->end) {
        *writer->out++ = value;
    } else {
        writer->overflow = true;
    }
}

static void VofaTelemetry_AppendUnsigned(VofaTelemetryWriter *writer,
                                         uint32_t value)
{
    char reversed[10];
    uint8_t count = 0U;

    do {
        reversed[count++] = (char)('0' + (value % 10U));
        value /= 10U;
    } while (value != 0U);
    while (count > 0U) {
        VofaTelemetry_AppendChar(writer, reversed[--count]);
    }
}

static void VofaTelemetry_AppendSigned(VofaTelemetryWriter *writer,
                                       int32_t value)
{
    uint32_t magnitude;

    if (value < 0) {
        VofaTelemetry_AppendChar(writer, '-');
        magnitude = (uint32_t)(-(value + 1)) + 1U;
    } else {
        magnitude = (uint32_t)value;
    }
    VofaTelemetry_AppendUnsigned(writer, magnitude);
}

static void VofaTelemetry_AppendText(VofaTelemetryWriter *writer,
                                     const char *text)
{
    while ((text != 0) && (*text != '\0')) {
        VofaTelemetry_AppendChar(writer, *text++);
    }
}

static void VofaTelemetry_AppendU32Field(VofaTelemetryWriter *writer,
                                         uint32_t value,
                                         bool *first)
{
    if (!*first) {
        VofaTelemetry_AppendChar(writer, ',');
    }
    *first = false;
    VofaTelemetry_AppendUnsigned(writer, value);
}

static void VofaTelemetry_AppendI32Field(VofaTelemetryWriter *writer,
                                         int32_t value,
                                         bool *first)
{
    if (!*first) {
        VofaTelemetry_AppendChar(writer, ',');
    }
    *first = false;
    VofaTelemetry_AppendSigned(writer, value);
}

static bool VofaTelemetry_EnqueueWriter(
    const VofaTelemetryWriter *writer)
{
    if (writer->overflow) {
        g_tx_drop_count++;
        return false;
    }
    return VofaTelemetry_Enqueue(
        writer->begin, (uint16_t)(writer->out - writer->begin));
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

typedef struct {
    const char *name;
    size_t car_config_offset;
    float minimum;
    float maximum;
    bool reset_line_controller;
} VofaLineParameter;

static const VofaLineParameter g_line_parameters[] = {
    {"LINEKP", offsetof(CarConfig, line_kp), 0.0f, 0.5f, true},
    {"LINEKD", offsetof(CarConfig, line_kd), 0.0f, 0.05f, true},
    {"LINETAU", offsetof(CarConfig, line_derivative_filter_tau_s),
     0.0f, 0.3f, true},
    {"ARCTRKMM", offsetof(CarConfig, arc_effective_track_width_mm),
     80.0f, 220.0f, false},
    {"ARCLIM", offsetof(CarConfig, arc_line_max_correction_mm_s),
     0.0f, 150.0f, true},
    {"ARCSPD", offsetof(CarConfig, arc_speed_mm_s), 80.0f, 350.0f, false},
    {"STRSPD", offsetof(CarConfig, straight_speed_mm_s), 80.0f, 350.0f,
     false},
    {"ACCEL", offsetof(CarConfig, track_wheel_accel_limit_mm_s2),
     100.0f, 3000.0f, false},
    {"SEARCHMM", offsetof(CarConfig, required_line_search_mm),
     10.0f, 500.0f, false}
};

static void VofaTelemetry_SendError(const char *reason)
{
    char line[96];
    VofaTelemetryWriter writer = VofaTelemetry_BeginWrite(
        line, (uint16_t)sizeof(line));

    VofaTelemetry_AppendText(&writer, "#ERR ");
    VofaTelemetry_AppendText(&writer, reason);
    VofaTelemetry_AppendChar(&writer, '\r');
    VofaTelemetry_AppendChar(&writer, '\n');
    (void)VofaTelemetry_EnqueueWriter(&writer);
}

static void VofaTelemetry_SendOk(const char *name)
{
    char line[96];
    VofaTelemetryWriter writer = VofaTelemetry_BeginWrite(
        line, (uint16_t)sizeof(line));

    VofaTelemetry_AppendText(&writer, "#OK ");
    VofaTelemetry_AppendText(&writer, name);
    VofaTelemetry_AppendText(&writer, " RAM_ONLY\r\n");
    (void)VofaTelemetry_EnqueueWriter(&writer);
}

static void VofaTelemetry_SendParams(const CarFirmware *firmware)
{
    TB6612SpeedLoopConfig speed = {0};
    char line[384];
    VofaTelemetryWriter writer = VofaTelemetry_BeginWrite(
        line, (uint16_t)sizeof(line));

    if (!TB6612_DriveGetSpeedLoopConfig(
            VofaTelemetry_GetDriveConst(firmware), &speed)) {
        VofaTelemetry_SendError("TB6612_NOT_READY");
        return;
    }
    VofaTelemetry_AppendText(&writer, "#PARAMS SPDKP_MILLI=");
    VofaTelemetry_AppendUnsigned(&writer, speed.kp_milli);
    VofaTelemetry_AppendText(&writer, " SPDKI_MILLI=");
    VofaTelemetry_AppendUnsigned(&writer, speed.ki_milli);
    VofaTelemetry_AppendText(&writer, " SPDKD_MILLI=");
    VofaTelemetry_AppendUnsigned(&writer, speed.kd_milli);
    VofaTelemetry_AppendText(&writer, " SPDLIMIT_PCT=");
    VofaTelemetry_AppendUnsigned(&writer, speed.output_limit_percent);
    VofaTelemetry_AppendText(&writer, " LINEKP_X1E6=");
    VofaTelemetry_AppendSigned(
        &writer, VofaTelemetry_RoundFloat(
                     firmware->app.config.line_kp * 1000000.0f));
    VofaTelemetry_AppendText(&writer, " LINEKD_X1E6=");
    VofaTelemetry_AppendSigned(
        &writer, VofaTelemetry_RoundFloat(
                     firmware->app.config.line_kd * 1000000.0f));
    VofaTelemetry_AppendText(&writer, " LINETAU_MS=");
    VofaTelemetry_AppendSigned(
        &writer, VofaTelemetry_RoundFloat(
                     firmware->app.config.line_derivative_filter_tau_s *
                     1000.0f));
    VofaTelemetry_AppendText(&writer, " ARCTRKMM_X10=");
    VofaTelemetry_AppendSigned(
        &writer, VofaTelemetry_RoundFloat(
                     firmware->app.config.arc_effective_track_width_mm *
                     10.0f));
    VofaTelemetry_AppendText(&writer, " ARCLIM_X10=");
    VofaTelemetry_AppendSigned(
        &writer, VofaTelemetry_RoundFloat(
                     firmware->app.config.arc_line_max_correction_mm_s *
                     10.0f));
    VofaTelemetry_AppendText(&writer, " ARCSPD_X10=");
    VofaTelemetry_AppendSigned(
        &writer, VofaTelemetry_RoundFloat(
                     firmware->app.config.arc_speed_mm_s * 10.0f));
    VofaTelemetry_AppendText(&writer, " STRSPD_X10=");
    VofaTelemetry_AppendSigned(
        &writer, VofaTelemetry_RoundFloat(
                     firmware->app.config.straight_speed_mm_s * 10.0f));
    VofaTelemetry_AppendText(&writer, " ACCEL_X10=");
    VofaTelemetry_AppendSigned(
        &writer, VofaTelemetry_RoundFloat(
                     firmware->app.config.track_wheel_accel_limit_mm_s2 *
                     10.0f));
    VofaTelemetry_AppendText(&writer, " SEARCHMM_X10=");
    VofaTelemetry_AppendSigned(
        &writer, VofaTelemetry_RoundFloat(
                     firmware->app.config.required_line_search_mm * 10.0f));
    VofaTelemetry_AppendChar(&writer, '\r');
    VofaTelemetry_AppendChar(&writer, '\n');
    (void)VofaTelemetry_EnqueueWriter(&writer);
}

static void VofaTelemetry_ResetLineController(CarFirmware *firmware,
                                               uint32_t now_ms)
{
    firmware->app.executor.line_integral = 0.0f;
    firmware->app.executor.line_previous_error = 0.0f;
    firmware->app.executor.line_derivative_filtered = 0.0f;
    firmware->app.executor.line_previous_ms = now_ms;
    firmware->app.executor.line_derivative_initialized = false;
    firmware->app.executor.line_correction_mm_s = 0.0f;
}

static bool VofaTelemetry_UpdateLineParameter(CarFirmware *firmware,
                                              const char *command,
                                              const char *argument,
                                              uint32_t now_ms)
{
    const VofaLineParameter *parameter = 0;
    float value;
    size_t index;

    for (index = 0U;
         index < (sizeof(g_line_parameters) / sizeof(g_line_parameters[0]));
         index++) {
        if (strcmp(command, g_line_parameters[index].name) == 0) {
            parameter = &g_line_parameters[index];
            break;
        }
    }
    if (parameter == 0) {
        return false;
    }
    if (!VofaTelemetry_ParseFloat(argument, parameter->minimum,
                                  parameter->maximum, &value)) {
        VofaTelemetry_SendError("PARAMETER_RANGE");
        return true;
    }
    *(float *)((uint8_t *)&firmware->config.car +
               parameter->car_config_offset) = value;
    *(float *)((uint8_t *)&firmware->app.config +
               parameter->car_config_offset) = value;
    if (parameter->reset_line_controller) {
        VofaTelemetry_ResetLineController(firmware, now_ms);
    }
    VofaTelemetry_SendOk(command);
    return true;
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
            "#HELP STOP PARAMS SPDKP SPDKI SPDKD SPDLIMIT LINEKP LINEKD "
            "LINETAU ARCTRKMM ARCLIM ARCSPD STRSPD ACCEL SEARCHMM HELP\r\n");
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
    if (VofaTelemetry_UpdateLineParameter(firmware, command, argument,
                                          now_ms)) {
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

void VofaTelemetry_SendSpeedLoopBanner(void)
{
    VofaTelemetry_SendText(
        "#H2026_SPEED_TUNING_FIREWATER_V1 fields=9 period_ms=25 "
        "baud=115200\r\n");
    VofaTelemetry_SendText(
        "#FIELDS t_ms,target_l_rpm,measured_l_rpm,output_l_pct,"
        "target_r_rpm,measured_r_rpm,output_r_pct,faults,tx_drop\r\n");
    VofaTelemetry_SendText(
        "#KEY1 ARM_OR_STOP 5S; STOP LATCHES; PARAMS READS; "
        "SPDKP/SPDKI/SPDKD/SPDLIMIT REQUIRE_STOP\r\n");
}

void VofaTelemetry_SendSpeedLoopArm(uint32_t trial_id,
                                    int16_t target_mm_s,
                                    uint32_t uptime_ms)
{
    char line[96];
    VofaTelemetryWriter writer = VofaTelemetry_BeginWrite(
        line, (uint16_t)sizeof(line));

    VofaTelemetry_AppendText(&writer, "#EVT ARM ");
    VofaTelemetry_AppendUnsigned(&writer, trial_id);
    VofaTelemetry_AppendText(&writer, " SPEED_LOOP ");
    VofaTelemetry_AppendSigned(&writer, target_mm_s);
    VofaTelemetry_AppendChar(&writer, ' ');
    VofaTelemetry_AppendUnsigned(&writer, uptime_ms);
    VofaTelemetry_AppendChar(&writer, '\r');
    VofaTelemetry_AppendChar(&writer, '\n');
    (void)VofaTelemetry_EnqueueWriter(&writer);
}

void VofaTelemetry_SendSpeedLoopDone(uint32_t trial_id,
                                     const char *reason,
                                     uint32_t duration_ms,
                                     uint32_t faults)
{
    char line[112];
    VofaTelemetryWriter writer = VofaTelemetry_BeginWrite(
        line, (uint16_t)sizeof(line));

    VofaTelemetry_AppendText(&writer, "#EVT DONE ");
    VofaTelemetry_AppendUnsigned(&writer, trial_id);
    VofaTelemetry_AppendChar(&writer, ' ');
    VofaTelemetry_AppendText(&writer, reason);
    VofaTelemetry_AppendChar(&writer, ' ');
    VofaTelemetry_AppendUnsigned(&writer, duration_ms);
    VofaTelemetry_AppendChar(&writer, ' ');
    VofaTelemetry_AppendUnsigned(&writer, faults);
    VofaTelemetry_AppendChar(&writer, '\r');
    VofaTelemetry_AppendChar(&writer, '\n');
    (void)VofaTelemetry_EnqueueWriter(&writer);
}

void VofaTelemetry_SendLineTrialArm(uint32_t trial_id,
                                    uint32_t uptime_ms)
{
    char line[96];
    VofaTelemetryWriter writer = VofaTelemetry_BeginWrite(
        line, (uint16_t)sizeof(line));

    VofaTelemetry_AppendText(&writer, "#EVT ARM ");
    VofaTelemetry_AppendUnsigned(&writer, trial_id);
    VofaTelemetry_AppendText(&writer, " LINE B2 ");
    VofaTelemetry_AppendUnsigned(&writer, uptime_ms);
    VofaTelemetry_AppendText(&writer, "\r\n");
    (void)VofaTelemetry_EnqueueWriter(&writer);
}

void VofaTelemetry_SendLineTrialDone(uint32_t trial_id,
                                     uint32_t duration_ms,
                                     uint32_t faults,
                                     uint32_t exit_reason)
{
    char line[112];
    VofaTelemetryWriter writer = VofaTelemetry_BeginWrite(
        line, (uint16_t)sizeof(line));

    VofaTelemetry_AppendText(&writer, "#EVT DONE ");
    VofaTelemetry_AppendUnsigned(&writer, trial_id);
    VofaTelemetry_AppendText(&writer, " LINE ");
    VofaTelemetry_AppendUnsigned(&writer, duration_ms);
    VofaTelemetry_AppendChar(&writer, ' ');
    VofaTelemetry_AppendUnsigned(&writer, exit_reason);
    VofaTelemetry_AppendChar(&writer, ' ');
    VofaTelemetry_AppendUnsigned(&writer, faults);
    VofaTelemetry_AppendText(&writer, "\r\n");
    (void)VofaTelemetry_EnqueueWriter(&writer);
}

void VofaTelemetry_SendLineTrialReject(uint32_t action,
                                       int32_t status,
                                       uint32_t uptime_ms)
{
    char line[96];
    VofaTelemetryWriter writer = VofaTelemetry_BeginWrite(
        line, (uint16_t)sizeof(line));

    VofaTelemetry_AppendText(&writer, "#EVT REJECT ");
    VofaTelemetry_AppendUnsigned(&writer, action);
    VofaTelemetry_AppendChar(&writer, ' ');
    VofaTelemetry_AppendSigned(&writer, status);
    VofaTelemetry_AppendChar(&writer, ' ');
    VofaTelemetry_AppendUnsigned(&writer, uptime_ms);
    VofaTelemetry_AppendText(&writer, "\r\n");
    (void)VofaTelemetry_EnqueueWriter(&writer);
}

void VofaTelemetry_SendSpeedLoopFrame(const CarFirmware *firmware,
                                      uint32_t uptime_ms)
{
    TB6612SpeedLoopStatus speed = {0};
    VofaTelemetryWriter writer = VofaTelemetry_BeginWrite(
        g_frame, (uint16_t)sizeof(g_frame));
    bool first = true;
    uint32_t faults;

    if (firmware == 0) {
        return;
    }
    TB6612_DriveGetSpeedLoopStatus(
        VofaTelemetry_GetDriveConst(firmware), &speed);
    faults = firmware->hardware_faults | firmware->output.faults;
    VofaTelemetry_AppendU32Field(&writer, uptime_ms, &first);
    VofaTelemetry_AppendI32Field(&writer, speed.target_left_rpm, &first);
    VofaTelemetry_AppendI32Field(&writer, speed.measured_left_rpm, &first);
    VofaTelemetry_AppendI32Field(&writer, speed.left_output_percent, &first);
    VofaTelemetry_AppendI32Field(&writer, speed.target_right_rpm, &first);
    VofaTelemetry_AppendI32Field(&writer, speed.measured_right_rpm, &first);
    VofaTelemetry_AppendI32Field(&writer, speed.right_output_percent, &first);
    VofaTelemetry_AppendU32Field(&writer, faults, &first);
    VofaTelemetry_AppendU32Field(&writer, g_tx_drop_count, &first);
    VofaTelemetry_AppendChar(&writer, '\r');
    VofaTelemetry_AppendChar(&writer, '\n');
    (void)VofaTelemetry_EnqueueWriter(&writer);
}

void VofaTelemetry_SendFrame(const CarFirmware *firmware,
                             uint32_t uptime_ms)
{
    TB6612SpeedLoopStatus speed = {0};
    VofaTelemetryWriter writer = VofaTelemetry_BeginWrite(
        g_frame, (uint16_t)sizeof(g_frame));
    bool first = true;
    uint32_t faults;

    if (firmware == 0) {
        return;
    }
    TB6612_DriveGetSpeedLoopStatus(
        VofaTelemetry_GetDriveConst(firmware), &speed);
    faults = firmware->hardware_faults | firmware->output.faults;

    VofaTelemetry_AppendU32Field(&writer, uptime_ms, &first);                  /* I0 */
    VofaTelemetry_AppendU32Field(&writer, firmware->config.mode, &first);     /* I1 */
    VofaTelemetry_AppendU32Field(&writer,
        firmware->app.executor.running ? 1U : 0U, &first);                    /* I2 */
    VofaTelemetry_AppendU32Field(&writer,
        firmware->app.executor.finished ? 1U : 0U, &first);                   /* I3 */
    VofaTelemetry_AppendU32Field(&writer,
        firmware->app.executor.index, &first);                                /* I4 */
    VofaTelemetry_AppendU32Field(&writer,
        firmware->app.executor.track_phase, &first);                          /* I5 */
    VofaTelemetry_AppendI32Field(&writer, VofaTelemetry_RoundFloat(
        firmware->app.executor.segment_progress_mm * 10.0f), &first);         /* I6 */
    VofaTelemetry_AppendI32Field(&writer, VofaTelemetry_RoundFloat(
        firmware->app.odometry.center_distance_mm * 10.0f), &first);          /* I7 */
    VofaTelemetry_AppendU32Field(&writer,
        firmware->app.line.valid ? 1U : 0U, &first);                          /* I8 */
    VofaTelemetry_AppendI32Field(&writer,
        firmware->app.line.track_position, &first);                           /* I9 */
    VofaTelemetry_AppendU32Field(&writer,
        firmware->app.line.track_confidence, &first);                         /* I10 */
    VofaTelemetry_AppendU32Field(&writer,
        firmware->app.line.track_active_count, &first);                       /* I11 */
    VofaTelemetry_AppendU32Field(&writer,
        firmware->app.line.track_active_mask, &first);                        /* I12 */
    VofaTelemetry_AppendU32Field(&writer,
        firmware->app.line.pattern, &first);                                  /* I13 */
    VofaTelemetry_AppendI32Field(&writer, VofaTelemetry_RoundFloat(
        firmware->app.executor.line_correction_mm_s * 10.0f), &first);        /* I14 */
    VofaTelemetry_AppendI32Field(&writer, VofaTelemetry_RoundFloat(
        firmware->output.motor.left_mm_s * 10.0f), &first);                   /* I15 */
    VofaTelemetry_AppendI32Field(&writer, VofaTelemetry_RoundFloat(
        firmware->output.motor.right_mm_s * 10.0f), &first);                  /* I16 */
    VofaTelemetry_AppendI32Field(&writer, speed.target_left_rpm, &first);      /* I17 */
    VofaTelemetry_AppendI32Field(&writer, speed.target_right_rpm, &first);     /* I18 */
    VofaTelemetry_AppendI32Field(&writer, speed.measured_left_rpm, &first);    /* I19 */
    VofaTelemetry_AppendI32Field(&writer, speed.measured_right_rpm, &first);   /* I20 */
    VofaTelemetry_AppendI32Field(&writer, speed.left_output_percent, &first);  /* I21 */
    VofaTelemetry_AppendI32Field(&writer, speed.right_output_percent, &first); /* I22 */
    VofaTelemetry_AppendI32Field(&writer, speed.left_delta_count, &first);     /* I23 */
    VofaTelemetry_AppendI32Field(&writer, speed.right_delta_count, &first);    /* I24 */
    VofaTelemetry_AppendU32Field(&writer, speed.sample_elapsed_ms, &first);    /* I25 */
    VofaTelemetry_AppendU32Field(&writer, speed.update_count, &first);         /* I26 */
    VofaTelemetry_AppendI32Field(&writer, VofaTelemetry_RoundFloat(
        firmware->imu_sample.yaw_deg * 10.0f), &first);                       /* I27 */
    VofaTelemetry_AppendU32Field(&writer, faults, &first);                     /* I28 */
    VofaTelemetry_AppendU32Field(&writer,
        firmware->app.executor.last_exit_reason, &first);                     /* I29 */
    VofaTelemetry_AppendI32Field(&writer, VofaTelemetry_RoundFloat(
        firmware->app.executor.track_line_gap_mm * 10.0f), &first);           /* I30 */
    VofaTelemetry_AppendU32Field(&writer,
        firmware->app.executor.marker_streak, &first);                        /* I31 */
    VofaTelemetry_AppendU32Field(&writer,
        firmware->app.executor.last_marker_confidence, &first);               /* I32 */
    VofaTelemetry_AppendU32Field(&writer,
        firmware->app.executor.last_marker_active_count, &first);             /* I33 */
    VofaTelemetry_AppendI32Field(&writer, VofaTelemetry_RoundFloat(
        firmware->app.executor.finish_offset_progress_mm * 10.0f), &first);   /* I34 */
    VofaTelemetry_AppendU32Field(&writer,
        firmware->output.result_time_ms, &first);                             /* I35 */
    VofaTelemetry_AppendU32Field(&writer,
        firmware->output.result_valid ? 1U : 0U, &first);                     /* I36 */
    VofaTelemetry_AppendU32Field(&writer,
        firmware->encoder_valid_current ? 1U : 0U, &first);                   /* I37 */
    VofaTelemetry_AppendU32Field(&writer,
        firmware->drive_active ? 1U : 0U, &first);                            /* I38 */
    for (uint8_t i = 0U; i < GRAY_ARRAY_CHANNELS; i++) {
        VofaTelemetry_AppendU32Field(
            &writer, firmware->gray_sample.normalized[i], &first);            /* I39-I46 */
    }
    VofaTelemetry_AppendU32Field(&writer, g_tx_drop_count, &first);            /* I47 */
    VofaTelemetry_AppendChar(&writer, '\r');
    VofaTelemetry_AppendChar(&writer, '\n');
    (void)VofaTelemetry_EnqueueWriter(&writer);
}
