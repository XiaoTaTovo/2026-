#include "vofa_telemetry.h"

#include "ti_msp_dl_config.h"
#include "tb6612.h"

#include <stdlib.h>
#include <string.h>

#define ROUTE_COMMAND_RX_SIZE       (64U)
#define ROUTE_COMMAND_BUFFER_SIZE   (64U)
#define ROUTE_COMMAND_IDLE_MS       (100U)
#define VOFA_TX_BUFFER_SIZE         (1024U)
#define VOFA_TX_BUFFER_MASK         (VOFA_TX_BUFFER_SIZE - 1U)

static volatile uint8_t gRouteRxBuffer[ROUTE_COMMAND_RX_SIZE];
static volatile uint8_t gRouteRxHead;
static volatile uint8_t gRouteRxTail;
static volatile bool gRouteRxOverflow;
static char gRouteCommandBuffer[ROUTE_COMMAND_BUFFER_SIZE];
static uint8_t gRouteCommandLength;
static uint32_t gRouteLastCommandByteMs;
static bool gRouteDiscardCommand;
static volatile uint8_t gVofaTxBuffer[VOFA_TX_BUFFER_SIZE];
static volatile uint16_t gVofaTxHead;
static volatile uint16_t gVofaTxTail;
static volatile uint32_t gVofaTxDropCount;

static void vofa_tx_fill_fifo(void)
{
    while ((gVofaTxTail != gVofaTxHead) &&
           !DL_UART_Main_isTXFIFOFull(UART_BLUETOOTH_INST)) {
        DL_UART_Main_transmitData(
            UART_BLUETOOTH_INST, gVofaTxBuffer[gVofaTxTail]);
        gVofaTxTail = (uint16_t)((gVofaTxTail + 1U) & VOFA_TX_BUFFER_MASK);
    }
    if (gVofaTxTail == gVofaTxHead) {
        DL_UART_Main_disableInterrupt(
            UART_BLUETOOTH_INST, DL_UART_MAIN_INTERRUPT_TX);
    } else {
        DL_UART_Main_enableInterrupt(
            UART_BLUETOOTH_INST, DL_UART_MAIN_INTERRUPT_TX);
    }
}

static bool vofa_tx_enqueue(const char *data, uint16_t length)
{
    uint16_t head;
    uint16_t free_bytes;
    uint32_t interrupt_state;

    if ((data == 0) || (length == 0U)) {
        return false;
    }
    head = gVofaTxHead;
    free_bytes = (uint16_t)((gVofaTxTail - head - 1U) & VOFA_TX_BUFFER_MASK);
    if (length > free_bytes) {
        gVofaTxDropCount++;
        return false;
    }
    for (uint16_t i = 0U; i < length; i++) {
        gVofaTxBuffer[head] = (uint8_t)data[i];
        head = (uint16_t)((head + 1U) & VOFA_TX_BUFFER_MASK);
    }
    __DMB();
    gVofaTxHead = head;

    /* Prime only the hardware FIFO. The remaining bytes leave from the TX
     * interrupt, so formatting telemetry never waits for the wire. */
    interrupt_state = __get_PRIMASK();
    __disable_irq();
    vofa_tx_fill_fifo();
    if (interrupt_state == 0U) {
        __enable_irq();
    }
    return true;
}

static void send_text(const char *text)
{
    (void)vofa_tx_enqueue(text, (uint16_t)strlen(text));
}

static char *append_unsigned(char *out, uint32_t value)
{
    char reversed[10];
    uint32_t count = 0;

    do {
        reversed[count++] = (char) ('0' + (value % 10U));
        value /= 10U;
    } while (value != 0U);

    while (count != 0U) {
        *out++ = reversed[--count];
    }
    return out;
}

static char *append_signed(char *out, int32_t value)
{
    uint32_t magnitude;

    if (value < 0) {
        *out++ = '-';
        magnitude = (uint32_t) (-(value + 1)) + 1U;
    } else {
        magnitude = (uint32_t) value;
    }
    return append_unsigned(out, magnitude);
}

static char *append_separator(char *out)
{
    *out++ = ',';
    return out;
}

static char *append_u32_field(char *out, uint32_t value)
{
    out = append_unsigned(out, value);
    return append_separator(out);
}

static char *append_i32_field(char *out, int32_t value)
{
    out = append_signed(out, value);
    return append_separator(out);
}

static int32_t round_float(float value)
{
    return (int32_t)(value + ((value >= 0.0f) ? 0.5f : -0.5f));
}

static char *route_append_text(char *out, const char *text)
{
    while (*text != '\0') {
        *out++ = *text++;
    }
    return out;
}

static void route_uppercase_ascii(char *text)
{
    while (*text != '\0') {
        if ((*text >= 'a') && (*text <= 'z')) {
            *text = (char)(*text - ('a' - 'A'));
        }
        text++;
    }
}

static char *route_split_command(char *text)
{
    char *argument = text;

    while ((*argument != '\0') && (*argument != ' ') &&
           (*argument != '\t') && (*argument != ',')) {
        argument++;
    }
    if (*argument == '\0') {
        return 0;
    }
    *argument++ = '\0';
    while ((*argument == ' ') || (*argument == '\t') ||
           (*argument == ',')) {
        argument++;
    }
    return (*argument == '\0') ? 0 : argument;
}

static bool route_parse_float(const char *text, float minimum,
                              float maximum, float *value)
{
    char *end;
    float parsed;

    if ((text == 0) || (value == 0)) {
        return false;
    }
    parsed = strtof(text, &end);
    while ((*end == ' ') || (*end == '\t')) {
        end++;
    }
    if ((end == text) || (*end != '\0') ||
        !(parsed >= minimum && parsed <= maximum)) {
        return false;
    }
    *value = parsed;
    return true;
}

static bool route_parse_u32(const char *text, uint32_t minimum,
                            uint32_t maximum, uint32_t *value)
{
    char *end;
    unsigned long parsed;

    if ((text == 0) || (value == 0) || (*text < '0') || (*text > '9')) {
        return false;
    }
    parsed = strtoul(text, &end, 10);
    while ((*end == ' ') || (*end == '\t')) {
        end++;
    }
    if ((*end != '\0') || (parsed < minimum) || (parsed > maximum)) {
        return false;
    }
    *value = (uint32_t)parsed;
    return true;
}

static void route_send_param(const char *name, int32_t value)
{
    char line[64];
    char *out = line;

    out = route_append_text(out, "#PARAM " );
    out = route_append_text(out, name);
    *out++ = '=';
    out = append_signed(out, value);
    out = route_append_text(out, " RAM");
    *out++ = '\r';
    *out++ = '\n';
    (void)vofa_tx_enqueue(line, (uint16_t)(out - line));
}

static void route_send_ok_ram(const char *name)
{
    char line[64];
    char *out = line;

    out = route_append_text(out, "#OK ");
    out = route_append_text(out, name);
    out = route_append_text(out, " RAM\r\n");
    (void)vofa_tx_enqueue(line, (uint16_t)(out - line));
}

static void route_send_error(const char *reason)
{
    char line[64];
    char *out = line;

    out = route_append_text(out, "#ERR ");
    out = route_append_text(out, reason);
    out = route_append_text(out, "\r\n");
    (void)vofa_tx_enqueue(line, (uint16_t)(out - line));
}

static const TB6612MotorBoardContext *route_get_speed_context_const(
    const CarFirmware *firmware)
{
    if ((firmware == 0) ||
        (firmware->config.motor.direct_set_wheel_speeds !=
         TB6612_MotorBoard_SetWheelSpeeds)) {
        return 0;
    }
    return (const TB6612MotorBoardContext *)
        firmware->config.motor.direct_context;
}

static TB6612MotorBoardContext *route_get_speed_context(
    CarFirmware *firmware)
{
    if ((firmware == 0) ||
        (firmware->config.motor.direct_set_wheel_speeds !=
         TB6612_MotorBoard_SetWheelSpeeds)) {
        return 0;
    }
    return (TB6612MotorBoardContext *)firmware->config.motor.direct_context;
}

static void route_send_tuning_status(const CarFirmware *firmware)
{
    const CarConfig *config = &firmware->app.config;
    const TB6612MotorBoardContext *speed_context =
        route_get_speed_context_const(firmware);
    TB6612SpeedLoopConfig speed_config;

    if (TB6612_MotorBoardGetSpeedLoopConfig(speed_context, &speed_config)) {
        route_send_param("SPDKP_X1000", (int32_t)speed_config.kp_milli);
        route_send_param("SPDKI_X1000", (int32_t)speed_config.ki_milli);
        route_send_param("SPDKD_X1000", (int32_t)speed_config.kd_milli);
        route_send_param("SPDLIMIT_PCT",
                         (int32_t)speed_config.output_limit_percent);
    }

    route_send_param("LINEKP_X10000",
        round_float(config->line_follow_kp * 10000.0f));
    route_send_param("YAWKP_X1000",
        round_float(config->straight_heading_kp * 1000.0f));
    route_send_param("TURNKP_X1000",
        round_float(config->turn_heading_kp * 1000.0f));
    route_send_param("TURNMIN_X10",
        round_float(config->turn_min_speed_mm_s * 10.0f));
    route_send_param("ARCAXLE_MM_X10",
        round_float(config->line_sensor_to_axle_mm * 10.0f));
    route_send_param("CORNERAXLE_MM_X10",
        round_float(config->line_corner_pivot_approach_mm * 10.0f));
    route_send_param("REACQ_FRAMES",
        (int32_t)config->turn_line_reacquire_frames);
    route_send_param("REACQ_DEG_X10",
        round_float(config->turn_line_reacquire_min_angle_deg * 10.0f));
    route_send_param("GRAYOFF", (int32_t)config->gray_center_offset);
    route_send_param("GRAYDELTA", (int32_t)config->gray_relative_delta);
    route_send_param("TRACKCONF", (int32_t)config->gray_track_min_confidence);
    route_send_param("TRACKIN", (int32_t)config->gray_track_enter_frames);
    route_send_param("TRACKOUT", (int32_t)config->gray_track_lost_frames);
    route_send_param("TRACKMAX", (int32_t)config->gray_track_max_active);
    route_send_param("SPAN", (int32_t)config->gray_track_max_span);
    route_send_param("WIDEMIN", (int32_t)config->gray_wide_min_active);
    route_send_param("WIDEBG", (int32_t)config->gray_wide_min_background);
    route_send_param("ARCENTRYDEG_X10",
        round_float(config->arc_line_entry_min_angle_deg * 10.0f));
    route_send_param("ARCENTRY", (int32_t)config->arc_line_entry_frames);
    route_send_param("ARCBLEND_MS", (int32_t)config->arc_line_blend_ms);
    route_send_param("DIAGARM_PM",
        round_float(config->diagonal_line_arm_ratio * 1000.0f));
    route_send_param("DIAGAXLE_MM_X10",
        round_float(config->diagonal_line_approach_mm * 10.0f));
    route_send_param("CROSSPOS", (int32_t)config->line_cross_center_position);
    route_send_param("CROSSWIN_MM_X10",
        round_float(config->line_cross_capture_window_mm * 10.0f));
    route_send_param("CROSSKP_X10000",
        round_float(config->line_cross_capture_kp * 10000.0f));
    route_send_param("CROSSMAX_X10", round_float(
        config->line_cross_capture_max_correction_mm_s * 10.0f));
    route_send_param("SEEKCORR_X10", round_float(
        config->line_seek_max_correction_mm_s * 10.0f));
    route_send_param("TURNRATIO_PM",
        round_float(config->turn_inner_speed_ratio * 1000.0f));
    route_send_param("SETTLEMS", (int32_t)config->turn_settle_ms);
    route_send_param("SEARCHMM_X10",
        round_float(config->required_line_search_mm * 10.0f));
}

static void route_send_command_help(void)
{
    send_text("#TASK HELP | PARAMS | STOP\r\n");
    send_text("#SPDKP 0.250 | SPDKI 0.600 | SPDKD 0\r\n");
    send_text("#SPDLIMIT 35\r\n");
    send_text("#LINEKP 0.025 | YAWKP 1.0\r\n");
    send_text("#TURNKP 5.0 | TURNMIN 40\r\n");
    send_text("#ARCAXLE 150 | CORNERAXLE 50 | AXLE(alias) 50\r\n");
    send_text("#REACQ 2 | REACQDEG 80\r\n");
    send_text("#GRAYOFF 0 | GRAYDELTA 120 | TRACKCONF 250\r\n");
    send_text("#TRACKIN 2 | TRACKOUT 3 | TRACKMAX 4\r\n");
    send_text("#SPAN 4 | WIDEMIN 6 | WIDEBG 600\r\n");
    send_text("#ARCENTRYDEG 8 | ARCENTRY 2 | ARCBLEND 200\r\n");
    send_text("#DIAGARM 0.75 | DIAGAXLE 100 | CROSSPOS 1200\r\n");
    send_text("#CROSSWIN 80 | CROSSKP 0.025 | CROSSMAX 25\r\n");
    send_text("#SEEKCORR 25 | TURNRATIO 0.0 | SETTLEMS 300\r\n");
    send_text("#SEARCHMM 80 (STOP only; applies on next start)\r\n");
    send_text("#Task parameters are RAM-only until written to macros\r\n");
}

static void route_reset_line_controller(CarFirmware *firmware,
                                        uint32_t now_ms)
{
    firmware->app.executor.line_integral = 0.0f;
    firmware->app.executor.line_previous_error = 0.0f;
    firmware->app.executor.line_previous_ms = now_ms;
}

static void route_reset_gray_decisions(CarFirmware *firmware,
                                       uint32_t now_ms)
{
    CarRouteExecutor_ResetSegmentState(
        &firmware->app.executor, now_ms, firmware->imu_sample.yaw_deg);
}

static bool route_execute_command(CarFirmware *firmware, uint32_t now_ms)
{
    char *command = gRouteCommandBuffer;
    char *argument;
    float value;
    uint32_t integer_value;
    TB6612MotorBoardContext *speed_context;
    TB6612SpeedLoopConfig speed_config;

    while ((*command == ' ') || (*command == '\t') || (*command == ',')) {
        command++;
    }
    if (*command == '\0') {
        return false;
    }
    argument = route_split_command(command);
    route_uppercase_ascii(command);

    if ((strcmp(command, "HELP") == 0) || (strcmp(command, "?") == 0)) {
        if (argument != 0) {
            route_send_error("HELP takes no arguments");
            return true;
        }
        route_send_command_help();
        return true;
    }
    if (strcmp(command, "PARAMS") == 0) {
        if (argument != 0) {
            route_send_error("PARAMS takes no arguments");
            return true;
        }
        route_send_tuning_status(firmware);
        return true;
    }
    if (strcmp(command, "STOP") == 0) {
        if (argument != 0) {
            route_send_error("STOP takes no arguments");
            return true;
        }
        CarFirmware_ForceStop(firmware, 0U);
        send_text("#OK STOP\r\n");
        return true;
    }
    if ((strcmp(command, "SPDKP") == 0) ||
        (strcmp(command, "SPDKI") == 0) ||
        (strcmp(command, "SPDKD") == 0)) {
        uint32_t *gain;
        const char *parameter_name;

        speed_context = route_get_speed_context(firmware);
        if (!TB6612_MotorBoardGetSpeedLoopConfig(speed_context,
                                                 &speed_config)) {
            route_send_error("task speed loop is unavailable");
            return true;
        }
        if (strcmp(command, "SPDKP") == 0) {
            gain = &speed_config.kp_milli;
            parameter_name = "SPDKP_X1000";
        } else if (strcmp(command, "SPDKI") == 0) {
            gain = &speed_config.ki_milli;
            parameter_name = "SPDKI_X1000";
        } else {
            gain = &speed_config.kd_milli;
            parameter_name = "SPDKD_X1000";
        }
        if (argument == 0) {
            route_send_param(parameter_name, (int32_t)*gain);
            return true;
        }
        if (!route_parse_float(argument, 0.0f, 10.0f, &value)) {
            route_send_error("speed gain range is 0..10");
            return true;
        }
        *gain = (uint32_t)round_float(value * 1000.0f);
        if (!TB6612_MotorBoardUpdateSpeedLoopTuning(
                speed_context, speed_config.kp_milli,
                speed_config.ki_milli, speed_config.kd_milli,
                speed_config.output_limit_percent)) {
            route_send_error("speed gain update failed");
            return true;
        }
        route_send_ok_ram(command);
        route_send_param(parameter_name, (int32_t)*gain);
        return true;
    }
    if (strcmp(command, "SPDLIMIT") == 0) {
        speed_context = route_get_speed_context(firmware);
        if (!TB6612_MotorBoardGetSpeedLoopConfig(speed_context,
                                                 &speed_config)) {
            route_send_error("task speed loop is unavailable");
            return true;
        }
        if (argument == 0) {
            route_send_param("SPDLIMIT_PCT",
                             (int32_t)speed_config.output_limit_percent);
            return true;
        }
        if (!route_parse_u32(argument, 1U, TB6612_MAX_DUTY_PERCENT,
                             &integer_value)) {
            route_send_error("SPDLIMIT range is 1..80 percent");
            return true;
        }
        speed_config.output_limit_percent = (uint8_t)integer_value;
        if (!TB6612_MotorBoardUpdateSpeedLoopTuning(
                speed_context, speed_config.kp_milli,
                speed_config.ki_milli, speed_config.kd_milli,
                speed_config.output_limit_percent)) {
            route_send_error("speed limit update failed");
            return true;
        }
        route_send_ok_ram("SPDLIMIT");
        route_send_param("SPDLIMIT_PCT", (int32_t)integer_value);
        return true;
    }
    if (strcmp(command, "LINEKP") == 0) {
        if (argument == 0) {
            route_send_param("LINEKP_X10000", round_float(
                firmware->app.config.line_follow_kp * 10000.0f));
            return true;
        }
        if (!route_parse_float(argument, 0.0f, 0.1f, &value)) {
            route_send_error("LINEKP range is 0..0.100");
            return true;
        }
        firmware->app.config.line_follow_kp = value;
        firmware->config.car.line_follow_kp = value;
        route_reset_line_controller(firmware, now_ms);
        route_send_ok_ram("LINEKP");
        route_send_param("LINEKP_X10000", round_float(value * 10000.0f));
        return true;
    }
    if (strcmp(command, "YAWKP") == 0) {
        if (argument == 0) {
            route_send_param("YAWKP_X1000", round_float(
                firmware->app.config.straight_heading_kp * 1000.0f));
            return true;
        }
        if (!route_parse_float(argument, 0.0f, 10.0f, &value)) {
            route_send_error("YAWKP range is 0..10");
            return true;
        }
        firmware->app.config.straight_heading_kp = value;
        firmware->config.car.straight_heading_kp = value;
        route_send_ok_ram("YAWKP");
        route_send_param("YAWKP_X1000", round_float(value * 1000.0f));
        return true;
    }
    if (strcmp(command, "TURNKP") == 0) {
        if (argument == 0) {
            route_send_param("TURNKP_X1000", round_float(
                firmware->app.config.turn_heading_kp * 1000.0f));
            return true;
        }
        if (!route_parse_float(argument, 0.0f, 20.0f, &value)) {
            route_send_error("TURNKP range is 0..20");
            return true;
        }
        firmware->app.config.turn_heading_kp = value;
        firmware->config.car.turn_heading_kp = value;
        route_send_ok_ram("TURNKP");
        route_send_param("TURNKP_X1000", round_float(value * 1000.0f));
        return true;
    }
    if (strcmp(command, "TURNMIN") == 0) {
        if (argument == 0) {
            route_send_param("TURNMIN_X10", round_float(
                firmware->app.config.turn_min_speed_mm_s * 10.0f));
            return true;
        }
        if (!route_parse_float(argument, 0.0f,
                               firmware->app.config.max_wheel_speed_mm_s,
                               &value)) {
            route_send_error("TURNMIN range is 0..max wheel speed");
            return true;
        }
        firmware->app.config.turn_min_speed_mm_s = value;
        firmware->config.car.turn_min_speed_mm_s = value;
        route_send_ok_ram("TURNMIN");
        route_send_param("TURNMIN_X10", round_float(value * 10.0f));
        return true;
    }
    if (strcmp(command, "ARCAXLE") == 0) {
        if (argument == 0) {
            route_send_param("ARCAXLE_MM_X10", round_float(
                firmware->app.config.line_sensor_to_axle_mm * 10.0f));
            return true;
        }
        if (!route_parse_float(argument, 0.0f, 300.0f, &value)) {
            route_send_error("ARCAXLE range is 0..300 mm");
            return true;
        }
        firmware->app.config.line_sensor_to_axle_mm = value;
        firmware->config.car.line_sensor_to_axle_mm = value;
        route_send_ok_ram("ARCAXLE");
        route_send_param("ARCAXLE_MM_X10", round_float(value * 10.0f));
        return true;
    }
    if ((strcmp(command, "CORNERAXLE") == 0) ||
        (strcmp(command, "AXLE") == 0)) {
        if (argument == 0) {
            route_send_param("CORNERAXLE_MM_X10", round_float(
                firmware->app.config.line_corner_pivot_approach_mm * 10.0f));
            return true;
        }
        if (!route_parse_float(argument, 0.0f, 300.0f, &value)) {
            route_send_error("CORNERAXLE range is 0..300 mm");
            return true;
        }
        firmware->app.config.line_corner_pivot_approach_mm = value;
        firmware->config.car.line_corner_pivot_approach_mm = value;
        route_send_ok_ram(strcmp(command, "AXLE") == 0 ?
                          "AXLE=CORNERAXLE" : "CORNERAXLE");
        route_send_param("CORNERAXLE_MM_X10", round_float(value * 10.0f));
        return true;
    }
    if (strcmp(command, "REACQ") == 0) {
        if (argument == 0) {
            route_send_param("REACQ_FRAMES", (int32_t)
                firmware->app.config.turn_line_reacquire_frames);
            return true;
        }
        if (!route_parse_u32(argument, 1U, 10U, &integer_value)) {
            route_send_error("REACQ range is 1..10 frames");
            return true;
        }
        firmware->app.config.turn_line_reacquire_frames =
            (uint8_t)integer_value;
        firmware->config.car.turn_line_reacquire_frames =
            (uint8_t)integer_value;
        firmware->app.executor.turn_line_reacquire_streak = 0U;
        route_send_ok_ram("REACQ");
        route_send_param("REACQ_FRAMES", (int32_t)integer_value);
        return true;
    }
    if (strcmp(command, "REACQDEG") == 0) {
        if (argument == 0) {
            route_send_param("REACQ_DEG_X10", round_float(
                firmware->app.config.turn_line_reacquire_min_angle_deg *
                10.0f));
            return true;
        }
        if (!route_parse_float(argument, 45.0f, 90.0f, &value)) {
            route_send_error("REACQDEG range is 45..90 deg");
            return true;
        }
        firmware->app.config.turn_line_reacquire_min_angle_deg = value;
        firmware->config.car.turn_line_reacquire_min_angle_deg = value;
        firmware->app.executor.turn_line_reacquire_streak = 0U;
        route_send_ok_ram("REACQDEG");
        route_send_param("REACQ_DEG_X10", round_float(value * 10.0f));
        return true;
    }
    if (strcmp(command, "GRAYOFF") == 0) {
        int32_t offset;

        if (argument == 0) {
            route_send_param("GRAYOFF",
                (int32_t)firmware->app.config.gray_center_offset);
            return true;
        }
        if (!route_parse_float(argument, -3500.0f, 3500.0f, &value)) {
            route_send_error("GRAYOFF range is -3500..3500");
            return true;
        }
        offset = round_float(value);
        firmware->app.config.gray_center_offset = (int16_t)offset;
        firmware->config.car.gray_center_offset = (int16_t)offset;
        route_reset_gray_decisions(firmware, now_ms);
        route_send_ok_ram("GRAYOFF");
        route_send_param("GRAYOFF", offset);
        return true;
    }
    if ((strcmp(command, "GRAYDELTA") == 0) ||
        (strcmp(command, "TRACKCONF") == 0) ||
        (strcmp(command, "TRACKIN") == 0) ||
        (strcmp(command, "TRACKOUT") == 0) ||
        (strcmp(command, "TRACKMAX") == 0) ||
        (strcmp(command, "SPAN") == 0) ||
        (strcmp(command, "WIDEMIN") == 0) ||
        (strcmp(command, "WIDEBG") == 0)) {
        uint32_t current;
        uint32_t minimum;
        uint32_t maximum;
        const char *range_error;

        if (strcmp(command, "GRAYDELTA") == 0) {
            current = firmware->app.config.gray_relative_delta;
            minimum = 1U;
            maximum = 1000U;
            range_error = "GRAYDELTA range is 1..1000";
        } else if (strcmp(command, "TRACKCONF") == 0) {
            current = firmware->app.config.gray_track_min_confidence;
            minimum = 0U;
            maximum = 8000U;
            range_error = "TRACKCONF range is 0..8000";
        } else if (strcmp(command, "TRACKIN") == 0) {
            current = firmware->app.config.gray_track_enter_frames;
            minimum = 1U;
            maximum = 10U;
            range_error = "TRACKIN range is 1..10";
        } else if (strcmp(command, "TRACKOUT") == 0) {
            current = firmware->app.config.gray_track_lost_frames;
            minimum = 1U;
            maximum = 10U;
            range_error = "TRACKOUT range is 1..10";
        } else if (strcmp(command, "TRACKMAX") == 0) {
            current = firmware->app.config.gray_track_max_active;
            minimum = 1U;
            maximum = 8U;
            range_error = "TRACKMAX range is 1..8";
        } else if (strcmp(command, "SPAN") == 0) {
            current = firmware->app.config.gray_track_max_span;
            minimum = 1U;
            maximum = 8U;
            range_error = "SPAN range is 1..8";
        } else if (strcmp(command, "WIDEMIN") == 0) {
            current = firmware->app.config.gray_wide_min_active;
            minimum = 1U;
            maximum = 8U;
            range_error = "WIDEMIN range is 1..8";
        } else {
            current = firmware->app.config.gray_wide_min_background;
            minimum = 0U;
            maximum = 1000U;
            range_error = "WIDEBG range is 0..1000";
        }
        if (argument == 0) {
            route_send_param(command, (int32_t)current);
            return true;
        }
        if (!route_parse_u32(argument, minimum, maximum, &integer_value)) {
            route_send_error(range_error);
            return true;
        }
        if (strcmp(command, "GRAYDELTA") == 0) {
            firmware->app.config.gray_relative_delta =
                (uint16_t)integer_value;
            firmware->config.car.gray_relative_delta =
                (uint16_t)integer_value;
        } else if (strcmp(command, "TRACKCONF") == 0) {
            firmware->app.config.gray_track_min_confidence =
                (uint16_t)integer_value;
            firmware->config.car.gray_track_min_confidence =
                (uint16_t)integer_value;
        } else if (strcmp(command, "TRACKIN") == 0) {
            firmware->app.config.gray_track_enter_frames =
                (uint8_t)integer_value;
            firmware->config.car.gray_track_enter_frames =
                (uint8_t)integer_value;
        } else if (strcmp(command, "TRACKOUT") == 0) {
            firmware->app.config.gray_track_lost_frames =
                (uint8_t)integer_value;
            firmware->config.car.gray_track_lost_frames =
                (uint8_t)integer_value;
        } else if (strcmp(command, "TRACKMAX") == 0) {
            firmware->app.config.gray_track_max_active =
                (uint8_t)integer_value;
            firmware->config.car.gray_track_max_active =
                (uint8_t)integer_value;
        } else if (strcmp(command, "SPAN") == 0) {
            firmware->app.config.gray_track_max_span =
                (uint8_t)integer_value;
            firmware->config.car.gray_track_max_span =
                (uint8_t)integer_value;
        } else if (strcmp(command, "WIDEMIN") == 0) {
            firmware->app.config.gray_wide_min_active =
                (uint8_t)integer_value;
            firmware->config.car.gray_wide_min_active =
                (uint8_t)integer_value;
        } else {
            firmware->app.config.gray_wide_min_background =
                (uint16_t)integer_value;
            firmware->config.car.gray_wide_min_background =
                (uint16_t)integer_value;
        }
        route_reset_gray_decisions(firmware, now_ms);
        route_send_ok_ram(command);
        route_send_param(command, (int32_t)integer_value);
        return true;
    }
    if (strcmp(command, "ARCENTRYDEG") == 0) {
        if (argument == 0) {
            route_send_param("ARCENTRYDEG_X10", round_float(
                firmware->app.config.arc_line_entry_min_angle_deg * 10.0f));
            return true;
        }
        if (!route_parse_float(argument, 0.0f, 90.0f, &value)) {
            route_send_error("ARCENTRYDEG range is 0..90 deg");
            return true;
        }
        firmware->app.config.arc_line_entry_min_angle_deg = value;
        firmware->config.car.arc_line_entry_min_angle_deg = value;
        firmware->app.executor.arc_line_locked = false;
        firmware->app.executor.arc_line_entry_streak = 0U;
        firmware->app.executor.arc_line_lock_ms = now_ms;
        firmware->app.executor.arc_line_blend_permille = 0U;
        route_send_ok_ram("ARCENTRYDEG");
        route_send_param("ARCENTRYDEG_X10", round_float(value * 10.0f));
        return true;
    }
    if (strcmp(command, "ARCENTRY") == 0) {
        if (argument == 0) {
            route_send_param("ARCENTRY",
                (int32_t)firmware->app.config.arc_line_entry_frames);
            return true;
        }
        if (!route_parse_u32(argument, 1U, 10U, &integer_value)) {
            route_send_error("ARCENTRY range is 1..10 frames");
            return true;
        }
        firmware->app.config.arc_line_entry_frames =
            (uint8_t)integer_value;
        firmware->config.car.arc_line_entry_frames =
            (uint8_t)integer_value;
        firmware->app.executor.arc_line_locked = false;
        firmware->app.executor.arc_line_entry_streak = 0U;
        firmware->app.executor.arc_line_lock_ms = now_ms;
        firmware->app.executor.arc_line_blend_permille = 0U;
        route_send_ok_ram("ARCENTRY");
        route_send_param("ARCENTRY", (int32_t)integer_value);
        return true;
    }
    if (strcmp(command, "ARCBLEND") == 0) {
        if (argument == 0) {
            route_send_param("ARCBLEND_MS",
                (int32_t)firmware->app.config.arc_line_blend_ms);
            return true;
        }
        if (!route_parse_u32(argument, 0U, 2000U, &integer_value)) {
            route_send_error("ARCBLEND range is 0..2000 ms");
            return true;
        }
        firmware->app.config.arc_line_blend_ms =
            (uint16_t)integer_value;
        firmware->config.car.arc_line_blend_ms =
            (uint16_t)integer_value;
        if (firmware->app.executor.arc_line_locked) {
            firmware->app.executor.arc_line_lock_ms = now_ms;
            firmware->app.executor.arc_line_blend_permille = 0U;
        }
        route_send_ok_ram("ARCBLEND");
        route_send_param("ARCBLEND_MS", (int32_t)integer_value);
        return true;
    }
    if (strcmp(command, "DIAGARM") == 0) {
        if (argument == 0) {
            route_send_param("DIAGARM_PM", round_float(
                firmware->app.config.diagonal_line_arm_ratio * 1000.0f));
            return true;
        }
        if (!route_parse_float(argument, 0.0f, 1.0f, &value)) {
            route_send_error("DIAGARM range is 0..1");
            return true;
        }
        firmware->app.config.diagonal_line_arm_ratio = value;
        firmware->config.car.diagonal_line_arm_ratio = value;
        route_reset_gray_decisions(firmware, now_ms);
        route_send_ok_ram("DIAGARM");
        route_send_param("DIAGARM_PM", round_float(value * 1000.0f));
        return true;
    }
    if (strcmp(command, "DIAGAXLE") == 0) {
        if (argument == 0) {
            route_send_param("DIAGAXLE_MM_X10", round_float(
                firmware->app.config.diagonal_line_approach_mm * 10.0f));
            return true;
        }
        if (!route_parse_float(argument, 0.0f, 300.0f, &value)) {
            route_send_error("DIAGAXLE range is 0..300 mm");
            return true;
        }
        firmware->app.config.diagonal_line_approach_mm = value;
        firmware->config.car.diagonal_line_approach_mm = value;
        route_send_ok_ram("DIAGAXLE");
        route_send_param("DIAGAXLE_MM_X10", round_float(value * 10.0f));
        return true;
    }
    if (strcmp(command, "CROSSPOS") == 0) {
        if (argument == 0) {
            route_send_param("CROSSPOS", (int32_t)
                firmware->app.config.line_cross_center_position);
            return true;
        }
        if (!route_parse_u32(argument, 0U, 3500U, &integer_value)) {
            route_send_error("CROSSPOS range is 0..3500");
            return true;
        }
        firmware->app.config.line_cross_center_position =
            (int16_t)integer_value;
        firmware->config.car.line_cross_center_position =
            (int16_t)integer_value;
        firmware->app.executor.line_event_streak = 0U;
        firmware->app.executor.line_capture_position_valid = false;
        route_send_ok_ram("CROSSPOS");
        route_send_param("CROSSPOS", (int32_t)integer_value);
        return true;
    }
    if (strcmp(command, "CROSSWIN") == 0) {
        if (argument == 0) {
            route_send_param("CROSSWIN_MM_X10", round_float(
                firmware->app.config.line_cross_capture_window_mm * 10.0f));
            return true;
        }
        if (!route_parse_float(argument, 0.0f, 300.0f, &value)) {
            route_send_error("CROSSWIN range is 0..300 mm");
            return true;
        }
        firmware->app.config.line_cross_capture_window_mm = value;
        firmware->config.car.line_cross_capture_window_mm = value;
        firmware->app.executor.line_event_streak = 0U;
        firmware->app.executor.line_capture_position_valid = false;
        route_send_ok_ram("CROSSWIN");
        route_send_param("CROSSWIN_MM_X10", round_float(value * 10.0f));
        return true;
    }
    if ((strcmp(command, "CROSSKP") == 0) ||
        (strcmp(command, "CROSSMAX") == 0) ||
        (strcmp(command, "SEEKCORR") == 0)) {
        float *app_parameter;
        float *platform_parameter;
        float minimum;
        float maximum;
        float scale;
        const char *parameter_name;
        const char *range_error;

        if (strcmp(command, "CROSSKP") == 0) {
            app_parameter = &firmware->app.config.line_cross_capture_kp;
            platform_parameter =
                &firmware->config.car.line_cross_capture_kp;
            minimum = 0.0f;
            maximum = 0.1f;
            scale = 10000.0f;
            parameter_name = "CROSSKP_X10000";
            range_error = "CROSSKP range is 0..0.100";
        } else if (strcmp(command, "CROSSMAX") == 0) {
            app_parameter = &firmware->app.config.
                line_cross_capture_max_correction_mm_s;
            platform_parameter = &firmware->config.car.
                line_cross_capture_max_correction_mm_s;
            minimum = -100.0f;
            maximum = 100.0f;
            scale = 10.0f;
            parameter_name = "CROSSMAX_X10";
            range_error = "CROSSMAX range is -100..100; <=0 is off";
        } else {
            app_parameter =
                &firmware->app.config.line_seek_max_correction_mm_s;
            platform_parameter =
                &firmware->config.car.line_seek_max_correction_mm_s;
            minimum = -100.0f;
            maximum = 100.0f;
            scale = 10.0f;
            parameter_name = "SEEKCORR_X10";
            range_error = "SEEKCORR range is -100..100; <=0 is off";
        }
        if (argument == 0) {
            route_send_param(parameter_name,
                             round_float(*app_parameter * scale));
            return true;
        }
        if (!route_parse_float(argument, minimum, maximum, &value)) {
            route_send_error(range_error);
            return true;
        }
        *app_parameter = value;
        *platform_parameter = value;
        route_reset_line_controller(firmware, now_ms);
        firmware->app.executor.line_capture_guide_mm_s = 0.0f;
        firmware->app.executor.line_seek_correction_mm_s = 0.0f;
        route_send_ok_ram(command);
        route_send_param(parameter_name, round_float(value * scale));
        return true;
    }
    if (strcmp(command, "TURNRATIO") == 0) {
        if (argument == 0) {
            route_send_param("TURNRATIO_PM", round_float(
                firmware->app.config.turn_inner_speed_ratio * 1000.0f));
            return true;
        }
        if (!route_parse_float(argument, 0.0f, 0.8f, &value)) {
            route_send_error("TURNRATIO range is 0..0.8");
            return true;
        }
        firmware->app.config.turn_inner_speed_ratio = value;
        firmware->config.car.turn_inner_speed_ratio = value;
        route_send_ok_ram("TURNRATIO");
        route_send_param("TURNRATIO_PM", round_float(value * 1000.0f));
        return true;
    }
    if (strcmp(command, "SETTLEMS") == 0) {
        bool settle_active;

        if (argument == 0) {
            route_send_param("SETTLEMS",
                (int32_t)firmware->app.config.turn_settle_ms);
            return true;
        }
        if (!route_parse_u32(argument, 0U, 2000U, &integer_value)) {
            route_send_error("SETTLEMS range is 0..2000 ms");
            return true;
        }
        settle_active =
            (firmware->app.executor.post_turn_settle_until_ms != 0U) &&
            ((int32_t)(firmware->app.executor.post_turn_settle_until_ms -
                       now_ms) > 0);
        firmware->app.config.turn_settle_ms = (uint16_t)integer_value;
        firmware->config.car.turn_settle_ms = (uint16_t)integer_value;
        if (settle_active) {
            firmware->app.executor.post_turn_settle_until_ms =
                now_ms + integer_value;
        }
        route_send_ok_ram("SETTLEMS");
        route_send_param("SETTLEMS", (int32_t)integer_value);
        return true;
    }
    if (strcmp(command, "SEARCHMM") == 0) {
        if (argument == 0) {
            route_send_param("SEARCHMM_X10", round_float(
                firmware->app.config.required_line_search_mm * 10.0f));
            return true;
        }
        if (firmware->app.armed || firmware->app.executor.running) {
            route_send_error("SEARCHMM requires STOP; retry before start");
            return true;
        }
        if (!route_parse_float(argument, 0.0f, 300.0f, &value)) {
            route_send_error("SEARCHMM range is 0..300 mm");
            return true;
        }
        firmware->app.config.required_line_search_mm = value;
        firmware->config.car.required_line_search_mm = value;
        route_send_ok_ram("SEARCHMM");
        route_send_param("SEARCHMM_X10", round_float(value * 10.0f));
        return true;
    }

    route_send_error("unavailable in TASK mode; send HELP");
    return true;
}

void VofaTelemetry_RouteCommandInit(void)
{
    VofaTelemetry_TxInit();
    gRouteRxHead = 0U;
    gRouteRxTail = 0U;
    gRouteRxOverflow = false;
    gRouteCommandLength = 0U;
    gRouteLastCommandByteMs = 0U;
    gRouteDiscardCommand = false;
}

void VofaTelemetry_TxInit(void)
{
    uint32_t interrupt_state = __get_PRIMASK();

    __disable_irq();
    gVofaTxHead = 0U;
    gVofaTxTail = 0U;
    gVofaTxDropCount = 0U;
    DL_UART_Main_disableInterrupt(
        UART_BLUETOOTH_INST, DL_UART_MAIN_INTERRUPT_TX);
    if (interrupt_state == 0U) {
        __enable_irq();
    }
}

void VofaTelemetry_TxIrqHandler(void)
{
    vofa_tx_fill_fifo();
}

void VofaTelemetry_RouteCommandPushRxFromIsr(uint8_t byte)
{
    uint8_t next = (uint8_t)((gRouteRxHead + 1U) % ROUTE_COMMAND_RX_SIZE);

    if (next == gRouteRxTail) {
        gRouteRxOverflow = true;
        return;
    }
    gRouteRxBuffer[gRouteRxHead] = byte;
    gRouteRxHead = next;
}

bool VofaTelemetry_ProcessRouteCommands(CarFirmware *firmware,
                                        uint32_t uptimeMs)
{
    bool event = false;

    if (firmware == 0) {
        return false;
    }

    if (gRouteRxOverflow) {
        /* Drop an incomplete burst as one unit; never execute its tail as a
         * different command after the ISR ring buffer overflows. */
        NVIC_DisableIRQ(UART_BLUETOOTH_INST_INT_IRQN);
        gRouteRxTail = gRouteRxHead;
        gRouteRxOverflow = false;
        gRouteCommandLength = 0U;
        gRouteDiscardCommand = true;
        NVIC_EnableIRQ(UART_BLUETOOTH_INST_INT_IRQN);
        route_send_error("RX overflow; command discarded");
        event = true;
    }

    while (gRouteRxTail != gRouteRxHead) {
        uint8_t byte = gRouteRxBuffer[gRouteRxTail];

        gRouteRxTail = (uint8_t)((gRouteRxTail + 1U) %
                                 ROUTE_COMMAND_RX_SIZE);
        gRouteLastCommandByteMs = uptimeMs;

        if ((byte == '\r') || (byte == '\n')) {
            if (gRouteDiscardCommand) {
                gRouteDiscardCommand = false;
                gRouteCommandLength = 0U;
            } else if (gRouteCommandLength != 0U) {
                gRouteCommandBuffer[gRouteCommandLength] = '\0';
                event = route_execute_command(firmware, uptimeMs) || event;
                gRouteCommandLength = 0U;
            }
        } else if (gRouteDiscardCommand) {
            /* Wait for the delimiter so a truncated suffix cannot execute. */
        } else if ((byte == '\b') || (byte == 0x7FU)) {
            if (gRouteCommandLength != 0U) {
                gRouteCommandLength--;
            }
        } else if ((byte >= 32U) && (byte <= 126U)) {
            if (gRouteCommandLength < (ROUTE_COMMAND_BUFFER_SIZE - 1U)) {
                gRouteCommandBuffer[gRouteCommandLength++] = (char)byte;
            } else {
                gRouteCommandLength = 0U;
                gRouteDiscardCommand = true;
                route_send_error("command too long; discarded");
                event = true;
            }
        }
    }

    if ((gRouteCommandLength != 0U) && !gRouteDiscardCommand &&
        ((uint32_t)(uptimeMs - gRouteLastCommandByteMs) >=
         ROUTE_COMMAND_IDLE_MS)) {
        gRouteCommandBuffer[gRouteCommandLength] = '\0';
        event = route_execute_command(firmware, uptimeMs) || event;
        gRouteCommandLength = 0U;
    } else if (gRouteDiscardCommand &&
               ((uint32_t)(uptimeMs - gRouteLastCommandByteMs) >=
                ROUTE_COMMAND_IDLE_MS)) {
        /* VOFA can send without CR/LF; idle also ends an oversized command. */
        gRouteDiscardCommand = false;
        gRouteCommandLength = 0U;
    }
    return event;
}

void VofaTelemetry_SendBanner(void)
{
    send_text("#READY Bluetooth encoder and speed-loop debug\r\n");
    send_text("#CSV mode,motion,target_l,target_r,rpm_l,rpm_r,"
              "duty_l,duty_r,delta_l,delta_r,total_l,total_r,"
              "cpr_l,cpr_r,kp_x1000,ki_x1000,kd_x1000,"
              "limit,duty_step,rx,errors,failsafe,ms\r\n");
    send_text("#Send HELP for commands\r\n");
}

void VofaTelemetry_Send(
    const BluetoothControlStatus *status, uint32_t uptimeMs)
{
    char line[256];
    char *out = line;

    out = append_u32_field(out, (uint32_t) status->mode);
    out = append_u32_field(out, (uint32_t) status->motion);
    out = append_i32_field(out, status->targetLeftRpm);
    out = append_i32_field(out, status->targetRightRpm);
    out = append_i32_field(out, status->measuredLeftRpm);
    out = append_i32_field(out, status->measuredRightRpm);
    out = append_i32_field(out, status->leftCommandPercent);
    out = append_i32_field(out, status->rightCommandPercent);
    out = append_i32_field(out, status->leftDeltaCount);
    out = append_i32_field(out, status->rightDeltaCount);
    out = append_i32_field(out, status->leftTotalCount);
    out = append_i32_field(out, status->rightTotalCount);
    out = append_u32_field(out, status->leftCountsPerRev);
    out = append_u32_field(out, status->rightCountsPerRev);
    out = append_u32_field(out, status->kpMilli);
    out = append_u32_field(out, status->kiMilli);
    out = append_u32_field(out, status->kdMilli);
    out = append_u32_field(out, status->speedLimitPercent);
    out = append_u32_field(out, status->dutyPercent);
    out = append_u32_field(out, status->rxCount);
    out = append_u32_field(out, status->errorCount);
    out = append_u32_field(out, status->failsafeCount);
    out = append_unsigned(out, uptimeMs);
    *out++ = '\r';
    *out++ = '\n';
    (void)vofa_tx_enqueue(line, (uint16_t)(out - line));
}

void VofaTelemetry_SendRouteBanner(void)
{
    send_text("#ROUTE READY H2026 OFFICIAL B2 DEFAULT V53\r\n");
    send_text("#Send HELP for task-safe RAM tuning commands\r\n");
    send_text("#RCSV stream_id,ms,seg,type,phase,progress_mm,yaw_x10,line_valid,"
              "position,confidence,active,mask,right_pm,span,candidate,"
              "event_streak,reacq_streak,axle_mm,cmd_l,cmd_r,pwm_l,pwm_r,"
              "exit,fault,g0,g1,g2,g3,g4,g5,g6,g7,"
              "speed_loop,target_rpm_l,target_rpm_r,rpm_l,rpm_r,enc_dl,enc_dr,"
              "yaw_rate_x10,target_yaw_x10,yaw_error_x10,yaw_corr,spd_updates,"
              "speed_dt_ms,pattern,bg,contrast,track_pos,track_conf,track_count,"
              "track_mask,track_span,clusters,track_locked,track_in,track_out,"
              "arc_locked,arc_entry,arc_blend_pm,semantic_miss,turn_ratio_pm,"
              "settle_active,capture_guide,seek_corr,search_used,"
              "search_active,mode,run_ms,result_ms,result_valid,"
              "center_ref_x10_mm_s,track_gap_mm,"
              "kappa_x1e6_inv_mm\r\n");
    send_text("#pattern 0=NONE 1=NARROW 2=WIDE 3=SPLIT\r\n");
    send_text("#phase 0=TRACK 1=AXLE 2=CAPTURE 3=SEARCH\r\n");
    send_text("#exit 0=NONE 1=LINE 2=DIST 3=ANGLE 4=REACQ 5=LINE_MISS\r\n");
}

void VofaTelemetry_SendRoute(const CarFirmware *firmware, uint32_t uptimeMs)
{
    char line[768];
    char *out = line;
    const CarRouteExecutor *executor;
    const CarLineEstimate *estimate;
    uint32_t segment_type = 255U;
    uint32_t faults;
    float progress_mm;
    float target_yaw_deg;
    float yaw_error_deg = 0.0f;
    float yaw_correction = 0.0f;
    float search_used_mm = 0.0f;
    bool settle_active;
    TB6612SpeedLoopStatus speed_loop = {0};

    if (firmware == 0) {
        return;
    }

    executor = &firmware->app.executor;
    estimate = &firmware->app.line;
    if ((executor->route != 0) &&
        (executor->index < executor->route->count)) {
        segment_type = (uint32_t)executor->route->segments[
            executor->index].type;
    }
    target_yaw_deg = firmware->imu_sample.yaw_deg;
    if (segment_type == (uint32_t)CAR_SEGMENT_STRAIGHT) {
        float yaw_correction_limit =
            firmware->app.config.straight_heading_max_correction_mm_s;

        if (yaw_correction_limit <= 0.0f) {
            yaw_correction_limit =
                firmware->app.config.max_wheel_speed_mm_s * 0.4f;
        }
        target_yaw_deg = executor->pending_straight_yaw_valid ?
            executor->pending_straight_yaw_deg :
            executor->segment_start_yaw_deg;
        yaw_error_deg = target_yaw_deg - firmware->imu_sample.yaw_deg;
        yaw_correction = yaw_error_deg * firmware->app.config.straight_heading_kp;
        if (yaw_correction > yaw_correction_limit) {
            yaw_correction = yaw_correction_limit;
        } else if (yaw_correction < -yaw_correction_limit) {
            yaw_correction = -yaw_correction_limit;
        }
    }
    if (firmware->config.motor.direct_set_wheel_speeds ==
        TB6612_MotorBoard_SetWheelSpeeds) {
        TB6612_MotorBoardGetSpeedLoopStatus(
            (const TB6612MotorBoardContext *)
                firmware->config.motor.direct_context,
            &speed_loop);
    }
    progress_mm = firmware->app.odometry.center_distance_mm -
                  executor->segment_start_distance_mm;
    if (executor->line_search_budget_active) {
        search_used_mm =
            progress_mm - executor->line_search_start_progress_mm;
        if (search_used_mm < 0.0f) {
            search_used_mm = -search_used_mm;
        }
    }
    faults = firmware->hardware_faults | firmware->output.faults;
    settle_active = (executor->post_turn_settle_until_ms != 0U) &&
        ((int32_t)(executor->post_turn_settle_until_ms - uptimeMs) > 0);

    out = append_u32_field(out, 2026U);
    out = append_u32_field(out, uptimeMs);
    out = append_u32_field(out, executor->index);
    out = append_u32_field(out, segment_type);
    out = append_u32_field(out, (uint32_t)executor->line_follow_phase);
    out = append_i32_field(out, round_float(progress_mm));
    out = append_i32_field(out, round_float(firmware->imu_sample.yaw_deg *
                                             10.0f));
    out = append_u32_field(out, estimate->valid ? 1U : 0U);
    out = append_i32_field(out, estimate->position);
    out = append_u32_field(out, estimate->confidence);
    out = append_u32_field(out, estimate->active_count);
    out = append_u32_field(out, estimate->active_mask);
    out = append_u32_field(out, estimate->right_ratio_permille);
    out = append_u32_field(out, estimate->active_span);
    out = append_u32_field(out, executor->line_corner_candidate ? 1U : 0U);
    out = append_u32_field(out, executor->line_event_streak);
    out = append_u32_field(out, executor->turn_line_reacquire_streak);
    out = append_i32_field(out, round_float(
        executor->line_corner_approach_mm));
    out = append_i32_field(out, round_float(
        firmware->output.motor.left_mm_s));
    out = append_i32_field(out, round_float(
        firmware->output.motor.right_mm_s));
    out = append_i32_field(out, TB6612_GetLeftCommand());
    out = append_i32_field(out, TB6612_GetRightCommand());
    out = append_u32_field(out,
                           (uint32_t)executor->last_motion_exit_reason);
    out = append_u32_field(out, faults);
    for (uint8_t i = 0U; i < 7U; i++) {
        out = append_u32_field(out, firmware->gray_sample.normalized[i]);
    }
    out = append_u32_field(out, firmware->gray_sample.normalized[7]);
    out = append_u32_field(out, speed_loop.enabled ? 1U : 0U);
    out = append_i32_field(out, speed_loop.target_left_rpm);
    out = append_i32_field(out, speed_loop.target_right_rpm);
    out = append_i32_field(out, speed_loop.measured_left_rpm);
    out = append_i32_field(out, speed_loop.measured_right_rpm);
    out = append_i32_field(out, speed_loop.left_delta_count);
    out = append_i32_field(out, speed_loop.right_delta_count);
    out = append_i32_field(out, round_float(
        firmware->imu_sample.yaw_rate_dps * 10.0f));
    out = append_i32_field(out, round_float(target_yaw_deg * 10.0f));
    out = append_i32_field(out, round_float(yaw_error_deg * 10.0f));
    out = append_i32_field(out, round_float(yaw_correction));
    out = append_u32_field(out, speed_loop.update_count);
    out = append_u32_field(out, speed_loop.sample_elapsed_ms);
    out = append_u32_field(out, (uint32_t)estimate->pattern);
    out = append_u32_field(out, estimate->adaptive_background);
    out = append_u32_field(out, estimate->adaptive_contrast);
    out = append_i32_field(out, estimate->track_position);
    out = append_u32_field(out, estimate->track_confidence);
    out = append_u32_field(out, estimate->track_active_count);
    out = append_u32_field(out, estimate->track_active_mask);
    out = append_u32_field(out, estimate->track_active_span);
    out = append_u32_field(out, estimate->track_cluster_count);
    out = append_u32_field(out, executor->track_locked ? 1U : 0U);
    out = append_u32_field(out, executor->track_enter_streak);
    out = append_u32_field(out, executor->track_lost_streak);
    out = append_u32_field(out, executor->arc_line_locked ? 1U : 0U);
    out = append_u32_field(out, executor->arc_line_entry_streak);
    out = append_u32_field(out, executor->arc_line_blend_permille);
    out = append_u32_field(out, executor->semantic_line_miss_count);
    out = append_u32_field(out, (uint32_t)round_float(
        firmware->app.config.turn_inner_speed_ratio * 1000.0f));
    out = append_u32_field(out, settle_active ? 1U : 0U);
    out = append_i32_field(out, round_float(
        executor->line_capture_guide_mm_s));
    out = append_i32_field(out, round_float(
        executor->line_seek_correction_mm_s));
    out = append_i32_field(out, round_float(search_used_mm));
    out = append_u32_field(out,
        executor->line_search_budget_active ? 1U : 0U);
    out = append_u32_field(out, (uint32_t)firmware->config.mode);
    out = append_u32_field(out, firmware->output.run_time_ms);
    out = append_u32_field(out, firmware->output.result_time_ms);
    out = append_u32_field(out, firmware->output.result_valid ? 1U : 0U);
    out = append_i32_field(out, round_float(
        executor->track_center_speed_mm_s * 10.0f));
    out = append_i32_field(out, round_float(executor->track_line_gap_mm));
    /* FireWater rejects a trailing empty token, so the final field has no
     * comma. Curvature is scaled to preserve -0.002 1/mm as -2000. */
    out = append_signed(out, round_float(
        executor->track_curvature_inv_mm * 1000000.0f));
    *out++ = '\r';
    *out++ = '\n';
    (void)vofa_tx_enqueue(line, (uint16_t)(out - line));
}
