#include "platform/ti_mspm0_platform.h"

#include <string.h>

#include "app/h2026_task.h"
#include "platform/ti_mspm0_platform_config.h"
#include "encoder.h"
#include "tb6612.h"
#include "ti_msp_dl_config.h"

static volatile uint32_t g_millis;
static volatile bool g_adc_ready;
static volatile uint8_t g_motor_rx[H2024_UART_RX_BUFFER_SIZE];
static volatile uint8_t g_motor_rx_head;
static volatile uint8_t g_motor_rx_tail;
static volatile TiMspm0PlatformDiagnostics g_diagnostics;
static TB6612MotorBoardContext g_tb6612_context;

static uint32_t TiMspm0Platform_MillisAdapter(void *context)
{
    (void)context;
    return TiMspm0Platform_Millis();
}

static uint8_t TiMotor_NextRxIndex(uint8_t index)
{
    index++;
    return (index >= H2024_UART_RX_BUFFER_SIZE) ? 0U : index;
}

static void TiMotor_QueueRxByte(uint8_t byte)
{
    uint8_t next = TiMotor_NextRxIndex(g_motor_rx_head);

    g_diagnostics.motor_rx_bytes++;
    if (next == g_motor_rx_tail) {
        g_diagnostics.motor_rx_overflows++;
        return;
    }
    g_motor_rx[g_motor_rx_head] = byte;
    g_motor_rx_head = next;
}

static bool TiMotor_ReadRxByte(uint8_t *byte)
{
    uint8_t tail;

    if (byte == 0) {
        return false;
    }
    tail = g_motor_rx_tail;
    if (tail == g_motor_rx_head) {
        return false;
    }
    *byte = g_motor_rx[tail];
    g_motor_rx_tail = TiMotor_NextRxIndex(tail);
    return true;
}

static bool TiMotor_Send(const uint8_t *data, uint8_t length, void *context)
{
    uint32_t timeout;

    (void)context;
    if ((data == 0) || (length == 0U)) {
        return false;
    }
    for (uint8_t i = 0U; i < length; i++) {
        timeout = H2024_UART_TX_TIMEOUT_LOOPS;
        while (DL_UART_isBusy(UART_MOTOR_INST) && (timeout > 0U)) {
            timeout--;
        }
        if (timeout == 0U) {
            g_diagnostics.motor_tx_timeouts++;
            return false;
        }
        DL_UART_Main_transmitData(UART_MOTOR_INST, data[i]);
    }
    timeout = H2024_UART_TX_TIMEOUT_LOOPS;
    while (DL_UART_isBusy(UART_MOTOR_INST) && (timeout > 0U)) {
        timeout--;
    }
    if (timeout == 0U) {
        g_diagnostics.motor_tx_timeouts++;
        return false;
    }
    return true;
}

static uint8_t TiImu_Transfer(uint8_t value, void *context)
{
    uint32_t timeout = H2024_SPI_TIMEOUT_LOOPS;

    (void)context;
    while (DL_SPI_isTXFIFOFull(SPI_IMU_INST) && (timeout > 0U)) {
        timeout--;
    }
    if (timeout == 0U) {
        g_diagnostics.imu_spi_timeouts++;
        return 0xFFU;
    }
    DL_SPI_transmitData8(SPI_IMU_INST, value);
    timeout = H2024_SPI_TIMEOUT_LOOPS;
    while (DL_SPI_isRXFIFOEmpty(SPI_IMU_INST) && (timeout > 0U)) {
        timeout--;
    }
    if (timeout == 0U) {
        g_diagnostics.imu_spi_timeouts++;
        return 0xFFU;
    }
    return DL_SPI_receiveData8(SPI_IMU_INST);
}

static void TiImu_Select(bool active, void *context)
{
    (void)context;
    if (active) {
        while (!DL_SPI_isRXFIFOEmpty(SPI_IMU_INST)) {
            (void)DL_SPI_receiveData8(SPI_IMU_INST);
        }
        DL_GPIO_clearPins(GPIO_IMU_PORT, GPIO_IMU_CS_PIN);
    } else {
        DL_GPIO_setPins(GPIO_IMU_PORT, GPIO_IMU_CS_PIN);
    }
}

static void TiDelayMs(uint32_t delay_ms, void *context)
{
    (void)context;
    while (delay_ms > 0U) {
        DL_Common_delayCycles(CPUCLK_FREQ / 1000U);
        delay_ms--;
    }
}

static void TiDelayUs(uint32_t delay_us, void *context)
{
    (void)context;
    while (delay_us > 0U) {
        DL_Common_delayCycles(CPUCLK_FREQ / 1000000U);
        delay_us--;
    }
}

static bool TiGray_Select(uint8_t channel, void *context)
{
    (void)context;
    if (channel >= GRAY_ARRAY_CHANNELS) {
        return false;
    }
    if ((channel & 0x01U) != 0U) {
        DL_GPIO_setPins(GPIO_GRAY_AD0_PORT, GPIO_GRAY_AD0_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_GRAY_AD0_PORT, GPIO_GRAY_AD0_PIN);
    }
    if ((channel & 0x02U) != 0U) {
        DL_GPIO_setPins(GPIO_GRAY_AD1_PORT, GPIO_GRAY_AD1_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_GRAY_AD1_PORT, GPIO_GRAY_AD1_PIN);
    }
    if ((channel & 0x04U) != 0U) {
        DL_GPIO_setPins(GPIO_GRAY_AD2_PORT, GPIO_GRAY_AD2_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_GRAY_AD2_PORT, GPIO_GRAY_AD2_PIN);
    }
    return true;
}

static bool TiGray_ReadAdc(uint16_t *value, void *context)
{
    uint32_t timeout = H2024_ADC_TIMEOUT_LOOPS;

    (void)context;
    if (value == 0) {
        return false;
    }
    g_adc_ready = false;
    DL_ADC12_startConversion(ADC_GRAY_INST);
    while (!g_adc_ready && (timeout > 0U)) {
        timeout--;
    }
    if (!g_adc_ready) {
        g_diagnostics.gray_adc_timeouts++;
        return false;
    }
    *value = (uint16_t)DL_ADC12_getMemResult(
        ADC_GRAY_INST, ADC_GRAY_ADCMEM_GRAY_OUT);
    return true;
}

static bool TiButton_Read(void *context)
{
    (void)context;
    return TiMspm0Platform_ReadKey1Level();
}

static bool TiGrayCalButton_Read(void *context)
{
    (void)context;
    return TiMspm0Platform_ReadKey3Level();
}

static void TiBuzzer_Set(bool enabled, void *context)
{
    (void)context;
    if (enabled) {
        DL_GPIO_setPins(GPIO_BUZZER_PORT, GPIO_BUZZER_BUZZER_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_BUZZER_PORT, GPIO_BUZZER_BUZZER_PIN);
    }
}

void TiMspm0Platform_OnSysTick(void)
{
    g_millis++;
}

void UART_MOTOR_INST_IRQHandler(void)
{
    (void)DL_UART_Main_getPendingInterrupt(UART_MOTOR_INST);
    while (!DL_UART_Main_isRXFIFOEmpty(UART_MOTOR_INST)) {
        TiMotor_QueueRxByte(DL_UART_Main_receiveData(UART_MOTOR_INST));
    }
}

void ADC_GRAY_INST_IRQHandler(void)
{
    if (DL_ADC12_getPendingInterrupt(ADC_GRAY_INST) ==
        DL_ADC12_IIDX_MEM0_RESULT_LOADED) {
        g_adc_ready = true;
    }
}

void TiMspm0Platform_Init(void)
{
    g_adc_ready = false;
    g_motor_rx_head = 0U;
    g_motor_rx_tail = 0U;
    g_diagnostics = (TiMspm0PlatformDiagnostics){0};
    /* Keep the motor driver electrically disabled during every startup path. */
    DL_TimerA_stopCounter(PWM_TB1_INST);
    DL_GPIO_clearPins(STBY_PORT, STBY_PIN_STBY_PIN);
    DL_GPIO_clearPins(A_PORT, A_PIN_AIN1_PIN | A_PIN_AIN2_PIN);
    DL_GPIO_clearPins(B_PORT, B_PIN_BIN1_PIN | B_PIN_BIN2_PIN);
    DL_GPIO_clearPins(GPIO_GRAY_EN_PORT, GPIO_GRAY_EN_PIN);
    DL_GPIO_setPins(GPIO_IMU_PORT, GPIO_IMU_CS_PIN);
    DL_GPIO_clearPins(GPIO_BUZZER_PORT, GPIO_BUZZER_BUZZER_PIN);
    TB6612_MotorBoardContextInit(
        &g_tb6612_context, TiMspm0Platform_MillisAdapter, 0,
        H2024_TB6612_SPEED_UNITS_AT_MAX_DUTY);
#if H2024_TASK_SPEED_LOOP_ENABLE
    {
        const TB6612SpeedLoopConfig speed_loop = {
            .wheel_diameter_mm = H2024_WHEEL_DIAMETER_MM,
            .left_counts_per_rev =
                (uint32_t)H2024_ENCODER_COUNTS_PER_WHEEL_REV,
            .right_counts_per_rev =
                (uint32_t)H2024_ENCODER_COUNTS_PER_WHEEL_REV,
            .control_period_ms = H2024_TASK_SPEED_LOOP_PERIOD_MS,
            .kp_milli = H2024_TASK_SPEED_KP_MILLI,
            .ki_milli = H2024_TASK_SPEED_KI_MILLI,
            .kd_milli = H2024_TASK_SPEED_KD_MILLI,
            .feedforward_static_milli =
                H2024_TASK_SPEED_FF_STATIC_MILLI,
            .feedforward_rpm_milli = H2024_TASK_SPEED_FF_RPM_MILLI,
            .error_deadband_rpm = H2024_TASK_SPEED_DEADBAND_RPM,
            .output_limit_percent = H2024_TASK_SPEED_LIMIT_PERCENT,
            .motion_watchdog_min_target_rpm =
                H2024_MOTION_WATCHDOG_MIN_TARGET_RPM,
            .motion_watchdog_timeout_ms =
                H2024_MOTION_WATCHDOG_TIMEOUT_MS
        };
        (void)TB6612_MotorBoardConfigureSpeedLoop(&g_tb6612_context,
                                                   &speed_loop);
    }
#endif
    TB6612_Init();
    Encoder_Init();
    while (!DL_UART_Main_isRXFIFOEmpty(UART_MOTOR_INST)) {
        (void)DL_UART_Main_receiveData(UART_MOTOR_INST);
    }
    DL_ADC12_disableConversions(ADC_GRAY_INST);
    DL_ADC12_initSingleSample(
        ADC_GRAY_INST,
        DL_ADC12_REPEAT_MODE_ENABLED,
        DL_ADC12_SAMPLING_SOURCE_AUTO,
        DL_ADC12_TRIG_SRC_SOFTWARE,
        DL_ADC12_SAMP_CONV_RES_12_BIT,
        DL_ADC12_SAMP_CONV_DATA_FORMAT_UNSIGNED);
    DL_ADC12_setSampleTime0(ADC_GRAY_INST, 8U);
    DL_ADC12_clearInterruptStatus(
        ADC_GRAY_INST, DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);
    DL_ADC12_enableConversions(ADC_GRAY_INST);
    NVIC_ClearPendingIRQ(UART_MOTOR_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(ADC_GRAY_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_MOTOR_INST_INT_IRQN);
    NVIC_EnableIRQ(ADC_GRAY_INST_INT_IRQN);
}

uint32_t TiMspm0Platform_Millis(void)
{
    return g_millis;
}

bool TiMspm0Platform_ReadKey1Level(void)
{
    return (DL_GPIO_readPins(GPIO_KEYS_PORT, GPIO_KEYS_KEY1_PIN) != 0U);
}

bool TiMspm0Platform_ReadKey2Level(void)
{
    return (DL_GPIO_readPins(GPIO_KEYS_PORT, GPIO_KEYS_KEY2_PIN) != 0U);
}

bool TiMspm0Platform_ReadKey3Level(void)
{
    return (DL_GPIO_readPins(GPIO_KEYS_PORT, GPIO_KEYS_KEY3_PIN) != 0U);
}

void TiMspm0Platform_ServiceMotorBackend(void)
{
#if H2024_TASK_SPEED_LOOP_ENABLE
    /* Keep the 50 ms wheel loop independent from the 20 ms route update. */
    TB6612_MotorBoardService(&g_tb6612_context);
#endif
}

void TiMspm0Platform_PollMotorRx(CarFirmware *firmware)
{
    uint32_t now_ms = TiMspm0Platform_Millis();
    uint8_t byte;

    while (TiMotor_ReadRxByte(&byte)) {
        CarFirmware_OnMotorRxByte(firmware, byte, now_ms);
    }
}

void TiMspm0Platform_GetDiagnostics(TiMspm0PlatformDiagnostics *diagnostics)
{
    if (diagnostics == 0) {
        return;
    }
    diagnostics->motor_rx_bytes = g_diagnostics.motor_rx_bytes;
    diagnostics->motor_rx_overflows = g_diagnostics.motor_rx_overflows;
    diagnostics->motor_tx_timeouts = g_diagnostics.motor_tx_timeouts;
    diagnostics->imu_spi_timeouts = g_diagnostics.imu_spi_timeouts;
    diagnostics->gray_adc_timeouts = g_diagnostics.gray_adc_timeouts;
}

CarStatus TiMspm0Platform_BuildConfig(CarFirmwareConfig *config,
                                      H2024Mode mode)
{
    if (config == 0) {
        return CAR_ERROR_ARG;
    }
    *config = (CarFirmwareConfig){0};
    config->car = CarConfig_MakeDefault();
    config->car.wheel_diameter_mm = H2024_WHEEL_DIAMETER_MM;
    config->car.track_width_mm = H2024_TRACK_WIDTH_MM;
    config->car.encoder_counts_per_wheel_rev =
        H2024_ENCODER_COUNTS_PER_WHEEL_REV;
    config->car.straight_heading_kp = H2024_STRAIGHT_HEADING_KP;
    config->car.arc_line_kp = H2024_LINE_PID_KP;
    config->car.arc_line_ki = H2024_LINE_PID_KI;
    config->car.arc_line_kd = H2024_LINE_PID_KD;
    config->car.arc_line_integral_limit = H2024_LINE_PID_INTEGRAL_LIMIT;
    config->car.gray_finish_arm_ratio = H2024_FINISH_ARM_RATIO;
    config->car.gray_finish_min_confidence = H2024_FINISH_MIN_CONFIDENCE;
    config->car.gray_finish_min_active = H2024_FINISH_MIN_ACTIVE;
    config->car.gray_finish_consecutive_frames =
        H2024_FINISH_CONSECUTIVE_FRAMES;
    config->car.line_corner_min_position = H2024_LINE_CORNER_MIN_POSITION;
    config->car.line_corner_consecutive_frames =
        H2024_LINE_CORNER_CONSECUTIVE_FRAMES;
    config->car.line_lost_consecutive_frames =
        H2024_LINE_LOST_CONSECUTIVE_FRAMES;
    config->car.line_heading_kp = H2026_LINE_HEADING_KP;
    config->car.line_heading_max_correction_mm_s =
        H2026_LINE_HEADING_MAX_CORRECTION;
    config->car.line_follow_kp = H2026_LINE_FOLLOW_KP;
    config->car.line_follow_ki = H2026_LINE_FOLLOW_KI;
    config->car.line_follow_kd = H2026_LINE_FOLLOW_KD;
    config->car.line_follow_integral_limit =
        H2026_LINE_FOLLOW_INTEGRAL_LIMIT;
    config->car.line_corner_min_right_ratio_permille =
        H2026_CORNER_MIN_RIGHT_RATIO_PERMILLE;
    config->car.line_corner_min_active = H2026_CORNER_MIN_ACTIVE;
    config->car.line_corner_min_span = H2026_CORNER_MIN_SPAN;
    config->car.line_sensor_to_axle_mm = H2026_ARC_SENSOR_TO_AXLE_MM;
    config->car.line_corner_pivot_approach_mm =
        H2026_CORNER_PIVOT_APPROACH_MM;
    config->car.line_corner_approach_speed_mm_s =
        H2026_CORNER_APPROACH_SPEED_MM_S;
    config->car.turn_line_reacquire_min_angle_deg =
        H2026_TURN_REACQUIRE_MIN_DEG;
    config->car.turn_line_reacquire_max_position =
        H2026_TURN_REACQUIRE_MAX_POSITION;
    config->car.turn_line_reacquire_frames =
        H2026_TURN_REACQUIRE_FRAMES;
    if ((mode == H2026_MODE_ITEM_1) ||
        (mode == H2026_MODE_ITEM_2) ||
        (mode == H2026_MODE_ITEM_3) ||
        (mode == H2026_MODE_ITEM_4) || H2026_ModeUsesLine(mode)) {
        /* H2024 keeps legacy line.valid semantics; only H2026 consumes the
         * adaptive narrow/wide/split classification and safe line search. */
        config->car.gray_shape_filter_enable = true;
        config->car.gray_center_offset = H2026_GRAY_CENTER_OFFSET;
        config->car.gray_relative_delta = H2026_GRAY_RELATIVE_DELTA;
        config->car.gray_track_min_confidence =
            H2026_GRAY_TRACK_MIN_CONFIDENCE;
        config->car.gray_track_enter_frames =
            H2026_GRAY_TRACK_ENTER_FRAMES;
        config->car.gray_track_lost_frames =
            H2026_GRAY_TRACK_LOST_FRAMES;
        config->car.gray_track_max_active = H2026_GRAY_TRACK_MAX_ACTIVE;
        config->car.gray_track_max_span = H2026_GRAY_TRACK_MAX_SPAN;
        config->car.gray_wide_min_active = H2026_GRAY_WIDE_MIN_ACTIVE;
        config->car.gray_wide_min_background =
            H2026_GRAY_WIDE_MIN_BACKGROUND;
        config->car.arc_line_entry_min_angle_deg =
            H2026_ARC_LINE_ENTRY_MIN_ANGLE_DEG;
        config->car.arc_line_entry_frames = H2026_ARC_LINE_ENTRY_FRAMES;
        config->car.arc_line_blend_ms = H2026_ARC_LINE_BLEND_MS;
        config->car.diagonal_line_arm_ratio =
            H2026_DIAGONAL_LINE_ARM_RATIO;
        config->car.diagonal_line_approach_mm =
            H2026_DIAGONAL_LINE_APPROACH_MM;
        config->car.line_cross_center_position =
            H2026_LINE_CROSS_CENTER_POSITION;
        config->car.line_cross_capture_window_mm =
            H2026_LINE_CROSS_CAPTURE_WINDOW_MM;
        config->car.line_cross_capture_speed_mm_s =
            H2026_LINE_CROSS_CAPTURE_SPEED_MM_S;
        config->car.line_cross_capture_kp =
            H2026_LINE_CROSS_CAPTURE_KP;
        config->car.line_cross_capture_max_correction_mm_s =
            H2026_LINE_CROSS_CAPTURE_MAX_CORRECTION_MM_S;
        config->car.line_seek_max_correction_mm_s =
            H2026_LINE_SEEK_MAX_CORRECTION_MM_S;
        config->car.turn_inner_speed_ratio = H2026_TURN_INNER_SPEED_RATIO;
        config->car.turn_settle_ms = H2026_TURN_SETTLE_MS;
        config->car.turn_settle_speed_mm_s =
            H2026_TURN_SETTLE_SPEED_MM_S;
        config->car.required_line_search_mm =
            H2026_REQUIRED_LINE_SEARCH_MM;
        config->car.required_line_search_speed_mm_s =
            H2026_REQUIRED_LINE_SEARCH_SPEED_MM_S;
    }
    if ((mode == H2026_MODE_ITEM_2) ||
        (mode == H2026_MODE_ITEM_3) ||
        (mode == H2026_MODE_ITEM_4) ||
        (mode == H2026_MODE_B2) || (mode == H2026_MODE_B5) ||
        (mode == H2026_MODE_B6)) {
        /* All H2026 multi-segment routes share the empirically identified
         * chassis geometry and conservative cascaded-loop limits. */
        config->car.arc_effective_track_width_mm =
            H2026_ARC_EFFECTIVE_TRACK_WIDTH_MM;
        config->car.straight_heading_kp = H2026_STRAIGHT_HEADING_KP;
        config->car.straight_heading_max_correction_mm_s =
            H2026_STRAIGHT_HEADING_MAX_CORR_MM_S;
        config->car.arc_line_max_correction_mm_s =
            H2026_ARC_LINE_MAX_CORR_MM_S;
    }
    if ((mode == H2026_MODE_ITEM_2) ||
        (mode == H2026_MODE_ITEM_3) ||
        (mode == H2026_MODE_ITEM_4)) {
        config->car.arc_line_exit_min_angle_deg =
            H2026_ARC_LINE_EXIT_MIN_ANGLE_DEG;
        config->car.arc_line_exit_lost_frames =
            H2026_ARC_LINE_EXIT_LOST_FRAMES;
    }
    if (H2026_ModeUsesLine(mode)) {
        config->car.finish_sensor_to_test_point_mm =
            H2026_FINISH_SENSOR_TO_TEST_POINT_MM;
        config->car.finish_approach_speed_mm_s =
            H2026_FINISH_APPROACH_SPEED_MM_S;
        config->car.track_wheel_accel_limit_mm_s2 =
            H2026_TRACK_WHEEL_ACCEL_LIMIT_MM_S2;
        /* Official continuous tracks must not consume the old predicted-map
         * 110 degree line-tail handoff. */
        config->car.arc_line_exit_min_angle_deg = 0.0f;
        config->car.arc_line_exit_lost_frames = 0U;
    }
    if (mode == H2026_MODE_B2) {
        config->car.straight_speed_mm_s = H2026_B2_STRAIGHT_SPEED_MM_S;
        config->car.arc_speed_mm_s = H2026_B2_ARC_CENTER_SPEED_MM_S;
    } else if ((mode == H2026_MODE_B4) || (mode == H2026_MODE_B5) ||
               (mode == H2026_MODE_B6)) {
        config->car.straight_speed_mm_s =
            H2026_BALL_TASK_TRACK_SPEED_MM_S;
        config->car.arc_speed_mm_s = H2026_BALL_TASK_TRACK_SPEED_MM_S;
    }
    config->mode = mode;

#if H2024_MOTOR_BACKEND_TB6612
    config->motor = (MotorBoardConfig){0};
    config->motor.left_channel = MOTOR_BOARD_CHANNEL_B;
    config->motor.right_channel = MOTOR_BOARD_CHANNEL_D;
    config->motor.left_inverted = false;
    config->motor.right_inverted = false;
    config->motor.direct_set_wheel_speeds =
        TB6612_MotorBoard_SetWheelSpeeds;
    config->motor.direct_get_encoder = TB6612_MotorBoard_GetEncoder;
    config->motor.direct_context = &g_tb6612_context;
#else
    config->motor = (MotorBoardConfig){
        TiMotor_Send, 0,
        MOTOR_BOARD_CHANNEL_B, MOTOR_BOARD_CHANNEL_D,
        false, true, 5U,
        0, 0, 0
    };//驱动板还是tb6612直接调试的开关
#endif
    config->imu = (Icm42688Port){
        TiImu_Transfer, TiImu_Select, TiDelayMs, 0
    };
    config->gray = (GrayArrayPort){
        TiGray_Select, TiGray_ReadAdc, TiDelayUs, 0,
        H2024_GRAY_SETTLE_US, H2024_GRAY_SAMPLES_PER_CHANNEL
    };
    config->button_read = TiButton_Read;
    config->gray_cal_button_read = TiGrayCalButton_Read;
    config->require_runtime_gray_calibration =
        (mode == H2026_MODE_ITEM_1) || (mode == H2026_MODE_ITEM_2) ||
        (mode == H2026_MODE_ITEM_3) || (mode == H2026_MODE_ITEM_4) ||
        H2026_ModeUsesLine(mode);
    config->buzzer_set = TiBuzzer_Set;
    config->motor_units_per_mm_s = H2024_MOTOR_UNITS_PER_MM_S;
    config->yaw_axis = CAR_IMU_AXIS_Z;
    config->yaw_sign = H2024_IMU_YAW_SIGN;
    config->yaw_bias_dps = H2024_IMU_BIAS_DPS;
    config->yaw_bias_fixed = H2024_IMU_USE_FIXED_BIAS != 0U;
    config->imu_calibration_samples = H2024_IMU_CALIBRATION_SAMPLES;
    config->imu_max_step_ms = 20U;
    config->button_debounce_ms = 20U;
    config->button_active_low = true;

    config->motor_command_spacing_ms = H2024_MOTOR_COMMAND_SPACING_MS;
#if H2024_MOTOR_BACKEND_TB6612
    config->set_encoder_polarity_on_arm = false;
    config->set_speed_pid_on_arm = false;
#else
    config->set_encoder_polarity_on_arm = true;
    config->set_speed_pid_on_arm = true;
#endif
    for (uint8_t channel = 0U; channel < MOTOR_BOARD_CHANNEL_COUNT; channel++) {
        config->encoder_polarity[channel] = true;
        config->speed_pid[channel] = (MotorBoardPid){
            H2024_MOTOR_PID_KP,
            H2024_MOTOR_PID_KI,
            H2024_MOTOR_PID_KD
        };
    }

    /* Measured full-white/full-black values for the current sensor board. */
    {
        static const uint16_t gray_black[GRAY_ARRAY_CHANNELS] = {
            108U, 112U, 113U, 114U, 114U, 114U, 114U, 114U
        };
        static const uint16_t gray_white[GRAY_ARRAY_CHANNELS] = {
            2415U, 1677U, 2310U, 2001U, 1693U, 1885U, 1869U, 1141U
        };

        memcpy(config->gray_black, gray_black, sizeof(gray_black));
        memcpy(config->gray_white, gray_white, sizeof(gray_white));
        config->gray_calibration_valid = true;
    }
    return CAR_OK;
}
