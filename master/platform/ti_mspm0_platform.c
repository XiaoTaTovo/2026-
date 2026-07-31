#include "platform/ti_mspm0_platform.h"

#include <string.h>

#include "encoder.h"
#include "platform/ti_mspm0_platform_config.h"
#include "tb6612.h"
#include "ti_msp_dl_config.h"

static volatile uint32_t g_millis;
static volatile bool g_adc_ready;
static TiMspm0PlatformDiagnostics g_diagnostics;
static TB6612Drive g_drive;
static bool g_drive_ready;

static uint32_t TiDrive_Millis(void *context)
{
    (void)context;
    return TiMspm0Platform_Millis();
}

static void TiDrive_Stop(void *context)
{
    (void)TB6612_DriveSetWheelSpeeds(0, 0, context);
}

static void TiDrive_Service(void *context)
{
    TB6612_DriveService((TB6612Drive *)context);
}

static bool TiImu_Transfer(uint8_t value,
                           uint8_t *received,
                           void *context)
{
    uint32_t timeout = H2026_SPI_TIMEOUT_LOOPS;

    (void)context;
    if (received == 0) {
        return false;
    }
    while (DL_SPI_isTXFIFOFull(SPI_IMU_INST) && (timeout > 0U)) {
        timeout--;
    }
    if (timeout == 0U) {
        g_diagnostics.imu_spi_timeouts++;
        return false;
    }
    DL_SPI_transmitData8(SPI_IMU_INST, value);
    timeout = H2026_SPI_TIMEOUT_LOOPS;
    while (DL_SPI_isRXFIFOEmpty(SPI_IMU_INST) && (timeout > 0U)) {
        timeout--;
    }
    if (timeout == 0U) {
        g_diagnostics.imu_spi_timeouts++;
        return false;
    }
    *received = DL_SPI_receiveData8(SPI_IMU_INST);
    return true;
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
    uint32_t timeout = H2026_ADC_TIMEOUT_LOOPS;
    bool completed_by_poll = false;

    (void)context;
    if (value == 0) {
        return false;
    }
    g_adc_ready = false;
    /* ADC0 also owns RED slots; select only GRAY MEM0 for this read. */
    DL_ADC12_setStartAddress(ADC_GRAY_INST, DL_ADC12_SEQ_START_ADDR_00);
    DL_ADC12_setEndAddress(ADC_GRAY_INST, DL_ADC12_SEQ_END_ADDR_00);
    DL_ADC12_clearInterruptStatus(
        ADC_GRAY_INST, DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);
    /* TI SDK adc12_single_conversion: re-arm ENC for every single sample. */
    DL_ADC12_enableConversions(ADC_GRAY_INST);
    DL_ADC12_startConversion(ADC_GRAY_INST);
    while (!g_adc_ready && (timeout > 0U)) {
        if (DL_ADC12_getRawInterruptStatus(
                ADC_GRAY_INST,
                DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED) != 0U) {
            completed_by_poll = true;
            g_adc_ready = true;
            break;
        }
        timeout--;
    }
    DL_ADC12_stopConversion(ADC_GRAY_INST);
    if (!g_adc_ready) {
        DL_ADC12_clearInterruptStatus(
            ADC_GRAY_INST, DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);
        NVIC_ClearPendingIRQ(ADC_GRAY_INST_INT_IRQN);
        g_diagnostics.gray_adc_timeouts++;
        return false;
    }
    if (completed_by_poll) {
        g_diagnostics.gray_adc_poll_completions++;
    } else {
        g_diagnostics.gray_adc_isr_completions++;
    }
    *value = (uint16_t)DL_ADC12_getMemResult(
        ADC_GRAY_INST, ADC_GRAY_ADCMEM_GRAY_OUT);
    DL_ADC12_clearInterruptStatus(
        ADC_GRAY_INST, DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);
    NVIC_ClearPendingIRQ(ADC_GRAY_INST_INT_IRQN);
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
    return TiMspm0Platform_ReadKey2Level();
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

void ADC_GRAY_INST_IRQHandler(void)
{
    if (DL_ADC12_getPendingInterrupt(ADC_GRAY_INST) ==
        DL_ADC12_IIDX_MEM0_RESULT_LOADED) {
        g_adc_ready = true;
    }
}

void TiMspm0Platform_Init(void)
{
    const TB6612SpeedLoopConfig speed_loop = {
        .wheel_diameter_mm = H2026_WHEEL_DIAMETER_MM,
        .left_counts_per_rev = H2026_ENCODER_COUNTS_PER_WHEEL_REV,
        .right_counts_per_rev = H2026_ENCODER_COUNTS_PER_WHEEL_REV,
        .control_period_ms = H2026_SPEED_LOOP_PERIOD_MS,
        .kp_milli = H2026_SPEED_KP_MILLI,
        .ki_milli = H2026_SPEED_KI_MILLI,
        .kd_milli = H2026_SPEED_KD_MILLI,
        .feedforward_static_milli = H2026_SPEED_FF_STATIC_MILLI,
        .feedforward_rpm_milli = H2026_SPEED_FF_RPM_MILLI,
        .error_deadband_rpm = H2026_SPEED_DEADBAND_RPM,
        .output_limit_percent = H2026_SPEED_LIMIT_PERCENT,
        .motion_watchdog_min_target_rpm =
            H2026_MOTION_WATCHDOG_MIN_TARGET_RPM,
        .motion_watchdog_timeout_ms =
            H2026_MOTION_WATCHDOG_TIMEOUT_MS
    };

    g_adc_ready = false;
    g_diagnostics = (TiMspm0PlatformDiagnostics){0};
    g_drive_ready = false;

    /* Electrical safe state before any sensor or control initialization. */
    DL_TimerA_stopCounter(PWM_TB1_INST);
    DL_GPIO_clearPins(STBY_PORT, STBY_PIN_STBY_PIN);
    DL_GPIO_clearPins(A_PORT, A_PIN_AIN1_PIN | A_PIN_AIN2_PIN);
    DL_GPIO_clearPins(B_PORT, B_PIN_BIN1_PIN | B_PIN_BIN2_PIN);
    DL_GPIO_clearPins(GPIO_GRAY_EN_PORT, GPIO_GRAY_EN_PIN);
    DL_GPIO_setPins(GPIO_IMU_PORT, GPIO_IMU_CS_PIN);
    DL_GPIO_clearPins(GPIO_BUZZER_PORT, GPIO_BUZZER_BUZZER_PIN);

    TB6612_DriveInit(&g_drive, TiDrive_Millis, 0);
    g_drive_ready = TB6612_DriveConfigureSpeedLoop(&g_drive, &speed_loop);
    TB6612_Init();
    Encoder_Init();

    /* SysConfig owns the GRAY ADC input and its pinmux. */
    DL_ADC12_clearInterruptStatus(
        ADC_GRAY_INST, DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);
    DL_ADC12_enableInterrupt(
        ADC_GRAY_INST, DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);
    DL_ADC12_enableConversions(ADC_GRAY_INST);
    NVIC_ClearPendingIRQ(ADC_GRAY_INST_INT_IRQN);
    NVIC_EnableIRQ(ADC_GRAY_INST_INT_IRQN);
}

uint32_t TiMspm0Platform_Millis(void)
{
    return g_millis;
}

bool TiMspm0Platform_ReadKey1Level(void)
{
    return DL_GPIO_readPins(GPIO_KEYS_PORT, GPIO_KEYS_KEY1_PIN) != 0U;
}

bool TiMspm0Platform_ReadKey2Level(void)
{
    return DL_GPIO_readPins(GPIO_KEYS_PORT, GPIO_KEYS_KEY2_PIN) != 0U;
}

bool TiMspm0Platform_ReadKey3Level(void)
{
    return DL_GPIO_readPins(GPIO_KEYS_PORT, GPIO_KEYS_KEY3_PIN) != 0U;
}

void TiMspm0Platform_GetDiagnostics(TiMspm0PlatformDiagnostics *diagnostics)
{
    if (diagnostics != 0) {
        *diagnostics = g_diagnostics;
    }
}

CarStatus TiMspm0Platform_BuildConfig(CarFirmwareConfig *config,
                                      H2026Mode mode)
{
    if ((config == 0) || !g_drive_ready ||
        !H2026_ModeIsOfficial(mode)) {
        return CAR_ERROR_ARG;
    }
    *config = (CarFirmwareConfig){0};
    config->car = CarConfig_MakeDefault();
    config->car.wheel_diameter_mm = H2026_WHEEL_DIAMETER_MM;
    config->car.track_width_mm = H2026_TRACK_WIDTH_MM;
    config->car.encoder_counts_per_wheel_rev =
        H2026_ENCODER_COUNTS_PER_WHEEL_REV;
    config->car.arc_effective_track_width_mm =
        H2026_ARC_EFFECTIVE_TRACK_WIDTH_MM;
    config->car.max_wheel_speed_mm_s = H2026_MAX_WHEEL_SPEED_MM_S;
    config->car.track_wheel_accel_limit_mm_s2 =
        H2026_TRACK_WHEEL_ACCEL_LIMIT_MM_S2;
    config->car.distance_tolerance_mm = H2026_DISTANCE_TOLERANCE_MM;
    config->car.straight_timeout_ms = H2026_STRAIGHT_TIMEOUT_MS;
    config->car.arc_timeout_ms = H2026_ARC_TIMEOUT_MS;

    config->car.line_kp = H2026_LINE_KP;
    config->car.line_ki = H2026_LINE_KI;
    config->car.line_kd = H2026_LINE_KD;
    config->car.line_derivative_filter_tau_s =
        H2026_LINE_D_FILTER_TAU_S;
    config->car.line_integral_limit = H2026_LINE_INTEGRAL_LIMIT;
    config->car.straight_line_max_correction_mm_s =
        H2026_STRAIGHT_LINE_MAX_CORRECTION_MM_S;
    config->car.arc_line_max_correction_mm_s =
        H2026_ARC_LINE_MAX_CORRECTION_MM_S;

    config->car.gray_min_signal = H2026_GRAY_MIN_SIGNAL;
    config->car.gray_min_confidence = H2026_GRAY_MIN_CONFIDENCE;
    config->car.gray_center_offset = H2026_GRAY_CENTER_OFFSET;
    config->car.gray_relative_delta = H2026_GRAY_RELATIVE_DELTA;
    config->car.gray_track_min_confidence =
        H2026_GRAY_TRACK_MIN_CONFIDENCE;
    config->car.gray_track_enter_frames = H2026_GRAY_TRACK_ENTER_FRAMES;
    config->car.gray_track_lost_frames = H2026_GRAY_TRACK_LOST_FRAMES;
    config->car.gray_track_max_active = H2026_GRAY_TRACK_MAX_ACTIVE;
    config->car.gray_track_max_span = H2026_GRAY_TRACK_MAX_SPAN;
    config->car.gray_wide_min_active = H2026_GRAY_WIDE_MIN_ACTIVE;
    config->car.gray_wide_min_background =
        H2026_GRAY_WIDE_MIN_BACKGROUND;
    config->car.gray_finish_min_confidence = H2026_FINISH_MIN_CONFIDENCE;
    config->car.gray_finish_min_active_mean =
        H2026_FINISH_MIN_ACTIVE_MEAN;
    config->car.gray_finish_min_active = H2026_FINISH_MIN_ACTIVE;
    config->car.gray_finish_consecutive_frames =
        H2026_FINISH_CONSECUTIVE_FRAMES;
    config->car.required_line_search_mm = H2026_REQUIRED_LINE_SEARCH_MM;
    config->car.required_line_search_speed_mm_s =
        H2026_REQUIRED_LINE_SEARCH_SPEED_MM_S;
    config->car.finish_sensor_to_test_point_mm =
        H2026_FINISH_SENSOR_TO_TEST_POINT_MM;
    config->car.finish_approach_speed_mm_s =
        H2026_FINISH_APPROACH_SPEED_MM_S;

    if (mode == H2026_MODE_B2) {
        config->car.straight_speed_mm_s = H2026_B2_STRAIGHT_SPEED_MM_S;
        config->car.arc_speed_mm_s = H2026_B2_ARC_CENTER_SPEED_MM_S;
    } else {
        config->car.straight_speed_mm_s =
            H2026_BALL_TASK_TRACK_SPEED_MM_S;
        config->car.arc_speed_mm_s = H2026_BALL_TASK_TRACK_SPEED_MM_S;
    }

    config->mode = mode;
    config->drive = (CarDrivePort){
        TB6612_DriveSetWheelSpeeds,
        TB6612_DriveReadEncoder,
        TiDrive_Stop,
        TiDrive_Service,
        &g_drive
    };
    config->imu = (Icm42688Port){
        TiImu_Transfer, TiImu_Select, TiDelayMs, 0
    };
    config->gray = (GrayArrayPort){
        TiGray_Select, TiGray_ReadAdc, TiDelayUs, 0,
        H2026_GRAY_SETTLE_US, H2026_GRAY_SAMPLES_PER_CHANNEL
    };
    config->button_read = TiButton_Read;
    config->gray_cal_button_read = TiGrayCalButton_Read;
    config->require_runtime_gray_calibration = H2026_ModeUsesLine(mode);
    config->buzzer_set = TiBuzzer_Set;
    config->yaw_axis = CAR_IMU_AXIS_Z;
    config->yaw_sign = H2026_IMU_YAW_SIGN;
    config->yaw_bias_dps = H2026_IMU_BIAS_DPS;
    config->yaw_bias_fixed = H2026_IMU_USE_FIXED_BIAS != 0U;
    config->imu_calibration_samples = H2026_IMU_CALIBRATION_SAMPLES;
    config->imu_max_step_ms = 20U;
    config->accel_axis = (ChassisFeedforwardAxis)H2026_IMU_ACCEL_AXIS;
    config->accel_sign = H2026_IMU_ACCEL_SIGN;
    config->accel_lsb_per_g = H2026_IMU_ACCEL_LSB_PER_G;
    config->accel_calibration_samples =
        H2026_IMU_ACCEL_CALIBRATION_SAMPLES;
    config->accel_filter_alpha = H2026_IMU_ACCEL_FILTER_ALPHA;
    config->button_debounce_ms = 8U;
    config->button_active_low = true;

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
