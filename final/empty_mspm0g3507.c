#include "ti_msp_dl_config.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "OLED.h"
#include "firmware.h"
#include "platform/ti_mspm0_platform.h"
#include "project_mode.h"
#include "tb6612.h"
#include "vofa_telemetry.h"

#define H2026_OLED_RENDER_PERIOD_MS (250U)
#define H2026_OLED_TX_PERIOD_MS (5U)
#define H2026_TELEMETRY_PERIOD_MS (100U)

static CarFirmware g_firmware;
static OLED_Status g_oled_status = OLED_STATUS_ERROR_NOT_INITIALIZED;
static bool g_oled_dirty = true;
static uint8_t g_oled_tx_page = OLED_PAGE_COUNT;
static uint32_t g_oled_last_render_ms;
static uint32_t g_oled_last_tx_ms;

static H2026Mode H2026_InitialMode(void)
{
    return (H2026Mode)H2026_INITIAL_TASK_NUMBER;
}

static const char *H2026_TrackName(const CarFirmware *firmware)
{
    return (firmware->config.track_sensor_source ==
            CAR_TRACK_SENSOR_RED_ARRAY) ? "RED" : "GRAY";
}

static const char *H2026_ExternalStateName(H2026TaskState state)
{
    switch (state) {
        case H2026_TASK_RUNNING:
            return "WAIT";
        case H2026_TASK_DONE:
            return "DONE";
        case H2026_TASK_FAULT:
            return "FAULT";
        case H2026_TASK_READY:
        default:
            return "READY";
    }
}

static int32_t H2026_ClampDisplayRpm(int32_t rpm)
{
    if (rpm > 999) {
        return 999;
    }
    if (rpm < -999) {
        return -999;
    }
    return rpm;
}

static void H2026_DrawCalibration(const CarFirmware *firmware)
{
    char line[22];

    switch (firmware->gray_cal_state) {
        case CAR_GRAY_CAL_WAIT_WHITE:
            (void)OLED_ShowString(0U, 1U, "SCAN WHITE");
            break;
        case CAR_GRAY_CAL_CAPTURE_WHITE:
            (void)snprintf(line, sizeof(line), "WHITE %u/%u",
                           (unsigned)firmware->gray_cal_frame_count,
                           (unsigned)CAR_GRAY_CALIBRATION_FRAMES);
            (void)OLED_ShowString(0U, 1U, line);
            break;
        case CAR_GRAY_CAL_WAIT_BLACK:
            (void)OLED_ShowString(0U, 1U, "SCAN BLACK");
            break;
        case CAR_GRAY_CAL_CAPTURE_BLACK:
            (void)snprintf(line, sizeof(line), "BLACK %u/%u",
                           (unsigned)firmware->gray_cal_frame_count,
                           (unsigned)CAR_GRAY_CALIBRATION_FRAMES);
            (void)OLED_ShowString(0U, 1U, line);
            break;
        case CAR_GRAY_CAL_READY:
            (void)OLED_ShowString(0U, 1U, "SCAN OK");
            break;
        case CAR_GRAY_CAL_ERROR:
            (void)snprintf(line, sizeof(line), "SCAN FAIL C%u",
                           (unsigned)(firmware->gray_cal_bad_channel + 1U));
            (void)OLED_ShowString(0U, 1U, line);
            break;
        default:
            (void)OLED_ShowString(0U, 1U, "SCAN ?");
            break;
    }
}

static void H2026_DrawStatus(const CarFirmware *firmware)
{
    char line[22];
    uint32_t faults = firmware->hardware_faults | firmware->output.faults;
    uint32_t limit_ms = H2026_ModeTimeLimitMs(firmware->config.mode);
    TB6612SpeedLoopStatus speed = {0};

    (void)TB6612_DriveGetSpeedLoopStatus(
        (const TB6612Drive *)firmware->config.drive.context, &speed);
    (void)snprintf(line, sizeof(line), "%s %s %s",
                   H2026_TrackName(firmware),
                   H2026_ModeName(firmware->config.mode),
                   H2026_TaskStateName(firmware->app.task_state));
    (void)OLED_ShowString(0U, 0U, line);
    H2026_DrawCalibration(firmware);

    (void)snprintf(line, sizeof(line), "SEG:%s",
                   H2026_RouteSegmentName(firmware->config.mode,
                                          firmware->app.executor.index));
    (void)OLED_ShowString(0U, 2U, line);
    if (limit_ms == 0U) {
        (void)snprintf(line, sizeof(line), "TIME:%lu.%02lu",
                       (unsigned long)(firmware->output.run_time_ms / 1000U),
                       (unsigned long)((firmware->output.run_time_ms / 10U) %
                                       100U));
    } else {
        (void)snprintf(line, sizeof(line), "TIME:%lu.%02lu/%lu",
                       (unsigned long)(firmware->output.run_time_ms / 1000U),
                       (unsigned long)((firmware->output.run_time_ms / 10U) %
                                       100U),
                       (unsigned long)(limit_ms / 1000U));
    }
    (void)OLED_ShowString(0U, 3U, line);

    if (H2026_ModeWaitsForExternalCompletion(firmware->config.mode)) {
        (void)snprintf(line, sizeof(line), "%s:%s",
                       (firmware->config.mode == H2026_MODE_B1) ?
                           "VIDEO" : "BALL",
                       H2026_ExternalStateName(firmware->app.task_state));
    } else {
        (void)snprintf(line, sizeof(line), "LINE:%c E:%+5d",
                       firmware->app.line.valid ? 'Y' : 'N',
                       (int)firmware->app.line.track_position);
    }
    (void)OLED_ShowString(0U, 4U, line);

    (void)snprintf(line, sizeof(line), "RPM:%+4ld/%+4ld",
                   (long)H2026_ClampDisplayRpm(speed.measured_left_rpm),
                   (long)H2026_ClampDisplayRpm(speed.measured_right_rpm));
    (void)OLED_ShowString(0U, 5U, line);
    (void)snprintf(line, sizeof(line), "SW:%lu REJ:%lu",
                   (unsigned long)firmware->task_switch_count,
                   (unsigned long)firmware->task_switch_reject_count);
    (void)OLED_ShowString(0U, 6U, line);
    (void)snprintf(line, sizeof(line), "F:%08lX",
                   (unsigned long)faults);
    (void)OLED_ShowString(0U, 7U, line);
}

static void H2026_InitOled(void)
{
    OLED_Config config = OLED_MakeSSD1306Config(OLED_DEFAULT_ADDR_7BIT);

    g_oled_status = OLED_Init(&config);
    if (g_oled_status == OLED_STATUS_ERROR_I2C_ADDRESS_NACK) {
        config = OLED_MakeSSD1306Config(0x3DU);
        g_oled_status = OLED_Init(&config);
    }
    g_oled_dirty = g_oled_status == OLED_STATUS_OK;
}

static void H2026_ServiceOled(const CarFirmware *firmware, uint32_t now_ms)
{
    if ((firmware == 0) || !OLED_IsInitialized()) {
        return;
    }
    if ((g_oled_tx_page >= OLED_PAGE_COUNT) &&
        (g_oled_dirty ||
         ((uint32_t)(now_ms - g_oled_last_render_ms) >=
          H2026_OLED_RENDER_PERIOD_MS))) {
        (void)OLED_Clear();
        H2026_DrawStatus(firmware);
        g_oled_dirty = false;
        g_oled_last_render_ms = now_ms;
        g_oled_tx_page = 0U;
    }
    if ((g_oled_tx_page >= OLED_PAGE_COUNT) ||
        ((uint32_t)(now_ms - g_oled_last_tx_ms) <
         H2026_OLED_TX_PERIOD_MS)) {
        return;
    }
    g_oled_last_tx_ms = now_ms;
    g_oled_status = OLED_UpdatePages(g_oled_tx_page, 1U);
    if (g_oled_status == OLED_STATUS_OK) {
        g_oled_tx_page++;
    } else {
        g_oled_tx_page = OLED_PAGE_COUNT;
    }
}

static void H2026_RunFirmware(void)
{
    CarFirmwareConfig config;
    CarStatus status;
    uint32_t last_telemetry_ms = 0U;
    H2026Mode last_mode = (H2026Mode)0;
    H2026TaskState last_task_state = (H2026TaskState)UINT8_MAX;
    CarGrayCalibrationState last_cal_state =
        (CarGrayCalibrationState)UINT8_MAX;
    uint16_t last_route_index = UINT16_MAX;
    CarTrackPhase last_track_phase = (CarTrackPhase)UINT8_MAX;
    CarRouteExitReason last_exit_reason =
        (CarRouteExitReason)UINT8_MAX;
    uint32_t last_faults = UINT32_MAX;

    TiMspm0Platform_Init();
    H2026_InitOled();
    status = TiMspm0Platform_BuildConfig(&config, H2026_InitialMode());
    if (status == CAR_OK) {
        status = CarFirmware_Init(&g_firmware, &config,
                                  TiMspm0Platform_Millis());
    }
    if (status != CAR_OK) {
        TB6612_Stop();
        while (1) {
            DL_WWDT_restart(WWDT0_INST);
            __WFI();
        }
    }

    VofaTelemetry_Init();
    NVIC_ClearPendingIRQ(UART_VOFA_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_VOFA_INST_INT_IRQN);
    VofaTelemetry_SendBanner();

    while (1) {
        uint32_t now_ms = TiMspm0Platform_Millis();
        uint32_t faults;
        bool command_event;
        bool state_event;

        command_event = VofaTelemetry_ProcessCommands(&g_firmware, now_ms);
        CarFirmware_Tick(&g_firmware, now_ms);
        faults = g_firmware.hardware_faults | g_firmware.output.faults;
        state_event =
            (g_firmware.config.mode != last_mode) ||
            (g_firmware.app.task_state != last_task_state) ||
            (g_firmware.gray_cal_state != last_cal_state) ||
            (g_firmware.app.executor.index != last_route_index) ||
            (g_firmware.app.executor.track_phase != last_track_phase) ||
            (g_firmware.app.executor.last_exit_reason != last_exit_reason) ||
            (faults != last_faults);
        if (state_event) {
            g_oled_dirty = true;
        }
        if (command_event || state_event ||
            ((uint32_t)(now_ms - last_telemetry_ms) >=
             H2026_TELEMETRY_PERIOD_MS)) {
            VofaTelemetry_SendFrame(&g_firmware, now_ms);
            last_telemetry_ms = now_ms;
            last_mode = g_firmware.config.mode;
            last_task_state = g_firmware.app.task_state;
            last_cal_state = g_firmware.gray_cal_state;
            last_route_index = g_firmware.app.executor.index;
            last_track_phase = g_firmware.app.executor.track_phase;
            last_exit_reason = g_firmware.app.executor.last_exit_reason;
            last_faults = faults;
        }
        H2026_ServiceOled(&g_firmware, now_ms);
        DL_WWDT_restart(WWDT0_INST);
        __WFI();
    }
}

void UART_VOFA_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_VOFA_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            while (!DL_UART_Main_isRXFIFOEmpty(UART_VOFA_INST)) {
                VofaTelemetry_PushRxFromIsr(
                    (uint8_t)DL_UART_Main_receiveData(UART_VOFA_INST));
            }
            break;
        case DL_UART_MAIN_IIDX_TX:
            VofaTelemetry_TxIrqHandler();
            break;
        default:
            break;
    }
}

int main(void)
{
    SYSCFG_DL_init();
    DL_WWDT_setCoreHaltBehavior(WWDT0_INST, DL_WWDT_CORE_HALT_STOP);
    DL_SYSTICK_config(CPUCLK_FREQ / 1000U);
    H2026_RunFirmware();
    return 0;
}

void SysTick_Handler(void)
{
    TiMspm0Platform_OnSysTick();
}
