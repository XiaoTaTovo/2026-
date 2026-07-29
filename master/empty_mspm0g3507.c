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
static bool g_oled_refresh_requested = true;

static H2026Mode H2026_SelectedMode(void)
{
#if PROJECT_MODE == PROJECT_MODE_H2026_B2
    return H2026_MODE_B2;
#elif PROJECT_MODE == PROJECT_MODE_H2026_B3
    return H2026_MODE_B3;
#elif PROJECT_MODE == PROJECT_MODE_H2026_B4
    return H2026_MODE_B4;
#elif PROJECT_MODE == PROJECT_MODE_H2026_B5
    return H2026_MODE_B5;
#elif PROJECT_MODE == PROJECT_MODE_H2026_B6
    return H2026_MODE_B6;
#else
#error "Unsupported 2026 H project mode"
#endif
}

static const char *H2026_ButtonActionName(CarButtonAction action)
{
    switch (action) {
        case CAR_BUTTON_ACTION_ARM_OK:
            return "ARM";
        case CAR_BUTTON_ACTION_ARM_REJECTED:
            return "REJ";
        case CAR_BUTTON_ACTION_GRAY_REQUIRED:
            return "GRAY";
        case CAR_BUTTON_ACTION_EMERGENCY_STOP:
            return "STOP";
        case CAR_BUTTON_ACTION_NONE:
        default:
            return "NONE";
    }
}

static const char *H2026_GrayCalibrationName(
    CarGrayCalibrationState state)
{
    switch (state) {
        case CAR_GRAY_CAL_WAIT_WHITE:
            return "WHITE";
        case CAR_GRAY_CAL_CAPTURE_WHITE:
            return "CAPW";
        case CAR_GRAY_CAL_WAIT_BLACK:
            return "BLACK";
        case CAR_GRAY_CAL_CAPTURE_BLACK:
            return "CAPB";
        case CAR_GRAY_CAL_READY:
            return "OK";
        case CAR_GRAY_CAL_ERROR:
            return "ERR";
        default:
            return "?";
    }
}

static const char *H2026_RouteExitName(CarRouteExitReason reason)
{
    switch (reason) {
        case CAR_ROUTE_EXIT_DISTANCE:
            return "DIST";
        case CAR_ROUTE_EXIT_FINISH_MARKER:
            return "MARK";
        case CAR_ROUTE_EXIT_LINE_MISS:
            return "MISS";
        case CAR_ROUTE_EXIT_TIMEOUT:
            return "TIME";
        case CAR_ROUTE_EXIT_NONE:
        default:
            return "-";
    }
}

static int32_t H2026_ToTenths(float value)
{
    float scaled = value * 10.0f;

    return (int32_t)(scaled + ((scaled >= 0.0f) ? 0.5f : -0.5f));
}

static void H2026_ShowFixed1(uint8_t page,
                             const char *prefix,
                             float value)
{
    char line[22];
    int32_t scaled = H2026_ToTenths(value);
    uint32_t magnitude;
    char sign = (scaled < 0) ? '-' : '+';

    if (scaled < 0) {
        magnitude = (uint32_t)(-(scaled + 1)) + 1U;
    } else {
        magnitude = (uint32_t)scaled;
    }
    (void)snprintf(line, sizeof(line), "%s%c%lu.%lu", prefix, sign,
                   (unsigned long)(magnitude / 10U),
                   (unsigned long)(magnitude % 10U));
    (void)OLED_ShowString(0U, page, line);
}

static void H2026_InitOled(void)
{
    OLED_Config config = OLED_MakeSSD1306Config(OLED_DEFAULT_ADDR_7BIT);

    g_oled_status = OLED_Init(&config);
    if (g_oled_status == OLED_STATUS_ERROR_I2C_ADDRESS_NACK) {
        config = OLED_MakeSSD1306Config(0x3DU);
        g_oled_status = OLED_Init(&config);
    }
}

static void H2026_RefreshOled(const CarFirmware *firmware,
                              uint32_t now_ms)
{
    static uint32_t last_render_ms;
    static uint32_t last_tx_ms;
    static uint8_t tx_page = OLED_PAGE_COUNT;
    char line[22];
    uint32_t faults;
    const char *run_state;

    if ((firmware == 0) || !OLED_IsInitialized() ||
        (g_oled_status != OLED_STATUS_OK)) {
        return;
    }

    if (g_oled_refresh_requested ||
        ((tx_page >= OLED_PAGE_COUNT) &&
         ((uint32_t)(now_ms - last_render_ms) >=
          H2026_OLED_RENDER_PERIOD_MS))) {
        last_render_ms = now_ms;
        g_oled_refresh_requested = false;
        tx_page = 0U;
        faults = firmware->hardware_faults | firmware->output.faults;
        run_state = firmware->app.executor.finished ? "DONE" :
                    (firmware->app.executor.running ? "RUN" : "STOP");

        (void)OLED_Clear();
        (void)snprintf(line, sizeof(line), "H26%s %s T:%lu.%02lu",
                       H2026_ModeName(firmware->config.mode), run_state,
                       (unsigned long)(firmware->output.run_time_ms / 1000U),
                       (unsigned long)((firmware->output.run_time_ms / 10U) %
                                       100U));
        (void)OLED_ShowString(0U, 0U, line);
        (void)snprintf(line, sizeof(line), "IMU:%s GC:%s",
                       firmware->yaw.calibrated ? "OK" : "WAIT",
                       H2026_GrayCalibrationName(firmware->gray_cal_state));
        (void)OLED_ShowString(0U, 1U, line);
        H2026_ShowFixed1(2U, "YAW:", firmware->imu_sample.yaw_deg);
        (void)snprintf(line, sizeof(line), "SEG:%02u P:%u X:%s",
                       (unsigned)firmware->app.executor.index,
                       (unsigned)firmware->app.executor.track_phase,
                       H2026_RouteExitName(
                           firmware->app.executor.last_exit_reason));
        (void)OLED_ShowString(0U, 3U, line);
        (void)snprintf(line, sizeof(line), "K:%u%u%u B:%lu %s",
                       TiMspm0Platform_ReadKey1Level() ? 1U : 0U,
                       TiMspm0Platform_ReadKey2Level() ? 1U : 0U,
                       TiMspm0Platform_ReadKey3Level() ? 1U : 0U,
                       (unsigned long)firmware->button_event_count,
                       H2026_ButtonActionName(firmware->last_button_action));
        (void)OLED_ShowString(0U, 4U, line);
        if ((firmware->gray_cal_state == CAR_GRAY_CAL_CAPTURE_WHITE) ||
            (firmware->gray_cal_state == CAR_GRAY_CAL_CAPTURE_BLACK)) {
            (void)snprintf(line, sizeof(line), "GC:%s %u/%u",
                           H2026_GrayCalibrationName(
                               firmware->gray_cal_state),
                           (unsigned)firmware->gray_cal_frame_count,
                           (unsigned)CAR_GRAY_CALIBRATION_FRAMES);
        } else if (firmware->gray_cal_state == CAR_GRAY_CAL_ERROR) {
            (void)snprintf(line, sizeof(line), "GC:ERR C%u D%ld",
                           (unsigned)(firmware->gray_cal_bad_channel + 1U),
                           (long)firmware->gray_cal_bad_span);
        } else {
            (void)snprintf(line, sizeof(line), "LN:%c C:%u A:%u M:%02X",
                           firmware->app.line.valid ? 'Y' : 'N',
                           (unsigned)firmware->app.line.track_confidence,
                           (unsigned)firmware->app.line.track_active_count,
                           (unsigned)firmware->app.line.track_active_mask);
        }
        (void)OLED_ShowString(0U, 5U, line);
        (void)snprintf(line, sizeof(line), "PWM:%d/%d",
                       TB6612_GetLeftCommand(), TB6612_GetRightCommand());
        (void)OLED_ShowString(0U, 6U, line);
        (void)snprintf(line, sizeof(line), "F:%08lX",
                       (unsigned long)faults);
        (void)OLED_ShowString(0U, 7U, line);
    }

    if ((tx_page >= OLED_PAGE_COUNT) ||
        ((uint32_t)(now_ms - last_tx_ms) < H2026_OLED_TX_PERIOD_MS)) {
        return;
    }
    last_tx_ms = now_ms;
    if (OLED_UpdatePages(tx_page, 1U) != OLED_STATUS_OK) {
        g_oled_status = OLED_STATUS_ERROR_I2C_BUS;
        return;
    }
    tx_page++;
}

static void H2026_RunFirmware(void)
{
    CarFirmwareConfig config;
    CarStatus status;
    uint32_t last_telemetry_ms = 0U;
    uint16_t last_route_index = UINT16_MAX;
    CarTrackPhase last_track_phase = (CarTrackPhase)UINT8_MAX;
    CarRouteExitReason last_exit_reason =
        (CarRouteExitReason)UINT8_MAX;
    uint32_t last_faults = UINT32_MAX;

    TiMspm0Platform_Init();
    H2026_InitOled();
    status = TiMspm0Platform_BuildConfig(&config, H2026_SelectedMode());
    if (status == CAR_OK) {
        status = CarFirmware_Init(&g_firmware, &config,
                                  TiMspm0Platform_Millis());
    }
    if (status != CAR_OK) {
        while (1) {
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
        bool telemetry_event;

        command_event = VofaTelemetry_ProcessCommands(&g_firmware, now_ms);
        CarFirmware_Tick(&g_firmware, now_ms);
        faults = g_firmware.hardware_faults | g_firmware.output.faults;
        telemetry_event =
            command_event ||
            (g_firmware.app.executor.index != last_route_index) ||
            (g_firmware.app.executor.track_phase != last_track_phase) ||
            (g_firmware.app.executor.last_exit_reason != last_exit_reason) ||
            (faults != last_faults);
        if (telemetry_event ||
            ((uint32_t)(now_ms - last_telemetry_ms) >=
             H2026_TELEMETRY_PERIOD_MS)) {
            VofaTelemetry_SendFrame(&g_firmware, now_ms);
            last_telemetry_ms = now_ms;
            last_route_index = g_firmware.app.executor.index;
            last_track_phase = g_firmware.app.executor.track_phase;
            last_exit_reason = g_firmware.app.executor.last_exit_reason;
            last_faults = faults;
        }
        H2026_RefreshOled(&g_firmware, now_ms);
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
                    (uint8_t)DL_UART_Main_receiveData(
                        UART_VOFA_INST));
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
