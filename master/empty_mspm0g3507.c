// #include "ti_msp_dl_config.h"

// #include <stdbool.h>
// #include <stdint.h>
// #include <stdio.h>

// #include "OLED.h"
// #include "firmware.h"
// #include "platform/ti_mspm0_platform.h"
// #include "project_mode.h"
// #include "tb6612.h"
// #include "vofa_telemetry.h"

// #define H2026_OLED_RENDER_PERIOD_MS (250U)
// #define H2026_OLED_TX_PERIOD_MS (5U)
// #define H2026_TELEMETRY_PERIOD_MS (100U)

// static CarFirmware g_firmware;
// static OLED_Status g_oled_status = OLED_STATUS_ERROR_NOT_INITIALIZED;
// static bool g_oled_refresh_requested = true;

// static H2026Mode H2026_SelectedMode(void)
// {
// #if PROJECT_MODE == PROJECT_MODE_H2026_B2
//     return H2026_MODE_B2;
// #elif PROJECT_MODE == PROJECT_MODE_H2026_B3
//     return H2026_MODE_B3;
// #elif PROJECT_MODE == PROJECT_MODE_H2026_B4
//     return H2026_MODE_B4;
// #elif PROJECT_MODE == PROJECT_MODE_H2026_B5
//     return H2026_MODE_B5;
// #elif PROJECT_MODE == PROJECT_MODE_H2026_B6
//     return H2026_MODE_B6;
// #else
// #error "Unsupported 2026 H project mode"
// #endif
// }

// static const char *H2026_ButtonActionName(CarButtonAction action)
// {
//     switch (action) {
//         case CAR_BUTTON_ACTION_ARM_OK:
//             return "ARM";
//         case CAR_BUTTON_ACTION_ARM_REJECTED:
//             return "REJ";
//         case CAR_BUTTON_ACTION_GRAY_REQUIRED:
//             return "GRAY";
//         case CAR_BUTTON_ACTION_EMERGENCY_STOP:
//             return "STOP";
//         case CAR_BUTTON_ACTION_NONE:
//         default:
//             return "NONE";
//     }
// }

// static const char *H2026_GrayCalibrationName(
//     CarGrayCalibrationState state)
// {
//     switch (state) {
//         case CAR_GRAY_CAL_WAIT_WHITE:
//             return "WHITE";
//         case CAR_GRAY_CAL_CAPTURE_WHITE:
//             return "CAPW";
//         case CAR_GRAY_CAL_WAIT_BLACK:
//             return "BLACK";
//         case CAR_GRAY_CAL_CAPTURE_BLACK:
//             return "CAPB";
//         case CAR_GRAY_CAL_READY:
//             return "OK";
//         case CAR_GRAY_CAL_ERROR:
//             return "ERR";
//         default:
//             return "?";
//     }
// }

// static const char *H2026_RouteExitName(CarRouteExitReason reason)
// {
//     switch (reason) {
//         case CAR_ROUTE_EXIT_DISTANCE:
//             return "DIST";
//         case CAR_ROUTE_EXIT_FINISH_MARKER:
//             return "MARK";
//         case CAR_ROUTE_EXIT_LINE_MISS:
//             return "MISS";
//         case CAR_ROUTE_EXIT_TIMEOUT:
//             return "TIME";
//         case CAR_ROUTE_EXIT_NONE:
//         default:
//             return "-";
//     }
// }

// static int32_t H2026_ToTenths(float value)
// {
//     float scaled = value * 10.0f;

//     return (int32_t)(scaled + ((scaled >= 0.0f) ? 0.5f : -0.5f));
// }

// static void H2026_ShowFixed1(uint8_t page,
//                              const char *prefix,
//                              float value)
// {
//     char line[22];
//     int32_t scaled = H2026_ToTenths(value);
//     uint32_t magnitude;
//     char sign = (scaled < 0) ? '-' : '+';

//     if (scaled < 0) {
//         magnitude = (uint32_t)(-(scaled + 1)) + 1U;
//     } else {
//         magnitude = (uint32_t)scaled;
//     }
//     (void)snprintf(line, sizeof(line), "%s%c%lu.%lu", prefix, sign,
//                    (unsigned long)(magnitude / 10U),
//                    (unsigned long)(magnitude % 10U));
//     (void)OLED_ShowString(0U, page, line);
// }

// static void H2026_InitOled(void)
// {
//     OLED_Config config = OLED_MakeSSD1306Config(OLED_DEFAULT_ADDR_7BIT);

//     g_oled_status = OLED_Init(&config);
//     if (g_oled_status == OLED_STATUS_ERROR_I2C_ADDRESS_NACK) {
//         config = OLED_MakeSSD1306Config(0x3DU);
//         g_oled_status = OLED_Init(&config);
//     }
// }

// static void H2026_RefreshOled(const CarFirmware *firmware,
//                               uint32_t now_ms)
// {
//     static uint32_t last_render_ms;
//     static uint32_t last_tx_ms;
//     static uint8_t tx_page = OLED_PAGE_COUNT;
//     char line[22];
//     uint32_t faults;
//     const char *run_state;

//     if ((firmware == 0) || !OLED_IsInitialized() ||
//         (g_oled_status != OLED_STATUS_OK)) {
//         return;
//     }

//     if (g_oled_refresh_requested ||
//         ((tx_page >= OLED_PAGE_COUNT) &&
//          ((uint32_t)(now_ms - last_render_ms) >=
//           H2026_OLED_RENDER_PERIOD_MS))) {
//         last_render_ms = now_ms;
//         g_oled_refresh_requested = false;
//         tx_page = 0U;
//         faults = firmware->hardware_faults | firmware->output.faults;
//         run_state = firmware->app.executor.finished ? "DONE" :
//                     (firmware->app.executor.running ? "RUN" : "STOP");

//         (void)OLED_Clear();
//         (void)snprintf(line, sizeof(line), "H26%s %s T:%lu.%02lu",
//                        H2026_ModeName(firmware->config.mode), run_state,
//                        (unsigned long)(firmware->output.run_time_ms / 1000U),
//                        (unsigned long)((firmware->output.run_time_ms / 10U) %
//                                        100U));
//         (void)OLED_ShowString(0U, 0U, line);
//         (void)snprintf(line, sizeof(line), "IMU:%s GC:%s",
//                        firmware->yaw.calibrated ? "OK" : "WAIT",
//                        H2026_GrayCalibrationName(firmware->gray_cal_state));
//         (void)OLED_ShowString(0U, 1U, line);
//         H2026_ShowFixed1(2U, "YAW:", firmware->imu_sample.yaw_deg);
//         (void)snprintf(line, sizeof(line), "SEG:%02u P:%u X:%s",
//                        (unsigned)firmware->app.executor.index,
//                        (unsigned)firmware->app.executor.track_phase,
//                        H2026_RouteExitName(
//                            firmware->app.executor.last_exit_reason));
//         (void)OLED_ShowString(0U, 3U, line);
//         (void)snprintf(line, sizeof(line), "K:%u%u%u B:%lu %s",
//                        TiMspm0Platform_ReadKey1Level() ? 1U : 0U,
//                        TiMspm0Platform_ReadKey2Level() ? 1U : 0U,
//                        TiMspm0Platform_ReadKey3Level() ? 1U : 0U,
//                        (unsigned long)firmware->button_event_count,
//                        H2026_ButtonActionName(firmware->last_button_action));
//         (void)OLED_ShowString(0U, 4U, line);
//         if ((firmware->gray_cal_state == CAR_GRAY_CAL_CAPTURE_WHITE) ||
//             (firmware->gray_cal_state == CAR_GRAY_CAL_CAPTURE_BLACK)) {
//             (void)snprintf(line, sizeof(line), "GC:%s %u/%u",
//                            H2026_GrayCalibrationName(
//                                firmware->gray_cal_state),
//                            (unsigned)firmware->gray_cal_frame_count,
//                            (unsigned)CAR_GRAY_CALIBRATION_FRAMES);
//         } else if (firmware->gray_cal_state == CAR_GRAY_CAL_ERROR) {
//             (void)snprintf(line, sizeof(line), "GC:ERR C%u D%ld",
//                            (unsigned)(firmware->gray_cal_bad_channel + 1U),
//                            (long)firmware->gray_cal_bad_span);
//         } else {
//             (void)snprintf(line, sizeof(line), "LN:%c C:%u A:%u M:%02X",
//                            firmware->app.line.valid ? 'Y' : 'N',
//                            (unsigned)firmware->app.line.track_confidence,
//                            (unsigned)firmware->app.line.track_active_count,
//                            (unsigned)firmware->app.line.track_active_mask);
//         }
//         (void)OLED_ShowString(0U, 5U, line);
//         (void)snprintf(line, sizeof(line), "PWM:%d/%d",
//                        TB6612_GetLeftCommand(), TB6612_GetRightCommand());
//         (void)OLED_ShowString(0U, 6U, line);
//         (void)snprintf(line, sizeof(line), "F:%08lX",
//                        (unsigned long)faults);
//         (void)OLED_ShowString(0U, 7U, line);
//     }

//     if ((tx_page >= OLED_PAGE_COUNT) ||
//         ((uint32_t)(now_ms - last_tx_ms) < H2026_OLED_TX_PERIOD_MS)) {
//         return;
//     }
//     last_tx_ms = now_ms;
//     if (OLED_UpdatePages(tx_page, 1U) != OLED_STATUS_OK) {
//         g_oled_status = OLED_STATUS_ERROR_I2C_BUS;
//         return;
//     }
//     tx_page++;
// }

// static void H2026_RunFirmware(void)
// {
//     CarFirmwareConfig config;
//     CarStatus status;
//     uint32_t last_telemetry_ms = 0U;
//     uint16_t last_route_index = UINT16_MAX;
//     CarTrackPhase last_track_phase = (CarTrackPhase)UINT8_MAX;
//     CarRouteExitReason last_exit_reason =
//         (CarRouteExitReason)UINT8_MAX;
//     uint32_t last_faults = UINT32_MAX;

//     TiMspm0Platform_Init();
//     H2026_InitOled();
//     status = TiMspm0Platform_BuildConfig(&config, H2026_SelectedMode());
//     if (status == CAR_OK) {
//         status = CarFirmware_Init(&g_firmware, &config,
//                                   TiMspm0Platform_Millis());
//     }
//     if (status != CAR_OK) {
//         while (1) {
//             __WFI();
//         }
//     }

//     VofaTelemetry_Init();
//     NVIC_ClearPendingIRQ(UART_VOFA_INST_INT_IRQN);
//     NVIC_EnableIRQ(UART_VOFA_INST_INT_IRQN);
//     VofaTelemetry_SendBanner();

//     while (1) {
//         uint32_t now_ms = TiMspm0Platform_Millis();
//         uint32_t faults;
//         bool command_event;
//         bool telemetry_event;

//         command_event = VofaTelemetry_ProcessCommands(&g_firmware, now_ms);
//         CarFirmware_Tick(&g_firmware, now_ms);
//         faults = g_firmware.hardware_faults | g_firmware.output.faults;
//         telemetry_event =
//             command_event ||
//             (g_firmware.app.executor.index != last_route_index) ||
//             (g_firmware.app.executor.track_phase != last_track_phase) ||
//             (g_firmware.app.executor.last_exit_reason != last_exit_reason) ||
//             (faults != last_faults);
//         if (telemetry_event ||
//             ((uint32_t)(now_ms - last_telemetry_ms) >=
//              H2026_TELEMETRY_PERIOD_MS)) {
//             VofaTelemetry_SendFrame(&g_firmware, now_ms);
//             last_telemetry_ms = now_ms;
//             last_route_index = g_firmware.app.executor.index;
//             last_track_phase = g_firmware.app.executor.track_phase;
//             last_exit_reason = g_firmware.app.executor.last_exit_reason;
//             last_faults = faults;
//         }
//         H2026_RefreshOled(&g_firmware, now_ms);
//         DL_WWDT_restart(WWDT0_INST);
//         __WFI();
//     }
// }

// void UART_VOFA_INST_IRQHandler(void)
// {
//     switch (DL_UART_Main_getPendingInterrupt(UART_VOFA_INST)) {
//         case DL_UART_MAIN_IIDX_RX:
//             while (!DL_UART_Main_isRXFIFOEmpty(UART_VOFA_INST)) {
//                 VofaTelemetry_PushRxFromIsr(
//                     (uint8_t)DL_UART_Main_receiveData(
//                         UART_VOFA_INST));
//             }
//             break;
//         case DL_UART_MAIN_IIDX_TX:
//             VofaTelemetry_TxIrqHandler();
//             break;
//         default:
//             break;
//     }
// }

// int main(void)
// {
//     SYSCFG_DL_init();
//     DL_WWDT_setCoreHaltBehavior(WWDT0_INST, DL_WWDT_CORE_HALT_STOP);
//     DL_SYSTICK_config(CPUCLK_FREQ / 1000U);
//     H2026_RunFirmware();
//     return 0;
// }

// void SysTick_Handler(void)
// {
//     TiMspm0Platform_OnSysTick();
// }

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
#define H2026_OLED_UI_PAGE_COUNT (2U)
#define H2026_BUTTON_DEBOUNCE_MS (20U)

static CarFirmware g_firmware;
static Button g_page_button;
static OLED_Status g_oled_status = OLED_STATUS_ERROR_NOT_INITIALIZED;
static bool g_oled_dirty = true;
static uint8_t g_oled_ui_page;
static uint8_t g_oled_tx_page = OLED_PAGE_COUNT;
static uint32_t g_oled_last_render_ms;
static uint32_t g_oled_last_tx_ms;

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

static bool H2026_ReadPageButton(void *context)
{
    (void)context;
    return TiMspm0Platform_ReadKey3Level();
}

static const char *H2026_TrackName(const CarFirmware *firmware)
{
    return (firmware->config.track_sensor_source ==
            CAR_TRACK_SENSOR_RED_ARRAY) ? "RED" : "GRAY";
}

static const uint16_t *H2026_TrackRaw(const CarFirmware *firmware)
{
    if (firmware->config.track_sensor_source ==
        CAR_TRACK_SENSOR_RED_ARRAY) {
        return firmware->red.raw;
    }
    return firmware->gray.raw;
}

static bool H2026_TrackCalibrated(const CarFirmware *firmware)
{
    return firmware->gray_cal_state == CAR_GRAY_CAL_READY;
}

static const char *H2026_RunStateName(const CarFirmware *firmware)
{
    if (firmware->app.executor.finished) {
        return "DONE";
    }
    return firmware->app.executor.running ? "RUN" : "STOP";
}

static const char *H2026_ButtonActionName(CarButtonAction action)
{
    switch (action) {
        case CAR_BUTTON_ACTION_ARM_OK:
            return "ARM";
        case CAR_BUTTON_ACTION_ARM_REJECTED:
            return "REJ";
        case CAR_BUTTON_ACTION_GRAY_REQUIRED:
            return "CAL";
        case CAR_BUTTON_ACTION_EMERGENCY_STOP:
            return "STOP";
        case CAR_BUTTON_ACTION_NONE:
        default:
            return "-";
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

static int32_t H2026_Round(float value)
{
    return (int32_t)(value + ((value >= 0.0f) ? 0.5f : -0.5f));
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
            (void)OLED_ShowString(0U, 1U, "CAL:WHITE -> K2");
            break;
        case CAR_GRAY_CAL_CAPTURE_WHITE:
            (void)snprintf(line, sizeof(line), "WHITE %u/%u",
                           (unsigned)firmware->gray_cal_frame_count,
                           (unsigned)CAR_GRAY_CALIBRATION_FRAMES);
            (void)OLED_ShowString(0U, 1U, line);
            break;
        case CAR_GRAY_CAL_WAIT_BLACK:
            (void)OLED_ShowString(0U, 1U, "CAL:BLACK -> K2");
            break;
        case CAR_GRAY_CAL_CAPTURE_BLACK:
            (void)snprintf(line, sizeof(line), "BLACK %u/%u",
                           (unsigned)firmware->gray_cal_frame_count,
                           (unsigned)CAR_GRAY_CALIBRATION_FRAMES);
            (void)OLED_ShowString(0U, 1U, line);
            break;
        case CAR_GRAY_CAL_READY:
            (void)OLED_ShowString(0U, 1U, "CAL:OK K1 START");
            break;
        case CAR_GRAY_CAL_ERROR:
            (void)snprintf(line, sizeof(line), "CAL:ERR C%u D%ld",
                           (unsigned)(firmware->gray_cal_bad_channel + 1U),
                           (long)firmware->gray_cal_bad_span);
            (void)OLED_ShowString(0U, 1U, line);
            break;
        default:
            (void)OLED_ShowString(0U, 1U, "CAL:?");
            break;
    }
}

static void H2026_DrawStatusPage(const CarFirmware *firmware)
{
    char line[22];
    uint32_t faults = firmware->hardware_faults | firmware->output.faults;

    (void)snprintf(line, sizeof(line), "%s P1/2 B2 %s",
                   H2026_TrackName(firmware), H2026_RunStateName(firmware));
    (void)OLED_ShowString(0U, 0U, line);
    H2026_DrawCalibration(firmware);
    (void)snprintf(line, sizeof(line), "IMU:%s ENC:%u",
                   firmware->yaw.calibrated ? "OK" : "WAIT",
                   firmware->encoder_valid_current ? 1U : 0U);
    (void)OLED_ShowString(0U, 2U, line);
    (void)snprintf(line, sizeof(line), "SEG:%u P:%u X:%s",
                   (unsigned)firmware->app.executor.index,
                   (unsigned)firmware->app.executor.track_phase,
                   H2026_RouteExitName(
                       firmware->app.executor.last_exit_reason));
    (void)OLED_ShowString(0U, 3U, line);
    (void)snprintf(line, sizeof(line), "LINE:%c E:%+5d",
                   firmware->app.line.valid ? 'Y' : 'N',
                   (int)firmware->app.line.track_position);
    (void)OLED_ShowString(0U, 4U, line);
    (void)snprintf(line, sizeof(line), "PWM:%+3d/%+3d",
                   (int)TB6612_GetLeftCommand(),
                   (int)TB6612_GetRightCommand());
    (void)OLED_ShowString(0U, 5U, line);
    (void)snprintf(line, sizeof(line), "K:%u%u%u A:%s",
                   TiMspm0Platform_ReadKey1Level() ? 1U : 0U,
                   TiMspm0Platform_ReadKey2Level() ? 1U : 0U,
                   TiMspm0Platform_ReadKey3Level() ? 1U : 0U,
                   H2026_ButtonActionName(firmware->last_button_action));
    (void)OLED_ShowString(0U, 6U, line);
    (void)snprintf(line, sizeof(line), "F:%08lX",
                   (unsigned long)faults);
    (void)OLED_ShowString(0U, 7U, line);
}

static void H2026_DrawDataPage(const CarFirmware *firmware)
{
    char line[22];
    const uint16_t *raw = H2026_TrackRaw(firmware);
    const uint16_t *values = H2026_TrackCalibrated(firmware) ?
        firmware->gray_sample.normalized : raw;
    char prefix = H2026_TrackCalibrated(firmware) ? 'N' : 'R';
    TB6612SpeedLoopStatus speed = {0};

    TB6612_DriveGetSpeedLoopStatus(
        (const TB6612Drive *)firmware->config.drive.context, &speed);
    (void)snprintf(line, sizeof(line), "%s P2/2 %s",
                   H2026_TrackName(firmware),
                   H2026_TrackCalibrated(firmware) ? "NRM" : "RAW");
    (void)OLED_ShowString(0U, 0U, line);
    for (uint8_t row = 0U; row < 4U; row++) {
        uint8_t first = (uint8_t)(row * 2U);

        (void)snprintf(line, sizeof(line), "%c%u:%4u %u:%4u",
                       prefix, (unsigned)(first + 1U),
                       (unsigned)values[first],
                       (unsigned)(first + 2U),
                       (unsigned)values[first + 1U]);
        (void)OLED_ShowString(0U, (uint8_t)(row + 1U), line);
    }
    (void)snprintf(line, sizeof(line), "E:%+5d U:%+4ld",
                   (int)firmware->app.line.track_position,
                   (long)H2026_Round(
                       firmware->app.executor.line_correction_mm_s));
    (void)OLED_ShowString(0U, 5U, line);
    (void)snprintf(line, sizeof(line), "T:%+4ld/%+4ld",
                   (long)H2026_ClampDisplayRpm(speed.target_left_rpm),
                   (long)H2026_ClampDisplayRpm(speed.target_right_rpm));
    (void)OLED_ShowString(0U, 6U, line);
    (void)snprintf(line, sizeof(line), "M:%+4ld/%+4ld",
                   (long)H2026_ClampDisplayRpm(speed.measured_left_rpm),
                   (long)H2026_ClampDisplayRpm(speed.measured_right_rpm));
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
        if (g_oled_ui_page == 0U) {
            H2026_DrawStatusPage(firmware);
        } else {
            H2026_DrawDataPage(firmware);
        }
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

static void H2026_ServicePageButton(uint32_t now_ms)
{
    Button_Update(&g_page_button, now_ms);
    if (Button_TakePressedEvent(&g_page_button)) {
        g_oled_ui_page = (uint8_t)((g_oled_ui_page + 1U) %
                                   H2026_OLED_UI_PAGE_COUNT);
        g_oled_dirty = true;
    }
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
    Button_Init(&g_page_button, H2026_ReadPageButton, 0, true,
                H2026_BUTTON_DEBOUNCE_MS);
    status = TiMspm0Platform_BuildConfig(&config, H2026_SelectedMode());
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
        bool telemetry_event;

        command_event = VofaTelemetry_ProcessCommands(&g_firmware, now_ms);
        CarFirmware_Tick(&g_firmware, now_ms);
        H2026_ServicePageButton(now_ms);
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

#if 0
/*
 * 最小验证单元：TB6612 开环 + 八路红外原始值/归一化值 + 按键黑白标定。
 *
 * 不使用 firmware.c / platform / OLED / IMU / 编码器，只依赖：
 *   ti_msp_dl_config.h（SysConfig 生成）、tb6612.c
 *   以及被选中的那一个巡线驱动：drivers/gray_array.c 或 drivers/red_array.c
 *
 * 传感器后端由 platform/ti_mspm0_platform_config.h 的
 * H2026_TRACK_SENSOR_SOURCE 决定（CAR_TRACK_SENSOR_GRAY_ARRAY /
 * CAR_TRACK_SENSOR_RED_ARRAY），与正式固件共用同一个开关。
 * 两者当前共用同一套硬件：3 位地址 mux PA24/25/26 + ADC PA27。
 *
 * 蓝牙 = UART_VOFA = UART3 = PB2(TX)/PB3(RX)，115200 8N1。
 *
 * 串口单字符命令：
 *   w/s/a/d  前进/后退/左转/右转      x 停车
 *   1        采白（8 路全压白底）      2 采黑（8 路全压黑胶带）
 *   3        打印当前标定表和 span     4 打印一次详细文本
 *   5        CSV 数据流开/关           h 打印帮助
 * 按键等价：KEY1(PB23)=启停  KEY2(PB26)=白/黑标定步骤
 *             KEY3(PB27)=OLED P1/P2 切换（均低有效）
 *
 * CSV 流（100 ms 一帧，VOFA+ FireWater 可直接绘图，16 通道）：
 *   raw0..raw7,norm0..norm7\r\n
 *   raw  = 12 位 ADC 原始值 0..4095
 *   norm = 归一化黑度 0..1000（0=白，1000=黑），未标定时恒为 0
 * 以 '#' 开头的行是文本状态，不是数据。
 */

#include "ti_msp_dl_config.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "OLED.h"

#include "firmware.h"                        /* CarTrackSensorSource 枚举 */
#include "platform/ti_mspm0_platform_config.h" /* H2026_TRACK_SENSOR_SOURCE */
#include "tb6612.h"

#define TRACK_CHANNELS GRAY_ARRAY_CHANNELS

_Static_assert(GRAY_ARRAY_CHANNELS == RED_ARRAY_CHANNELS,
               "GRAY and RED arrays must expose the same channel count");
_Static_assert((H2026_TRACK_SENSOR_SOURCE == CAR_TRACK_SENSOR_GRAY_ARRAY) ||
                   (H2026_TRACK_SENSOR_SOURCE == CAR_TRACK_SENSOR_RED_ARRAY),
               "H2026_TRACK_SENSOR_SOURCE must select GRAY_ARRAY or RED_ARRAY");

#define TEST_DUTY            (30)
#define GRAY_PRINT_PERIOD_MS (100U)
#define GRAY_CAL_FRAMES      (16U)
#define GRAY_SETTLE_US       (10U)
#define GRAY_SAMPLES_PER_CH  (4U)
#define KEY_DEBOUNCE_MS      (20U)
#define ADC_TIMEOUT_LOOPS    (100000U)
#define UART_TIMEOUT_LOOPS   (100000U)
#define OLED_REFRESH_MS      (200U)
#define OLED_TX_PERIOD_MS    (5U)
#define OLED_UI_PAGE_COUNT   (2U)
#define OLED_ERROR_REPORT_MS (1000U)
#define ADC_ERROR_REPORT_MS  (500U)
#define MOTOR_TEST_TIMEOUT_MS (2000U)

#define KEY_INDEX_START      (0U) /* KEY1: middle */
#define KEY_INDEX_CAL        (1U) /* KEY2: right */
#define KEY_INDEX_PAGE       (2U) /* KEY3: left */

typedef enum {
    TRACK_CAL_WAIT_WHITE = 0,
    TRACK_CAL_CAPTURE_WHITE,
    TRACK_CAL_WAIT_BLACK,
    TRACK_CAL_CAPTURE_BLACK,
    TRACK_CAL_READY,
    TRACK_CAL_ADC_ERROR,
    TRACK_CAL_SPAN_ERROR
} TrackCalibrationState;

typedef enum {
    TRACK_ADC_NONE = 0,
    TRACK_ADC_ISR,
    TRACK_ADC_POLL,
    TRACK_ADC_TIMEOUT
} TrackAdcCompletion;

static volatile uint32_t g_ms;
static volatile bool     g_adc_ready;
static volatile uint32_t g_adc_isr_count;
static uint32_t          g_adc_poll_count;
static uint32_t          g_adc_timeout_count;
static TrackAdcCompletion g_adc_last_completion;

static GrayArray g_gray;
static RedArray  g_red;
static uint16_t  g_cal_white[TRACK_CHANNELS];
static uint16_t  g_cal_black[TRACK_CHANNELS];
static bool      g_white_done;
static bool      g_black_done;
static bool      g_stream_on = true;
static bool      g_test_running;
static uint32_t  g_test_started_ms;
static TrackCalibrationState g_cal_state = TRACK_CAL_WAIT_WHITE;
static uint8_t   g_cal_bad_channel;
static int32_t   g_cal_bad_span;
static OLED_Status g_oled_status = OLED_STATUS_ERROR_NOT_INITIALIZED;
static bool      g_oled_dirty = true;
static uint8_t   g_oled_ui_page;
static uint8_t   g_oled_tx_page = OLED_PAGE_COUNT;
static uint32_t  g_oled_last_render_ms;
static uint32_t  g_oled_last_tx_ms;
static uint32_t  g_oled_last_error_report_ms;
static uint32_t  g_oled_update_error_count;

static const uint32_t kKeyPins[3] = {
    GPIO_KEYS_KEY1_PIN, GPIO_KEYS_KEY2_PIN, GPIO_KEYS_KEY3_PIN
};
static bool     g_key_level[3];
static bool     g_key_stable[3];
static uint32_t g_key_change_ms[3];

void SysTick_Handler(void)
{
    g_ms++;
}

void ADC_GRAY_INST_IRQHandler(void)
{
    if (DL_ADC12_getPendingInterrupt(ADC_GRAY_INST) ==
        DL_ADC12_IIDX_MEM0_RESULT_LOADED) {
        g_adc_ready = true;
    }
}

/* ---------------- UART 输出（必须等 TX FIFO，否则静默丢字节） --------------- */

static bool uart_putc(char value)
{
    uint32_t timeout = UART_TIMEOUT_LOOPS;

    while (DL_UART_Main_isTXFIFOFull(UART_VOFA_INST) && (timeout > 0U)) {
        timeout--;
    }
    if (timeout == 0U) {
        return false;
    }
    DL_UART_Main_transmitData(UART_VOFA_INST, (uint8_t)value);
    return true;
}

static void uart_puts(const char *text)
{
    while (*text != '\0') {
        if (!uart_putc(*text++)) {
            break;
        }
    }
}

/* ---------------- 红外阵列底层：3 位地址选通 + 单通道 ADC ------------------ */

static bool gray_select(uint8_t channel, void *context)
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

static bool gray_read_adc(uint16_t *value, void *context)
{
    uint32_t timeout = ADC_TIMEOUT_LOOPS;
    bool completed_by_poll = false;

    (void)context;
    if (value == 0) {
        return false;
    }
    g_adc_ready = false;
    g_adc_last_completion = TRACK_ADC_NONE;
    DL_ADC12_clearInterruptStatus(ADC_GRAY_INST,
        DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);
    /* Source: TI MSPM0 SDK 2.09 adc12_single_conversion.c. */
    /* In non-repeat mode, re-arm ENC before every software conversion. */
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
        g_adc_timeout_count++;
        g_adc_last_completion = TRACK_ADC_TIMEOUT;
        DL_ADC12_clearInterruptStatus(ADC_GRAY_INST,
            DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);
        NVIC_ClearPendingIRQ(ADC_GRAY_INST_INT_IRQN);
        return false;
    }
    if (completed_by_poll) {
        g_adc_poll_count++;
        g_adc_last_completion = TRACK_ADC_POLL;
    } else {
        g_adc_isr_count++;
        g_adc_last_completion = TRACK_ADC_ISR;
    }
    *value = (uint16_t)DL_ADC12_getMemResult(ADC_GRAY_INST,
                                             ADC_GRAY_ADCMEM_GRAY_OUT);
    DL_ADC12_clearInterruptStatus(ADC_GRAY_INST,
        DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);
    NVIC_ClearPendingIRQ(ADC_GRAY_INST_INT_IRQN);
    return true;
}

static void track_adc_init(void)
{
    /* Keep this runtime setup identical to the known-good platform path. */
    DL_ADC12_disableConversions(ADC_GRAY_INST);
    DL_ADC12_initSingleSample(
        ADC_GRAY_INST,
        DL_ADC12_REPEAT_MODE_DISABLED,
        DL_ADC12_SAMPLING_SOURCE_AUTO,
        DL_ADC12_TRIG_SRC_SOFTWARE,
        DL_ADC12_SAMP_CONV_RES_12_BIT,
        DL_ADC12_SAMP_CONV_DATA_FORMAT_UNSIGNED);
    DL_ADC12_setSampleTime0(ADC_GRAY_INST, 8U);
    DL_ADC12_clearInterruptStatus(
        ADC_GRAY_INST, DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);
    DL_ADC12_enableInterrupt(
        ADC_GRAY_INST, DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);
    DL_ADC12_enableConversions(ADC_GRAY_INST);
    NVIC_ClearPendingIRQ(ADC_GRAY_INST_INT_IRQN);
    NVIC_EnableIRQ(ADC_GRAY_INST_INT_IRQN);
}

static void gray_delay_us(uint32_t delay_us, void *context)
{
    (void)context;
    while (delay_us > 0U) {
        DL_Common_delayCycles(CPUCLK_FREQ / 1000000U);
        delay_us--;
    }
}

/* ---------------- 后端适配层：两种驱动挂到同一套 mux + ADC 上 ------------- */

/* RED 后端的 port 要的是「一次给完整 8 通道」的回调，这里自己扫一遍 mux。 */
static bool track_read_frame(uint16_t values[RED_ARRAY_CHANNELS],
                             void *context)
{
    (void)context;
    for (uint8_t channel = 0U; channel < RED_ARRAY_CHANNELS; channel++) {
        uint32_t sum = 0U;

        if (!gray_select(channel, 0)) {
            return false;
        }
        gray_delay_us(GRAY_SETTLE_US, 0);
        for (uint8_t s = 0U; s < GRAY_SAMPLES_PER_CH; s++) {
            uint16_t sample;

            if (!gray_read_adc(&sample, 0)) {
                return false;
            }
            sum += sample;
        }
        values[channel] = (uint16_t)(sum / GRAY_SAMPLES_PER_CH);
    }
    return true;
}

static void track_init(void)
{
    const GrayArrayPort gray_port = {
        gray_select, gray_read_adc, gray_delay_us, 0,
        GRAY_SETTLE_US, GRAY_SAMPLES_PER_CH
    };
    const RedArrayPort red_port = {track_read_frame, 0, 1U};

    GrayArray_Init(&g_gray, &gray_port);
    RedArray_Init(&g_red, &red_port);
}

/* GRAY 后端的 port 是「按通道」的，驱动自己负责选通/等待/多次采样。 */
/* Dispatch every diagnostic operation through the one build-time source. */
static const char *track_backend_name(void)
{
    return (H2026_TRACK_SENSOR_SOURCE == CAR_TRACK_SENSOR_RED_ARRAY) ?
        "RED_ARRAY" : "GRAY_ARRAY";
}

static int32_t track_minimum_calibration_span(void)
{
    return (H2026_TRACK_SENSOR_SOURCE == CAR_TRACK_SENSOR_RED_ARRAY) ?
        RED_ARRAY_MIN_CALIBRATION_SPAN :
        GRAY_ARRAY_MIN_CALIBRATION_SPAN;
}

static bool track_read(uint32_t now_ms)
{
    if (H2026_TRACK_SENSOR_SOURCE == CAR_TRACK_SENSOR_RED_ARRAY) {
        return RedArray_Read(&g_red, now_ms);
    }
    return GrayArray_Read(&g_gray, now_ms);
}

static bool track_set_cal(const uint16_t black[TRACK_CHANNELS],
                          const uint16_t white[TRACK_CHANNELS])
{
    if (H2026_TRACK_SENSOR_SOURCE == CAR_TRACK_SENSOR_RED_ARRAY) {
        return RedArray_SetCalibration(&g_red, black, white);
    }
    return GrayArray_SetCalibration(&g_gray, black, white);
}

static bool track_calibrated(void)
{
    if (H2026_TRACK_SENSOR_SOURCE == CAR_TRACK_SENSOR_RED_ARRAY) {
        return g_red.calibrated;
    }
    return g_gray.calibrated;
}

static const uint16_t *track_raw(void)
{
    if (H2026_TRACK_SENSOR_SOURCE == CAR_TRACK_SENSOR_RED_ARRAY) {
        return g_red.raw;
    }
    return g_gray.raw;
}

static const CarGraySample *track_latest(void)
{
    if (H2026_TRACK_SENSOR_SOURCE == CAR_TRACK_SENSOR_RED_ARRAY) {
        return &g_red.latest;
    }
    return &g_gray.latest;
}

/* ---------------- 按键：低有效 + 20 ms 消抖，只返回按下沿 ----------------- */

static const char *track_adc_completion_name(void)
{
    switch (g_adc_last_completion) {
        case TRACK_ADC_ISR: return "ISR";
        case TRACK_ADC_POLL: return "POLL";
        case TRACK_ADC_TIMEOUT: return "TIMEOUT";
        default: return "NONE";
    }
}

static void track_oled_init(void)
{
    OLED_Config config = OLED_MakeSSD1306Config(OLED_DEFAULT_ADDR_7BIT);
    char line[48];

    g_oled_status = OLED_Init(&config);
    if (g_oled_status == OLED_STATUS_ERROR_I2C_ADDRESS_NACK) {
        config = OLED_MakeSSD1306Config(0x3DU);
        g_oled_status = OLED_Init(&config);
    }
    if (g_oled_status == OLED_STATUS_OK) {
        uart_puts("#OLED OK\r\n");
        g_oled_dirty = true;
    } else {
        (void)snprintf(line, sizeof(line), "#OLED OFF STATUS:%u\r\n",
                       (unsigned)g_oled_status);
        uart_puts(line);
    }
}

static const char *track_backend_short_name(void)
{
    return (H2026_TRACK_SENSOR_SOURCE == CAR_TRACK_SENSOR_RED_ARRAY) ?
        "RED" : "GRAY";
}

static void track_oled_draw_status_page(void)
{
    char line[22];
    const uint16_t *raw = track_raw();

    if (H2026_TRACK_SENSOR_SOURCE == CAR_TRACK_SENSOR_RED_ARRAY) {
        (void)OLED_ShowString(0U, 0U, "RED P1/2 IR:NC");
    } else {
        (void)OLED_ShowString(0U, 0U, "GRAY P1/2 EN:PB24");
    }

    switch (g_cal_state) {
        case TRACK_CAL_WAIT_WHITE:
            (void)OLED_ShowString(0U, 1U, "WHITE -> K2");
            break;
        case TRACK_CAL_CAPTURE_WHITE:
            (void)OLED_ShowString(0U, 1U, "WHITE SAMPLING");
            break;
        case TRACK_CAL_WAIT_BLACK:
            (void)OLED_ShowString(0U, 1U, "BLACK -> K2");
            break;
        case TRACK_CAL_CAPTURE_BLACK:
            (void)OLED_ShowString(0U, 1U, "BLACK SAMPLING");
            break;
        case TRACK_CAL_READY:
            (void)OLED_ShowString(0U, 1U, "CAL OK");
            break;
        case TRACK_CAL_ADC_ERROR:
            (void)OLED_ShowString(0U, 1U, "ADC FAIL -> K2");
            break;
        case TRACK_CAL_SPAN_ERROR:
            (void)snprintf(line, sizeof(line), "SPAN C%u:%ld",
                (unsigned)(g_cal_bad_channel + 1U),
                (long)g_cal_bad_span);
            (void)OLED_ShowString(0U, 1U, line);
            break;
        default:
            break;
    }

    (void)snprintf(line, sizeof(line), "W:%u B:%u CAL:%u",
        (unsigned)g_white_done, (unsigned)g_black_done,
        (unsigned)track_calibrated());
    (void)OLED_ShowString(0U, 2U, line);
    (void)snprintf(line, sizeof(line), "RUN:%u STREAM:%u",
        (unsigned)g_test_running, (unsigned)g_stream_on);
    (void)OLED_ShowString(0U, 3U, line);
    (void)snprintf(line, sizeof(line), "R1:%4u R8:%4u",
        (unsigned)raw[0], (unsigned)raw[7]);
    (void)OLED_ShowString(0U, 4U, line);
    (void)OLED_ShowString(0U, 5U, "K1 RUN/STOP");
    (void)OLED_ShowString(0U, 6U, "K2 CAL STEP");
    (void)snprintf(line, sizeof(line), "K3 PAGE ADC:%s",
        track_adc_completion_name());
    (void)OLED_ShowString(0U, 7U, line);
}

static void track_oled_draw_data_page(void)
{
    char line[22];
    const uint16_t *raw = track_raw();
    const CarGraySample *latest = track_latest();
    const uint16_t *values = track_calibrated() ? latest->normalized : raw;
    char value_prefix = track_calibrated() ? 'N' : 'R';

    (void)snprintf(line, sizeof(line), "%s P2/2 %s",
        track_backend_short_name(), track_calibrated() ? "NRM" : "RAW");
    (void)OLED_ShowString(0U, 0U, line);
    for (uint8_t row = 0U; row < 4U; row++) {
        uint8_t first = (uint8_t)(row * 2U);

        (void)snprintf(line, sizeof(line), "%c%u:%4u %u:%4u",
            value_prefix, (unsigned)(first + 1U),
            (unsigned)values[first], (unsigned)(first + 2U),
            (unsigned)values[first + 1U]);
        (void)OLED_ShowString(0U, (uint8_t)(row + 1U), line);
    }
    (void)snprintf(line, sizeof(line), "CAL:%u LAST:%s",
        (unsigned)track_calibrated(), track_adc_completion_name());
    (void)OLED_ShowString(0U, 5U, line);
    (void)snprintf(line, sizeof(line), "I:%05lu P:%05lu",
        (unsigned long)(g_adc_isr_count % 100000U),
        (unsigned long)(g_adc_poll_count % 100000U));
    (void)OLED_ShowString(0U, 6U, line);
    (void)snprintf(line, sizeof(line), "T:%05lu K3 PAGE",
        (unsigned long)(g_adc_timeout_count % 100000U));
    (void)OLED_ShowString(0U, 7U, line);
}

static void track_oled_draw_buffer(void)
{
    (void)OLED_Clear();
    if (g_oled_ui_page == 0U) {
        track_oled_draw_status_page();
    } else {
        track_oled_draw_data_page();
    }
}

static void track_oled_service(uint32_t now_ms)
{
    if (!OLED_IsInitialized()) {
        return;
    }

    /* Keep the framebuffer stable until all eight hardware pages are sent. */
    if ((g_oled_tx_page >= OLED_PAGE_COUNT) &&
        (g_oled_dirty ||
         ((uint32_t)(now_ms - g_oled_last_render_ms) >= OLED_REFRESH_MS))) {
        track_oled_draw_buffer();
        g_oled_dirty = false;
        g_oled_last_render_ms = now_ms;
        g_oled_tx_page = 0U;
    }

    if ((g_oled_tx_page >= OLED_PAGE_COUNT) ||
        ((uint32_t)(now_ms - g_oled_last_tx_ms) < OLED_TX_PERIOD_MS)) {
        return;
    }
    g_oled_last_tx_ms = now_ms;
    g_oled_status = OLED_UpdatePages(g_oled_tx_page, 1U);
    if (g_oled_status == OLED_STATUS_OK) {
        g_oled_tx_page++;
        return;
    }

    g_oled_update_error_count++;
    g_oled_tx_page = OLED_PAGE_COUNT;
    g_oled_dirty = false;
    g_oled_last_render_ms = now_ms;
    if ((uint32_t)(now_ms - g_oled_last_error_report_ms) >=
        OLED_ERROR_REPORT_MS) {
        char line[64];

        g_oled_last_error_report_ms = now_ms;
        (void)snprintf(line, sizeof(line),
            "#OLED UPDATE_FAIL:%u COUNT:%lu\r\n",
            (unsigned)g_oled_status,
            (unsigned long)g_oled_update_error_count);
        uart_puts(line);
    }
}

static void track_oled_next_page(void)
{
    char line[32];

    g_oled_ui_page = (uint8_t)((g_oled_ui_page + 1U) %
                               OLED_UI_PAGE_COUNT);
    g_oled_dirty = true;
    (void)snprintf(line, sizeof(line), "#OLED PAGE %u/%u\r\n",
        (unsigned)(g_oled_ui_page + 1U), (unsigned)OLED_UI_PAGE_COUNT);
    uart_puts(line);
}

static void track_set_test_motors(int8_t left, int8_t right)
{
    bool was_running = g_test_running;

    TB6612_SetMotors(left, right);
    g_test_running = (left != 0) || (right != 0);
    if (g_test_running && !was_running) {
        g_test_started_ms = g_ms;
    }
    g_oled_dirty = true;
}

static void track_stop_test(void)
{
    track_set_test_motors(0, 0);
}

static void track_start_key(void)
{
    if (g_test_running) {
        track_stop_test();
        uart_puts("#RUN STOP KEY1\r\n");
        return;
    }
    if (!track_calibrated() || !track_latest()->valid) {
        uart_puts("#RUN REJECT CAL_REQUIRED\r\n");
        g_oled_dirty = true;
        return;
    }
    track_set_test_motors(TEST_DUTY, TEST_DUTY);
    uart_puts("#RUN TEST_FORWARD KEY1\r\n");
}

static bool key_take_press(uint8_t index)
{
    bool level = (DL_GPIO_readPins(GPIO_KEYS_PORT, kKeyPins[index]) == 0U);
    bool pressed = false;

    if (level != g_key_level[index]) {
        g_key_level[index] = level;
        g_key_change_ms[index] = g_ms;
    } else if (((uint32_t)(g_ms - g_key_change_ms[index]) >=
                KEY_DEBOUNCE_MS) && (level != g_key_stable[index])) {
        g_key_stable[index] = level;
        pressed = level;
    }
    return pressed;
}

/* ---------------- 标定与打印 ---------------------------------------------- */

static bool gray_capture_average(uint16_t out[TRACK_CHANNELS])
{
    uint32_t sum[TRACK_CHANNELS] = {0};

    for (uint8_t frame = 0U; frame < GRAY_CAL_FRAMES; frame++) {
        if (!track_read(g_ms)) {
            return false;
        }
        const uint16_t *raw = track_raw();
        for (uint8_t i = 0U; i < TRACK_CHANNELS; i++) {
            sum[i] += raw[i];
        }
    }
    for (uint8_t i = 0U; i < TRACK_CHANNELS; i++) {
        out[i] = (uint16_t)(sum[i] / GRAY_CAL_FRAMES);
    }
    return true;
}

static void gray_print_table(void)
{
    char line[80];
    int32_t minimum_span = track_minimum_calibration_span();

    uart_puts("#BACKEND ");
    uart_puts(track_backend_name());
    uart_puts("\r\n");
    uart_puts(track_calibrated() ?
        "#CAL VALID\r\n" : "#CAL INVALID\r\n");
    for (uint8_t i = 0U; i < TRACK_CHANNELS; i++) {
        int32_t span = (int32_t)g_cal_white[i] - (int32_t)g_cal_black[i];

        (void)snprintf(line, sizeof(line),
            "#CH%u W:%4u B:%4u SPAN:%+5ld%s\r\n",
            (unsigned)i, (unsigned)g_cal_white[i],
            (unsigned)g_cal_black[i], (long)span,
            ((span > -minimum_span) && (span < minimum_span)) ?
                " <<TOO_SMALL" : "");
        uart_puts(line);
    }
}

static bool gray_apply_calibration(void)
{
    int32_t minimum_span = track_minimum_calibration_span();

    if (!g_white_done || !g_black_done) {
        uart_puts("#CAL NEED_BOTH\r\n");
        return false;
    }
    if (track_set_cal(g_cal_black, g_cal_white)) {
        g_cal_state = TRACK_CAL_READY;
        g_oled_dirty = true;
        uart_puts("#CAL OK\r\n");
    } else {
        g_cal_state = TRACK_CAL_SPAN_ERROR;
        for (uint8_t i = 0U; i < TRACK_CHANNELS; i++) {
            int32_t span =
                (int32_t)g_cal_white[i] - (int32_t)g_cal_black[i];

            if ((span > -minimum_span) && (span < minimum_span)) {
                g_cal_bad_channel = i;
                g_cal_bad_span = span;
                break;
            }
        }
        g_oled_dirty = true;
        {
            char line[48];

            (void)snprintf(line, sizeof(line),
                "#CAL REJECTED abs(span)<%ld\r\n", (long)minimum_span);
            uart_puts(line);
        }
    }
    gray_print_table();
    return track_calibrated();
}

static bool gray_capture_reference(bool is_white)
{
    uint16_t *target = is_white ? g_cal_white : g_cal_black;

    /* Calibration must never run while the standalone motor test is active. */
    track_stop_test();
    if (is_white) {
        g_white_done = false;
        g_black_done = false;
        g_gray.calibrated = false;
        g_red.calibrated = false;
        g_cal_state = TRACK_CAL_CAPTURE_WHITE;
    } else if (!g_white_done) {
        g_cal_state = TRACK_CAL_WAIT_WHITE;
        g_oled_dirty = true;
        uart_puts("#CAL NEED_WHITE\r\n");
        return false;
    } else {
        g_cal_state = TRACK_CAL_CAPTURE_BLACK;
    }
    g_oled_dirty = true;

    if (!gray_capture_average(target)) {
        g_cal_state = TRACK_CAL_ADC_ERROR;
        g_oled_dirty = true;
        uart_puts("#CAL ADC_FAIL\r\n");
        return false;
    }
    if (is_white) {
        g_white_done = true;
        g_cal_state = TRACK_CAL_WAIT_BLACK;
        g_oled_dirty = true;
        uart_puts("#CAL WHITE_CAPTURED\r\n");
        return true;
    } else {
        g_black_done = true;
        uart_puts("#CAL BLACK_CAPTURED\r\n");
    }
    return gray_apply_calibration();
}

static void track_calibration_step(void)
{
    if (g_white_done && !g_black_done) {
        (void)gray_capture_reference(false);
    } else {
        (void)gray_capture_reference(true);
    }
}

static void gray_print_csv(void)
{
    char line[128];
    const uint16_t *raw = track_raw();
    const CarGraySample *latest = track_latest();

    (void)snprintf(line, sizeof(line),
        "%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\r\n",
        (unsigned)raw[0], (unsigned)raw[1],
        (unsigned)raw[2], (unsigned)raw[3],
        (unsigned)raw[4], (unsigned)raw[5],
        (unsigned)raw[6], (unsigned)raw[7],
        (unsigned)latest->normalized[0],
        (unsigned)latest->normalized[1],
        (unsigned)latest->normalized[2],
        (unsigned)latest->normalized[3],
        (unsigned)latest->normalized[4],
        (unsigned)latest->normalized[5],
        (unsigned)latest->normalized[6],
        (unsigned)latest->normalized[7]);
    uart_puts(line);
}

static void gray_print_detail(void)
{
    char line[100];
    const uint16_t *raw = track_raw();
    const CarGraySample *latest = track_latest();

    if (H2026_TRACK_SENSOR_SOURCE == CAR_TRACK_SENSOR_RED_ARRAY) {
        (void)snprintf(line, sizeof(line),
            "#RAW %4u %4u %4u %4u %4u %4u %4u %4u ERR:NA\r\n",
            (unsigned)raw[0], (unsigned)raw[1],
            (unsigned)raw[2], (unsigned)raw[3],
            (unsigned)raw[4], (unsigned)raw[5],
            (unsigned)raw[6], (unsigned)raw[7]);
    } else {
        (void)snprintf(line, sizeof(line),
            "#RAW %4u %4u %4u %4u %4u %4u %4u %4u ERR:%u\r\n",
            (unsigned)raw[0], (unsigned)raw[1],
            (unsigned)raw[2], (unsigned)raw[3],
            (unsigned)raw[4], (unsigned)raw[5],
            (unsigned)raw[6], (unsigned)raw[7],
            (unsigned)(DL_GPIO_readPins(GPIO_GRAY_ERR_PORT,
                                        GPIO_GRAY_ERR_PIN) != 0U));
    }
    uart_puts(line);
    (void)snprintf(line, sizeof(line),
        "#NRM %4u %4u %4u %4u %4u %4u %4u %4u VALID:%u\r\n",
        (unsigned)latest->normalized[0],
        (unsigned)latest->normalized[1],
        (unsigned)latest->normalized[2],
        (unsigned)latest->normalized[3],
        (unsigned)latest->normalized[4],
        (unsigned)latest->normalized[5],
        (unsigned)latest->normalized[6],
        (unsigned)latest->normalized[7],
        (unsigned)(latest->valid ? 1U : 0U));
    uart_puts(line);
    (void)snprintf(line, sizeof(line),
        "#ADC LAST:%s ISR:%lu POLL:%lu TIMEOUT:%lu\r\n",
        track_adc_completion_name(),
        (unsigned long)g_adc_isr_count,
        (unsigned long)g_adc_poll_count,
        (unsigned long)g_adc_timeout_count);
    uart_puts(line);
}

static void print_help(void)
{
    uart_puts("#BACKEND ");
    uart_puts(track_backend_name());
    uart_puts("\r\n");
    uart_puts("#KEY KEY1=MID_START KEY2=RIGHT_CAL KEY3=LEFT_PAGE\r\n");
    uart_puts("#WIRING RED_IR=NC VCC=UNKNOWN ADC_MAX=VDDA_3V3\r\n");
    uart_puts("#SAFETY motor run has absolute 2 s limit\r\n");
    uart_puts("#HELP w/s/a/d=move x=stop c=calStep 1=calWhite "
              "2=calBlack 3=table 4=once 5=stream h=help\r\n");
}

static void handle_serial(void)
{
    uint8_t command;

    if (DL_UART_Main_isRXFIFOEmpty(UART_VOFA_INST)) {
        return;
    }
    command = (uint8_t)DL_UART_Main_receiveData(UART_VOFA_INST);

    switch (command) {
        case 'w': track_set_test_motors( TEST_DUTY,  TEST_DUTY); break;
        case 's': track_set_test_motors(-TEST_DUTY, -TEST_DUTY); break;
        case 'a': track_set_test_motors(-TEST_DUTY,  TEST_DUTY); break;
        case 'd': track_set_test_motors( TEST_DUTY, -TEST_DUTY); break;
        case 'x': track_stop_test();                              break;
        case 'c': track_calibration_step();                       break;
        case '1': (void)gray_capture_reference(true);             break;
        case '2': (void)gray_capture_reference(false);            break;
        case '3': gray_print_table();                             break;
        case '4': gray_print_detail();                            break;
        case '5':
            g_stream_on = !g_stream_on;
            uart_puts(g_stream_on ? "#STREAM ON\r\n" : "#STREAM OFF\r\n");
            break;
        case 'h': print_help();                             break;
        case '\r':
        case '\n':
            break;
        default:
            /* 未知字符不动电机，只回显，便于确认链路。 */
            (void)uart_putc((char)command);
            uart_puts("?\r\n");
            break;
    }
}

int main(void)
{
    uint32_t last_print_ms = 0U;
    uint32_t last_adc_error_ms = 0U;

    SYSCFG_DL_init();
    /* 调试器暂停时冻结看门狗，否则一停就复位。 */
    DL_WWDT_setCoreHaltBehavior(WWDT0_INST, DL_WWDT_CORE_HALT_STOP);
    DL_SYSTICK_config(CPUCLK_FREQ / 1000U);

    TB6612_Init();
    TB6612_SetMotors(0, 0);

    /* 红外阵列 EN 低有效。 */
    /* PB24 is legacy gray EN; the new RED IR pin is unconnected and unowned. */
    DL_GPIO_clearPins(GPIO_GRAY_EN_PORT, GPIO_GRAY_EN_PIN);
    track_adc_init();
    track_init();

    print_help();
    track_oled_init();

    while (1) {
        /* SysConfig 已把 WWDT0 配成 4 s 看门狗，主循环必须喂狗。 */
        DL_WWDT_restart(WWDT0_INST);

        handle_serial();
        if (key_take_press(KEY_INDEX_START)) {
            track_start_key();
        }
        if (key_take_press(KEY_INDEX_CAL)) {
            track_calibration_step();
        }
        if (key_take_press(KEY_INDEX_PAGE)) {
            track_oled_next_page();
        }
        if (g_test_running &&
            ((uint32_t)(g_ms - g_test_started_ms) >=
             MOTOR_TEST_TIMEOUT_MS)) {
            track_stop_test();
            uart_puts("#RUN STOP AUTO_TIMEOUT_2S\r\n");
        }

        if ((uint32_t)(g_ms - last_print_ms) >= GRAY_PRINT_PERIOD_MS) {
            last_print_ms = g_ms;
            if (!track_read(g_ms)) {
                char line[96];

                if (g_test_running) {
                    track_stop_test();
                    uart_puts("#RUN STOP ADC_FAIL\r\n");
                }
                g_cal_state = TRACK_CAL_ADC_ERROR;
                g_oled_dirty = true;
                if ((uint32_t)(g_ms - last_adc_error_ms) >=
                    ADC_ERROR_REPORT_MS) {
                    last_adc_error_ms = g_ms;
                    (void)snprintf(line, sizeof(line),
                        "#ERR ADC_TIMEOUT ISR:%lu POLL:%lu TIMEOUT:%lu\r\n",
                        (unsigned long)g_adc_isr_count,
                        (unsigned long)g_adc_poll_count,
                        (unsigned long)g_adc_timeout_count);
                    uart_puts(line);
                }
            } else {
                if (g_cal_state == TRACK_CAL_ADC_ERROR) {
                    g_cal_state = track_calibrated() ? TRACK_CAL_READY :
                        ((g_white_done && !g_black_done) ?
                         TRACK_CAL_WAIT_BLACK : TRACK_CAL_WAIT_WHITE);
                    g_oled_dirty = true;
                }
                if (g_stream_on) {
                    gray_print_csv();
                }
            }
        }
        track_oled_service(g_ms);
    }
}
#endif
