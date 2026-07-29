#include "ti_msp_dl_config.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>

#include "project_mode.h"
#include "platform/ti_mspm0_platform.h"

#if PROJECT_MODE == PROJECT_MODE_GRAY_ADC_DEBUG

#include "app/gray_adc_debug.h"

#elif PROJECT_MODE == PROJECT_MODE_BLUETOOTH_TUNING

#include "bluetooth_control.h"
#include "encoder.h"
#include "tb6612.h"
#include "vofa_telemetry.h"

#define TELEMETRY_PERIOD_MS (100U)

static void RunBluetoothTuning(void)
{
    uint32_t lastTelemetryMs = 0;

    TB6612_Init();
    Encoder_Init();
    BluetoothControl_Init();
    VofaTelemetry_TxInit();

    NVIC_ClearPendingIRQ(UART_BLUETOOTH_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_BLUETOOTH_INST_INT_IRQN);

    VofaTelemetry_SendBanner();

    while (1) {
        BluetoothControlStatus status;
        uint32_t nowMs = TiMspm0Platform_Millis();
        bool sendNow;

        sendNow = BluetoothControl_ProcessPending(nowMs);

        if (BluetoothControl_CheckFailsafe(nowMs)) {
            sendNow = true;
        }

        BluetoothControl_Update(nowMs);

        if (sendNow ||
            ((uint32_t)(nowMs - lastTelemetryMs) >=
                TELEMETRY_PERIOD_MS)) {
            BluetoothControl_GetStatus(&status);
            VofaTelemetry_Send(&status, nowMs);
            lastTelemetryMs = nowMs;
        }

        __WFI();
    }
}

void UART_BLUETOOTH_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_BLUETOOTH_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            while (!DL_UART_Main_isRXFIFOEmpty(UART_BLUETOOTH_INST)) {
                BluetoothControl_PushRxFromIsr(
                    (uint8_t) DL_UART_Main_receiveData(
                        UART_BLUETOOTH_INST));
            }
            break;
        case DL_UART_MAIN_IIDX_TX:
            VofaTelemetry_TxIrqHandler();
            break;
        default:
            break;
    }
}

#else

#include "firmware.h"
#include "OLED.h"
#include "tb6612.h"
#include "vofa_telemetry.h"

static CarFirmware gFirmware;
static OLED_Status gH2024OledStatus = OLED_STATUS_ERROR_NOT_INITIALIZED;
static bool gH2024OledRefreshRequested = true;

#define H2024_OLED_RENDER_PERIOD_MS   (250U)
#define H2024_OLED_TX_PERIOD_MS       (5U)
#define H2024_ROUTE_TELEMETRY_PERIOD_MS (100U)

static const char *H2024_ModeName(H2024Mode mode)
{
    switch (mode) {
        case H2024_MODE_ITEM_1:
            return "ITEM1";
        case H2024_MODE_ITEM_2:
            return "ITEM2";
        case H2024_MODE_ITEM_3:
            return "ITEM3";
        case H2024_MODE_ITEM_4:
            return "ITEM4";
        case H2024_MODE_TURN_DEBUG:
            return "TURN";
        case H2026_MODE_ITEM_1:
            return "H26I1";
        case H2026_MODE_ITEM_2:
            return "H26I2";
        case H2026_MODE_ITEM_3:
            return "H26I3";
        case H2026_MODE_ITEM_4:
            return "H26I4";
        case H2026_MODE_B2:
            return "B2";
        case H2026_MODE_B3:
            return "B3";
        case H2026_MODE_B4:
            return "B4";
        case H2026_MODE_B5:
            return "B5";
        case H2026_MODE_B6:
            return "B6";
        default:
            return "?????";
    }
}

static const char *H2024_ButtonActionName(CarButtonAction action)
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

static const char *H2024_GrayCalName(CarGrayCalibrationState state)
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

static const char *H2024_RouteExitName(CarRouteExitReason reason)
{
    switch (reason) {
        case CAR_ROUTE_EXIT_LINE:
            return "LINE";
        case CAR_ROUTE_EXIT_DISTANCE:
            return "DIST";
        case CAR_ROUTE_EXIT_ANGLE:
            return "ANGLE";
        case CAR_ROUTE_EXIT_REACQUIRE:
            return "REACQ";
        case CAR_ROUTE_EXIT_LINE_MISS:
            return "MISS";
        case CAR_ROUTE_EXIT_NONE:
        default:
            return "-";
    }
}

static int32_t H2024_ToTenths(float value)
{
    float scaled = value * 10.0f;

    return (int32_t)(scaled + ((scaled >= 0.0f) ? 0.5f : -0.5f));
}

static void H2024_ShowFixed1(uint8_t page, const char *prefix, float value)
{
    char line[22];
    int32_t scaled = H2024_ToTenths(value);
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

static void H2024_InitOled(void)
{
    OLED_Config config = OLED_MakeSSD1306Config(OLED_DEFAULT_ADDR_7BIT);

    gH2024OledStatus = OLED_Init(&config);
    if (gH2024OledStatus == OLED_STATUS_ERROR_I2C_ADDRESS_NACK) {
        config = OLED_MakeSSD1306Config(0x3DU);
        gH2024OledStatus = OLED_Init(&config);
    }
}

static void H2024_RefreshOled(const CarFirmware *firmware,
                              uint32_t now_ms)
{
    static uint32_t last_render_ms;
    static uint32_t last_tx_ms;
    static uint8_t tx_page = OLED_PAGE_COUNT;
    char line[22];
    uint32_t faults;
    const char *segment_state;

    if ((firmware == 0) || !OLED_IsInitialized() ||
        (gH2024OledStatus != OLED_STATUS_OK)) {
        return;
    }

    if (gH2024OledRefreshRequested ||
        ((tx_page >= OLED_PAGE_COUNT) &&
         ((uint32_t)(now_ms - last_render_ms) >=
          H2024_OLED_RENDER_PERIOD_MS))) {
        last_render_ms = now_ms;
        gH2024OledRefreshRequested = false;
        tx_page = 0U;

        faults = firmware->hardware_faults | firmware->output.faults;
        segment_state = firmware->app.executor.finished ? "DONE" :
                        (firmware->app.executor.running ? "RUN" : "STOP");
        (void)OLED_Clear();
#if ((PROJECT_MODE >= PROJECT_MODE_H2026_B2) && \
     (PROJECT_MODE <= PROJECT_MODE_H2026_B6))
        (void)snprintf(line, sizeof(line), "%s %s T:%lu.%02lu",
                       H2024_ModeName(firmware->config.mode), segment_state,
                       (unsigned long)(firmware->output.run_time_ms / 1000U),
                       (unsigned long)((firmware->output.run_time_ms / 10U) %
                                       100U));
#elif (PROJECT_MODE >= PROJECT_MODE_H2026_ITEM_1) && \
      (PROJECT_MODE <= PROJECT_MODE_H2026_ITEM_4)
        (void)snprintf(line, sizeof(line), "%s %s V49",
                       H2024_ModeName(firmware->config.mode), segment_state);
#else
        (void)snprintf(line, sizeof(line), "%s %s TB",
                       H2024_ModeName(firmware->config.mode), segment_state);
#endif
        (void)OLED_ShowString(0U, 0U, line);
        (void)snprintf(line, sizeof(line), "IMU:%s GC:%s",
                       firmware->yaw.calibrated ? "OK" : "WAIT",
                       H2024_GrayCalName(firmware->gray_cal_state));
        (void)OLED_ShowString(0U, 1U, line);
        H2024_ShowFixed1(2U, "YAW:", firmware->imu_sample.yaw_deg);
        (void)snprintf(line, sizeof(line), "SEG:%02u P:%u X:%s",
                       (unsigned)firmware->app.executor.index,
                       (unsigned)firmware->app.executor.line_follow_phase,
                       H2024_RouteExitName(
                           firmware->app.executor.last_motion_exit_reason));
        (void)OLED_ShowString(0U, 3U, line);
        (void)snprintf(line, sizeof(line), "K:%u%u%u B:%lu %s",
                       TiMspm0Platform_ReadKey1Level() ? 1U : 0U,
                       TiMspm0Platform_ReadKey2Level() ? 1U : 0U,
                       TiMspm0Platform_ReadKey3Level() ? 1U : 0U,
                       (unsigned long)firmware->button_event_count,
                       H2024_ButtonActionName(firmware->last_button_action));
        (void)OLED_ShowString(0U, 4U, line);
        if ((firmware->gray_cal_state == CAR_GRAY_CAL_CAPTURE_WHITE) ||
            (firmware->gray_cal_state == CAR_GRAY_CAL_CAPTURE_BLACK)) {
            (void)snprintf(line, sizeof(line), "GC:%s %u/%u",
                           H2024_GrayCalName(firmware->gray_cal_state),
                           (unsigned)firmware->gray_cal_frame_count,
                           (unsigned)CAR_GRAY_CALIBRATION_FRAMES);
        } else if (firmware->gray_cal_state == CAR_GRAY_CAL_ERROR) {
            (void)snprintf(line, sizeof(line), "GC:ERR C%u D%ld",
                           (unsigned)(firmware->gray_cal_bad_channel + 1U),
                           (long)firmware->gray_cal_bad_span);
        } else {
            (void)snprintf(line, sizeof(line), "LN:%c C:%u A:%u M:%02X",
                           firmware->app.line.valid ? 'Y' : 'N',
                           (unsigned)firmware->app.line.confidence,
                           (unsigned)firmware->app.line.active_count,
                           (unsigned)firmware->app.line.active_mask);
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
        ((uint32_t)(now_ms - last_tx_ms) < H2024_OLED_TX_PERIOD_MS)) {
        return;
    }
    last_tx_ms = now_ms;
    if (OLED_UpdatePages(tx_page, 1U) != OLED_STATUS_OK) {
        gH2024OledStatus = OLED_STATUS_ERROR_I2C_BUS;
        return;
    }
    tx_page++;
}

static H2024Mode GetH2024Mode(void)
{
#if PROJECT_MODE == PROJECT_MODE_H2024_ITEM_1
    return H2024_MODE_ITEM_1;
#elif PROJECT_MODE == PROJECT_MODE_H2024_ITEM_2
    return H2024_MODE_ITEM_2;
#elif PROJECT_MODE == PROJECT_MODE_H2024_ITEM_3
    return H2024_MODE_ITEM_3;
#elif PROJECT_MODE == PROJECT_MODE_TURN_DEBUG
    return H2024_MODE_TURN_DEBUG;
#elif PROJECT_MODE == PROJECT_MODE_H2026_ITEM_1
    return H2026_MODE_ITEM_1;
#elif PROJECT_MODE == PROJECT_MODE_H2026_ITEM_2
    return H2026_MODE_ITEM_2;
#elif PROJECT_MODE == PROJECT_MODE_H2026_ITEM_3
    return H2026_MODE_ITEM_3;
#elif PROJECT_MODE == PROJECT_MODE_H2026_ITEM_4
    /* PROJECT_MODE_H2026_MAIN aliases the final three-lap route. */
    return H2026_MODE_ITEM_4;
#elif PROJECT_MODE == PROJECT_MODE_H2026_B2
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
    return H2024_MODE_ITEM_4;
#endif
}

static void RunH2024Firmware(void)
{
    CarFirmwareConfig config;
    CarStatus status;
    uint32_t last_telemetry_ms = 0U;
    uint16_t last_telemetry_index = UINT16_MAX;
    CarLineFollowPhase last_telemetry_phase =
        (CarLineFollowPhase)UINT8_MAX;
    CarRouteExitReason last_telemetry_exit =
        (CarRouteExitReason)UINT8_MAX;
    bool last_telemetry_candidate = false;
    uint8_t last_telemetry_event_streak = UINT8_MAX;
    uint8_t last_telemetry_reacquire_streak = UINT8_MAX;

    TiMspm0Platform_Init();
    H2024_InitOled();
    status = TiMspm0Platform_BuildConfig(&config, GetH2024Mode());
    if (status == CAR_OK) {
        status = CarFirmware_Init(
            &gFirmware, &config, TiMspm0Platform_Millis());
    }
    if (status != CAR_OK) {
        while (1) {
            __WFI();
        }
    }
    VofaTelemetry_RouteCommandInit();
    NVIC_ClearPendingIRQ(UART_BLUETOOTH_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_BLUETOOTH_INST_INT_IRQN);
    VofaTelemetry_SendRouteBanner();

    while (1) {
        uint32_t now_ms = TiMspm0Platform_Millis();
        bool telemetry_event;
        bool command_event;

        command_event =
            VofaTelemetry_ProcessRouteCommands(&gFirmware, now_ms);
        TiMspm0Platform_PollMotorRx(&gFirmware);
        CarFirmware_Tick(&gFirmware, now_ms);
        TiMspm0Platform_ServiceMotorBackend();
        telemetry_event =
            command_event ||
            (gFirmware.app.executor.index != last_telemetry_index) ||
            (gFirmware.app.executor.line_follow_phase !=
             last_telemetry_phase) ||
            (gFirmware.app.executor.last_motion_exit_reason !=
             last_telemetry_exit) ||
            (gFirmware.app.executor.line_corner_candidate !=
             last_telemetry_candidate) ||
            (gFirmware.app.executor.line_event_streak !=
             last_telemetry_event_streak) ||
            (gFirmware.app.executor.turn_line_reacquire_streak !=
             last_telemetry_reacquire_streak);
        if (telemetry_event ||
            ((uint32_t)(now_ms - last_telemetry_ms) >=
             H2024_ROUTE_TELEMETRY_PERIOD_MS)) {
            VofaTelemetry_SendRoute(&gFirmware, now_ms);
            last_telemetry_ms = now_ms;
            last_telemetry_index = gFirmware.app.executor.index;
            last_telemetry_phase =
                gFirmware.app.executor.line_follow_phase;
            last_telemetry_exit =
                gFirmware.app.executor.last_motion_exit_reason;
            last_telemetry_candidate =
                gFirmware.app.executor.line_corner_candidate;
            last_telemetry_event_streak =
                gFirmware.app.executor.line_event_streak;
            last_telemetry_reacquire_streak =
                gFirmware.app.executor.turn_line_reacquire_streak;
        }
        H2024_RefreshOled(&gFirmware, now_ms);
        __WFI();
    }
}

void UART_BLUETOOTH_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_BLUETOOTH_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            while (!DL_UART_Main_isRXFIFOEmpty(UART_BLUETOOTH_INST)) {
                VofaTelemetry_RouteCommandPushRxFromIsr(
                    (uint8_t)DL_UART_Main_receiveData(
                        UART_BLUETOOTH_INST));
            }
            break;
        case DL_UART_MAIN_IIDX_TX:
            VofaTelemetry_TxIrqHandler();
            break;
        default:
            break;
    }
}

#endif

int main(void)
{
    SYSCFG_DL_init();
    DL_SYSTICK_config(CPUCLK_FREQ / 1000U);

#if PROJECT_MODE == PROJECT_MODE_GRAY_ADC_DEBUG
    GrayAdcDebug_Run();
#elif PROJECT_MODE == PROJECT_MODE_BLUETOOTH_TUNING
    RunBluetoothTuning();
#else
    RunH2024Firmware();
#endif

    return 0;
}

void SysTick_Handler(void)
{
    TiMspm0Platform_OnSysTick();
}
