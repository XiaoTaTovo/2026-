#include "speed_tuning.h"

#if H2026_SPEED_TUNING_BUILD

#include <stdbool.h>
#include <stdint.h>

#include "firmware.h"
#include "platform/ti_mspm0_platform.h"
#include "platform/ti_mspm0_platform_config.h"
#include "project_mode.h"
#include "tb6612.h"
#include "ti_msp_dl_config.h"
#include "vofa_telemetry.h"

#define SPEED_TUNING_TELEMETRY_PERIOD_MS (25U)
#define SPEED_TUNING_RUN_LIMIT_MS (5000U)
#define SPEED_TUNING_KEY_DEBOUNCE_MS (20U)
#define SPEED_TUNING_TARGET_MM_S (350)

static CarFirmware g_speed_firmware;
static CarFirmwareConfig g_speed_config;
static uint32_t g_trial_id;
static uint32_t g_trial_started_ms;
static uint32_t g_last_telemetry_ms;
static uint32_t g_key_changed_ms;
static bool g_key_level;
static bool g_key_reported;
static bool g_trial_active;

static bool SpeedTuning_KeyPressed(uint32_t now_ms)
{
    bool level = !TiMspm0Platform_ReadKey1Level();

    if (level != g_key_level) {
        g_key_level = level;
        g_key_changed_ms = now_ms;
    }
    if (!level) {
        g_key_reported = false;
        return false;
    }
    if (!g_key_reported &&
        ((uint32_t)(now_ms - g_key_changed_ms) >=
         SPEED_TUNING_KEY_DEBOUNCE_MS)) {
        g_key_reported = true;
        return true;
    }
    return false;
}

static void SpeedTuning_Stop(uint32_t now_ms, const char *reason)
{
    uint32_t duration_ms = 0U;

    if (g_trial_active) {
        duration_ms = now_ms - g_trial_started_ms;
    }
    g_speed_config.drive.stop(g_speed_config.drive.context);
    g_speed_firmware.output = (CarOutputSnapshot){0};
    g_speed_firmware.drive_active = false;
    if (g_trial_active) {
        VofaTelemetry_SendSpeedLoopDone(
            g_trial_id, reason, duration_ms,
            g_speed_firmware.hardware_faults |
            g_speed_firmware.output.faults);
    }
    g_trial_active = false;
}

static bool SpeedTuning_Start(uint32_t now_ms)
{
    const int16_t target_mm_s = SPEED_TUNING_TARGET_MM_S;

    if (g_speed_firmware.hardware_faults != CAR_FAULT_NONE) {
        return false;
    }
    if (!g_speed_config.drive.set_wheel_speeds(
            target_mm_s, target_mm_s, g_speed_config.drive.context)) {
        g_speed_firmware.hardware_faults |= CAR_FAULT_MOTOR_IO;
        return false;
    }
    g_speed_firmware.output = (CarOutputSnapshot){0};
    g_speed_firmware.output.motor.left_mm_s = (float)target_mm_s;
    g_speed_firmware.output.motor.right_mm_s = (float)target_mm_s;
    g_speed_firmware.output.motor.enable = true;
    g_speed_firmware.drive_active = true;
    g_trial_id++;
    g_trial_started_ms = now_ms;
    g_trial_active = true;
    VofaTelemetry_SendSpeedLoopArm(g_trial_id, target_mm_s, now_ms);
    return true;
}

int main(void)
{
    CarStatus status;

    SYSCFG_DL_init();
    DL_WWDT_setCoreHaltBehavior(WWDT0_INST, DL_WWDT_CORE_HALT_STOP);
    DL_SYSTICK_config(CPUCLK_FREQ / 1000U);
    TiMspm0Platform_Init();
    status = TiMspm0Platform_BuildConfig(&g_speed_config, H2026_MODE_B2);
    if (status == CAR_OK) {
        g_speed_firmware.config = g_speed_config;
        status = CarApp_Init(&g_speed_firmware.app, &g_speed_config.car);
        g_speed_firmware.initialized = status == CAR_OK;
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
    VofaTelemetry_SendSpeedLoopBanner();

    while (1) {
        bool command_event;
        uint32_t now_ms = TiMspm0Platform_Millis();

        command_event = VofaTelemetry_ProcessCommands(&g_speed_firmware,
                                                       now_ms);
        if (g_speed_firmware.hardware_faults != CAR_FAULT_NONE) {
            SpeedTuning_Stop(now_ms, "FAULT");
        } else if (SpeedTuning_KeyPressed(now_ms)) {
            if (g_trial_active) {
                SpeedTuning_Stop(now_ms, "KEY_STOP");
            } else {
                (void)SpeedTuning_Start(now_ms);
            }
        } else if (g_trial_active &&
                   ((uint32_t)(now_ms - g_trial_started_ms) >=
                    SPEED_TUNING_RUN_LIMIT_MS)) {
            SpeedTuning_Stop(now_ms, "TIME_LIMIT");
        }

        g_speed_config.drive.service(g_speed_config.drive.context);
        if (command_event ||
            ((uint32_t)(now_ms - g_last_telemetry_ms) >=
             SPEED_TUNING_TELEMETRY_PERIOD_MS)) {
            VofaTelemetry_SendSpeedLoopFrame(&g_speed_firmware, now_ms);
            g_last_telemetry_ms = now_ms;
        }
        DL_WWDT_restart(WWDT0_INST);
        __WFI();
    }
}

void SysTick_Handler(void)
{
    TiMspm0Platform_OnSysTick();
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

#endif

#if H2026_LINE_TUNING_BUILD

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "OLED.h"
#include "drivers/button.h"
#include "firmware.h"
#include "platform/chassis_feedforward_link.h"
#include "platform/ti_mspm0_platform.h"
#include "project_mode.h"
#include "tb6612.h"
#include "ti_msp_dl_config.h"
#include "vofa_telemetry.h"

#define LINE_TUNING_TELEMETRY_PERIOD_MS (25U)
#define LINE_TUNING_KEY_DEBOUNCE_MS (20U)
#define LINE_TUNING_OLED_PERIOD_MS (250U)
#define LINE_TUNING_OLED_TX_PERIOD_MS (5U)
#define LINE_TUNING_OLED_RENDER_ENABLED (1U)
#define LINE_TUNING_FEEDFORWARD_PERIOD_MS (20U)

static CarFirmware g_line_firmware;
static CarFirmwareConfig g_line_config;
static ChassisFeedforwardLink g_line_feedforward_link;
static Button g_line_stop_button;
static OLED_Status g_line_oled_status;
static uint32_t g_line_trial_id;
static uint32_t g_line_trial_started_ms;
static uint32_t g_line_last_telemetry_ms;
static uint32_t g_line_last_oled_ms;
static uint32_t g_line_last_oled_tx_ms;
static uint32_t g_line_last_feedforward_ms;
static uint32_t g_line_last_button_event_count;
static uint8_t g_line_oled_tx_page = OLED_PAGE_COUNT;
static bool g_line_trial_active;
static bool g_line_parameters_ok;
static bool g_line_boot_tick_sent;
static bool g_line_boot_frame_sent;
static bool g_line_key_levels_valid;
static bool g_line_key1_level;
static bool g_line_key2_level;
static bool g_line_key3_level;
static bool g_line_calibration_state_valid;
static CarGrayCalibrationState g_line_calibration_state;
static bool g_line_oled_status_reported;
static OLED_Status g_line_reported_oled_status;

static void LineTuning_ReportOledStatus(void)
{
    if (!g_line_oled_status_reported ||
        (g_line_oled_status != g_line_reported_oled_status)) {
        VofaTelemetry_SendLineTuningOledStatus(
            (uint32_t)g_line_oled_status, g_line_oled_tx_page);
        g_line_oled_status_reported = true;
        g_line_reported_oled_status = g_line_oled_status;
    }
}

static bool LineTuning_ReadKey3(void *context)
{
    (void)context;
    return TiMspm0Platform_ReadKey3Level();
}

static void LineTuning_Stop(uint32_t now_ms, const char *reason)
{
    uint32_t duration_ms = 0U;

    if (!g_line_trial_active) {
        return;
    }
    duration_ms = now_ms - g_line_trial_started_ms;
    CarApp_Stop(&g_line_firmware.app, CAR_FAULT_NONE, now_ms);
    g_line_config.drive.stop(g_line_config.drive.context);
    g_line_firmware.output = (CarOutputSnapshot){0};
    g_line_firmware.drive_active = false;
    VofaTelemetry_SendLineTuningDone(
        g_line_trial_id, reason, duration_ms,
        g_line_firmware.hardware_faults | g_line_firmware.app.faults);
    g_line_trial_active = false;
}

static void LineTuning_ServiceOled(uint32_t now_ms)
{
    char line[22];

    if ((LINE_TUNING_OLED_RENDER_ENABLED == 0U) ||
        (g_line_oled_status != OLED_STATUS_OK)) {
        return;
    }
    if (g_line_oled_tx_page >= OLED_PAGE_COUNT) {
        if ((uint32_t)(now_ms - g_line_last_oled_ms) <
            LINE_TUNING_OLED_PERIOD_MS) {
            return;
        }
        g_line_last_oled_ms = now_ms;
        (void)OLED_Clear();
        (void)OLED_ShowString(0U, 0U,
            g_line_trial_active ? "LINE:TUNE RUN" : "LINE:TUNE STOP");
        (void)snprintf(line, sizeof(line), "CAL:%s",
            (g_line_firmware.gray_cal_state == CAR_GRAY_CAL_READY) ?
            "OK K1 START" : "K2 WHITE/BLACK");
        (void)OLED_ShowString(0U, 1U, line);
        (void)snprintf(line, sizeof(line), "E:%+5d U:%+4d",
            (int)g_line_firmware.app.line.track_position,
            (int)g_line_firmware.app.executor.line_correction_mm_s);
        (void)OLED_ShowString(0U, 3U, line);
        (void)OLED_ShowString(0U, 6U,
            g_line_parameters_ok ? "BT PARAM:OK" : "BT PARAM:WAIT");
        g_line_oled_tx_page = 0U;
    }
    if ((uint32_t)(now_ms - g_line_last_oled_tx_ms) <
        LINE_TUNING_OLED_TX_PERIOD_MS) {
        return;
    }
    g_line_last_oled_tx_ms = now_ms;
    g_line_oled_status = OLED_UpdatePages(g_line_oled_tx_page, 1U);
    if (g_line_oled_status == OLED_STATUS_OK) {
        g_line_oled_tx_page++;
    } else {
        g_line_oled_tx_page = OLED_PAGE_COUNT;
    }
}

static void LineTuning_Start(uint32_t now_ms)
{
    if (g_line_firmware.last_button_action != CAR_BUTTON_ACTION_ARM_OK) {
        return;
    }
    g_line_trial_id++;
    g_line_trial_started_ms = now_ms;
    g_line_trial_active = true;
    g_line_parameters_ok = false;
    VofaTelemetry_SendLineTuningArm(
        g_line_trial_id, g_line_firmware.config.mode, now_ms);
}

int main(void)
{
    CarStatus status;
    OLED_Config oled_config = OLED_MakeSSD1306Config(OLED_DEFAULT_ADDR_7BIT);

    SYSCFG_DL_init();
    DL_WWDT_setCoreHaltBehavior(WWDT0_INST, DL_WWDT_CORE_HALT_STOP);
    DL_SYSTICK_config(CPUCLK_FREQ / 1000U);
    TiMspm0Platform_Init();
    ChassisFeedforwardLink_Init(&g_line_feedforward_link);
    status = TiMspm0Platform_BuildConfig(&g_line_config, H2026_MODE_B2);
    if (status == CAR_OK) {
        status = CarFirmware_Init(&g_line_firmware, &g_line_config,
                                  TiMspm0Platform_Millis());
    }
    if (status != CAR_OK) {
        TB6612_Stop();
        while (1) {
            DL_WWDT_restart(WWDT0_INST);
            __WFI();
        }
    }
    g_line_oled_status = OLED_Init(&oled_config);
    if (g_line_oled_status == OLED_STATUS_ERROR_I2C_ADDRESS_NACK) {
        oled_config = OLED_MakeSSD1306Config(0x3DU);
        g_line_oled_status = OLED_Init(&oled_config);
    }
    Button_Init(&g_line_stop_button, LineTuning_ReadKey3, 0, true,
                LINE_TUNING_KEY_DEBOUNCE_MS);
    VofaTelemetry_Init();
    NVIC_ClearPendingIRQ(UART_VOFA_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_VOFA_INST_INT_IRQN);
    VofaTelemetry_SendLineTuningBanner();
    VofaTelemetry_SendLineTuningBoot("READY");
    LineTuning_ReportOledStatus();

    while (1) {
        bool command_event;
        bool key1_level;
        bool key2_level;
        bool key3_level;
        uint32_t now_ms = TiMspm0Platform_Millis();

        key1_level = TiMspm0Platform_ReadKey1Level();
        key2_level = TiMspm0Platform_ReadKey2Level();
        key3_level = TiMspm0Platform_ReadKey3Level();
        if (!g_line_key_levels_valid ||
            (key1_level != g_line_key1_level) ||
            (key2_level != g_line_key2_level) ||
            (key3_level != g_line_key3_level)) {
            VofaTelemetry_SendLineTuningKeyLevels(
                key1_level, key2_level, key3_level);
            g_line_key_levels_valid = true;
            g_line_key1_level = key1_level;
            g_line_key2_level = key2_level;
            g_line_key3_level = key3_level;
        }
        command_event = VofaTelemetry_ProcessCommands(&g_line_firmware,
                                                       now_ms);
        if (VofaTelemetry_TakeParameterUpdateOk() && !g_line_trial_active) {
            g_line_parameters_ok = true;
        }
        CarFirmware_Tick(&g_line_firmware, now_ms);
        ChassisFeedforwardLink_Service(&g_line_feedforward_link);
        if ((uint32_t)(now_ms - g_line_last_feedforward_ms) >=
            LINE_TUNING_FEEDFORWARD_PERIOD_MS) {
            ChassisFeedforwardSample feedforward_sample;

            g_line_last_feedforward_ms = now_ms;
            if (CarFirmware_GetFeedforward(&g_line_firmware, now_ms,
                                           &feedforward_sample)) {
                uint16_t flags = ChassisFeedforward_MakeFlags(
                    &feedforward_sample,
                    g_line_firmware.app.executor.running,
                    (g_line_firmware.hardware_faults |
                     g_line_firmware.output.faults) != CAR_FAULT_NONE);

                (void)ChassisFeedforwardLink_QueueState(
                    &g_line_feedforward_link, &feedforward_sample, flags);
            }
        }
        if (!g_line_calibration_state_valid ||
            (g_line_firmware.gray_cal_state != g_line_calibration_state)) {
            VofaTelemetry_SendLineTuningCalibrationState(
                (uint32_t)g_line_firmware.gray_cal_state);
            g_line_calibration_state_valid = true;
            g_line_calibration_state = g_line_firmware.gray_cal_state;
        }
        if (!g_line_boot_tick_sent) {
            VofaTelemetry_SendLineTuningBoot("TICK");
            g_line_boot_tick_sent = true;
        }
        Button_Update(&g_line_stop_button, now_ms);
        if (g_line_firmware.button_event_count !=
            g_line_last_button_event_count) {
            if (g_line_firmware.last_button_action ==
                CAR_BUTTON_ACTION_ARM_OK) {
                LineTuning_Start(now_ms);
            }
            g_line_last_button_event_count =
                g_line_firmware.button_event_count;
        }
        if (Button_TakePressedEvent(&g_line_stop_button)) {
            LineTuning_Stop(now_ms, "KEY3_STOP");
        } else if (g_line_trial_active &&
                   (g_line_firmware.app.faults != CAR_FAULT_NONE)) {
            LineTuning_Stop(now_ms, "FAULT");
        } else if (g_line_trial_active &&
                   !g_line_firmware.app.executor.running) {
            LineTuning_Stop(now_ms, "ROUTE_DONE");
        }
        if (command_event ||
            ((uint32_t)(now_ms - g_line_last_telemetry_ms) >=
             LINE_TUNING_TELEMETRY_PERIOD_MS)) {
            VofaTelemetry_SendLineTuningFrame(&g_line_firmware, now_ms);
            g_line_last_telemetry_ms = now_ms;
            if (!g_line_boot_frame_sent) {
                VofaTelemetry_SendLineTuningBoot("FRAME");
                g_line_boot_frame_sent = true;
            }
        }
        LineTuning_ServiceOled(now_ms);
        LineTuning_ReportOledStatus();
        VofaTelemetry_ServiceTx();
        DL_WWDT_restart(WWDT0_INST);
        __WFI();
    }
}

void SysTick_Handler(void)
{
    TiMspm0Platform_OnSysTick();
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

#endif
