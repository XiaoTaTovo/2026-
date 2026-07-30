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
