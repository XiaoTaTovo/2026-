#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "pitch_task_controller.h"

static PitchAxisVisionConfig g_vision_config;
static PitchAxisVisionReport g_vision_report;
static PitchAxisVelocityTestConfig g_velocity_config;
static PitchAxisVelocityTestReport g_velocity_report;
static uint32_t g_reset_count;
static uint32_t g_arm_count;

bool PitchAxisVisionControl_GetConfig(
    const PitchAxisVisionControl *control,
    PitchAxisVisionConfig *config)
{
    (void)control;
    *config = g_vision_config;
    return true;
}

bool PitchAxisVisionControl_UpdateConfig(
    PitchAxisVisionControl *control,
    const PitchAxisVisionConfig *config,
    uint32_t now_ms)
{
    (void)control;
    (void)now_ms;
    g_vision_config = *config;
    return true;
}

void PitchAxisVisionControl_ResetController(
    PitchAxisVisionControl *control,
    uint32_t now_ms)
{
    (void)control;
    (void)now_ms;
    g_reset_count++;
}

bool PitchAxisVisionControl_GetReport(
    const PitchAxisVisionControl *control,
    PitchAxisVisionReport *report)
{
    (void)control;
    *report = g_vision_report;
    return true;
}

bool PitchAxisVelocityTest_GetReport(
    const PitchAxisVelocityTest *test,
    PitchAxisVelocityTestReport *report)
{
    (void)test;
    *report = g_velocity_report;
    return true;
}

bool PitchAxisVelocityTest_GetConfig(
    const PitchAxisVelocityTest *test,
    PitchAxisVelocityTestConfig *config)
{
    (void)test;
    *config = g_velocity_config;
    return true;
}

bool PitchAxisVelocityTest_UpdateConfig(
    PitchAxisVelocityTest *test,
    const PitchAxisVelocityTestConfig *config)
{
    (void)test;
    g_velocity_config = *config;
    return true;
}

bool PitchAxisVelocityTest_SetAutomaticArmed(
    PitchAxisVelocityTest *test,
    bool armed,
    uint32_t now_ms)
{
    (void)test;
    (void)now_ms;
    g_arm_count++;
    g_velocity_report.automatic_armed = armed;
    return true;
}

bool PitchAxisVelocityTest_ClearAutomaticHold(PitchAxisVelocityTest *test)
{
    (void)test;
    return true;
}

static void reset_fakes(void)
{
    memset(&g_vision_config, 0, sizeof(g_vision_config));
    g_vision_config.target_position_0_1mm = -50;
    g_vision_config.minimum_safe_position_0_1mm = -1250;
    g_vision_config.maximum_safe_position_0_1mm = 1250;
    memset(&g_vision_report, 0, sizeof(g_vision_report));
    memset(&g_velocity_report, 0, sizeof(g_velocity_report));
    memset(&g_velocity_config, 0, sizeof(g_velocity_config));
    g_velocity_config.automatic_tilt_limit_um = 3900U;
    g_velocity_report.enabled = true;
    g_velocity_report.communication_ready = true;
    g_velocity_report.state = PITCH_VELOCITY_TEST_STATE_ENABLED_STOPPED;
    g_reset_count = 0U;
    g_arm_count = 0U;
}

static void service(
    PitchTaskController *controller,
    uint32_t now_ms,
    bool key1,
    bool key2,
    bool key3)
{
    PitchAxisVelocityTestButtons buttons = {
        key1, key2, key3, false
    };
    PitchTaskController_Service(controller, buttons, now_ms);
}

static void press_key(
    PitchTaskController *controller,
    uint32_t now_ms,
    uint8_t key)
{
    service(controller, now_ms, key == 1U, key == 2U, key == 3U);
    service(controller, now_ms + 30U, key == 1U, key == 2U, key == 3U);
    service(controller, now_ms + 60U, false, false, false);
    service(controller, now_ms + 90U, false, false, false);
}

static void test_task3_and_switch_safely(void)
{
    PitchTaskController controller;
    PitchTaskControllerConfig config = {
        .center_position_0_1mm = -50,
        .task3_offset_0_1mm = 500U,
        .task3_tolerance_0_1mm = 100U,
        .task3_velocity_limit_0_1mm_s = 100U,
        .task3_turnaround_dwell_ms = 100U,
        .position_hold_tilt_limit_um = 3900U,
        .task3_tilt_limit_um = 4000U,
        .button_debounce_ms = 30U
    };
    PitchTaskControllerReport report;
    PitchAxisVisionControl vision;
    PitchAxisVelocityTest velocity;

    reset_fakes();
    assert(PitchTaskController_Init(
        &controller, &vision, &velocity, &config, 0U));
    assert(PitchTaskController_GetReport(&controller, &report));
    assert(report.selected_task == PITCH_TASK_2);
    press_key(&controller, 1U, 2U);
    assert(PitchTaskController_GetReport(&controller, &report));
    assert(report.selected_task == PITCH_TASK_3);
    assert(g_velocity_config.automatic_tilt_limit_um == 4000U);

    press_key(&controller, 120U, 1U);
    assert(PitchTaskController_GetReport(&controller, &report));
    assert(report.state == PITCH_TASK_STATE_STARTING);
    assert(g_vision_config.target_position_0_1mm == 450);
    assert(g_velocity_report.automatic_armed);

    g_velocity_report.automatic_armed = true;
    g_vision_report.state = PITCH_VISION_STATE_TRACKING;
    g_vision_report.observation_fresh = true;
    g_vision_report.observation.valid = true;
    g_vision_report.error_0_1mm = 0;
    g_vision_report.ball_velocity_0_1mm_s = 200;
    service(&controller, 250U, false, false, false);
    assert(PitchTaskController_GetReport(&controller, &report));
    assert(report.state == PITCH_TASK_STATE_RUNNING_POSITIVE);
    service(&controller, 350U, false, false, false);
    assert(PitchTaskController_GetReport(&controller, &report));
    assert(report.state == PITCH_TASK_STATE_RUNNING_POSITIVE);

    g_vision_report.observation.x_0_1mm = 450;
    service(&controller, 400U, false, false, false);
    assert(PitchTaskController_GetReport(&controller, &report));
    assert(report.state == PITCH_TASK_STATE_RUNNING_NEGATIVE);
    assert(g_vision_config.target_position_0_1mm == -550);

    g_vision_report.ball_velocity_0_1mm_s = 50;
    service(&controller, 600U, false, false, false);
    service(&controller, 700U, false, false, false);
    assert(PitchTaskController_GetReport(&controller, &report));
    assert(report.state == PITCH_TASK_STATE_HOLDING);

    press_key(&controller, 800U, 2U);
    assert(PitchTaskController_GetReport(&controller, &report));
    assert(report.selected_task == PITCH_TASK_4);
    assert(!g_velocity_report.automatic_armed);
    assert(g_reset_count > 0U);
    assert(g_velocity_config.automatic_tilt_limit_um == 3900U);
}

static void test_start_waits_for_task_switch_stop(void)
{
    PitchTaskController controller;
    PitchTaskControllerConfig config = {
        .center_position_0_1mm = -50,
        .task3_offset_0_1mm = 500U,
        .task3_tolerance_0_1mm = 100U,
        .task3_velocity_limit_0_1mm_s = 100U,
        .task3_turnaround_dwell_ms = 100U,
        .position_hold_tilt_limit_um = 3900U,
        .task3_tilt_limit_um = 4000U,
        .button_debounce_ms = 30U
    };
    PitchTaskControllerReport report;
    PitchAxisVisionControl vision;
    PitchAxisVelocityTest velocity;

    reset_fakes();
    assert(PitchTaskController_Init(
        &controller, &vision, &velocity, &config, 0U));
    g_velocity_report.automatic_armed = true;
    g_velocity_report.state = PITCH_VELOCITY_TEST_STATE_RUNNING_AUTOMATIC;

    press_key(&controller, 1U, 2U);
    assert(PitchTaskController_GetReport(&controller, &report));
    assert(report.selected_task == PITCH_TASK_3);

    press_key(&controller, 100U, 1U);
    assert(PitchTaskController_GetReport(&controller, &report));
    assert(report.state == PITCH_TASK_STATE_IDLE);

    g_velocity_report.automatic_armed = false;
    g_velocity_report.state = PITCH_VELOCITY_TEST_STATE_ENABLED_STOPPED;
    service(&controller, 220U, false, false, false);
    assert(PitchTaskController_GetReport(&controller, &report));
    assert(report.state == PITCH_TASK_STATE_STARTING);
    assert(g_velocity_report.automatic_armed);
    assert(g_vision_config.target_position_0_1mm == 450);
}

static void test_task6_captures_latest_coordinate(void)
{
    PitchTaskController controller;
    PitchTaskControllerConfig config = {
        .center_position_0_1mm = -50,
        .task3_offset_0_1mm = 500U,
        .task3_tolerance_0_1mm = 100U,
        .task3_velocity_limit_0_1mm_s = 100U,
        .task3_turnaround_dwell_ms = 100U,
        .position_hold_tilt_limit_um = 3900U,
        .task3_tilt_limit_um = 4000U,
        .button_debounce_ms = 30U
    };
    PitchTaskControllerReport report;
    PitchAxisVisionControl vision;
    PitchAxisVelocityTest velocity;

    reset_fakes();
    assert(PitchTaskController_Init(
        &controller, &vision, &velocity, &config, 0U));
    press_key(&controller, 1U, 2U);
    press_key(&controller, 100U, 2U);
    press_key(&controller, 200U, 2U);
    press_key(&controller, 300U, 2U);
    assert(PitchTaskController_GetReport(&controller, &report));
    assert(report.selected_task == PITCH_TASK_6);
    assert(report.state == PITCH_TASK_STATE_WAIT_CAPTURE);

    g_vision_report.state = PITCH_VISION_STATE_TRACKING;
    g_vision_report.observation_fresh = true;
    g_vision_report.observation.valid = true;
    g_vision_report.observation.x_0_1mm = 321;
    press_key(&controller, 400U, 3U);
    assert(PitchTaskController_GetReport(&controller, &report));
    assert(report.captured_position_valid);
    assert(report.captured_position_0_1mm == 321);
    assert(report.target_position_0_1mm == 321);
    assert(report.state == PITCH_TASK_STATE_IDLE);
    assert(report.pid_profile == 1U);

    press_key(&controller, 500U, 3U);
    assert(PitchTaskController_GetReport(&controller, &report));
    assert(report.captured_position_0_1mm == 321);
    assert(report.pid_profile == 2U);

    press_key(&controller, 650U, 1U);
    assert(PitchTaskController_GetReport(&controller, &report));
    assert(report.state == PITCH_TASK_STATE_HOLDING);
    assert(g_velocity_report.automatic_armed);
}

static void test_task_pid_profiles_are_isolated(void)
{
    PitchTaskController controller;
    PitchTaskControllerConfig config = {
        .center_position_0_1mm = -50,
        .task3_offset_0_1mm = 500U,
        .task3_tolerance_0_1mm = 100U,
        .task3_velocity_limit_0_1mm_s = 100U,
        .task3_turnaround_dwell_ms = 100U,
        .position_hold_tilt_limit_um = 3900U,
        .task3_tilt_limit_um = 4000U,
        .button_debounce_ms = 30U
    };
    PitchAxisVisionControl vision;
    PitchAxisVelocityTest velocity;
    PitchAxisVisionConfig pid_config;

    reset_fakes();
    g_vision_config.kp_rpm_per_mm = 0.40f;
    assert(PitchTaskController_Init(
        &controller, &vision, &velocity, &config, 0U));

    press_key(&controller, 100U, 2U);
    assert(g_vision_config.kp_rpm_per_mm == 0.40f);
    pid_config = g_vision_config;
    pid_config.kp_rpm_per_mm = 0.65f;
    assert(PitchTaskController_UpdateActivePidConfig(
        &controller, &pid_config, 200U));

    press_key(&controller, 250U, 3U);
    assert(g_vision_config.kp_rpm_per_mm == 0.40f);
    pid_config = g_vision_config;
    pid_config.kp_rpm_per_mm = 0.75f;
    assert(PitchTaskController_UpdateActivePidConfig(
        &controller, &pid_config, 350U));
    press_key(&controller, 400U, 3U);
    press_key(&controller, 500U, 3U);
    assert(g_vision_config.kp_rpm_per_mm == 0.65f);

    press_key(&controller, 600U, 2U);
    assert(g_vision_config.kp_rpm_per_mm == 0.40f);
    pid_config = g_vision_config;
    pid_config.kp_rpm_per_mm = 0.25f;
    assert(PitchTaskController_UpdateActivePidConfig(
        &controller, &pid_config, 700U));
    press_key(&controller, 750U, 3U);
    assert(g_vision_config.kp_rpm_per_mm == 0.40f);
    press_key(&controller, 850U, 2U);
    press_key(&controller, 950U, 2U);
    press_key(&controller, 1050U, 2U);
    press_key(&controller, 1150U, 2U);
    assert(g_vision_config.kp_rpm_per_mm == 0.65f);
}

static void test_task3_profile_switches_plant_limits(void)
{
    PitchTaskController controller;
    PitchTaskControllerConfig config = {
        -50, 450U, 100U, 100U, 100U, 3900U, 4200U, 30U,
        {
            { 0.30f, 0.010f, 0.080f, 2.0f, 250, 0.20f,
              800, 8U, 16U, 4000U },
            { 0.35f, 0.020f, 0.120f, 3.0f, 300, 0.25f,
              600, 10U, 18U, 4200U },
            { 0.40f, 0.015f, 0.100f, 2.5f, 350, 0.30f,
              450, 12U, 20U, 4400U }
        }
    };
    PitchTaskControllerReport report;
    PitchAxisVisionControl vision;
    PitchAxisVelocityTest velocity;

    reset_fakes();
    assert(PitchTaskController_Init(
        &controller, &vision, &velocity, &config, 0U));
    press_key(&controller, 100U, 2U);
    assert(PitchTaskController_GetReport(&controller, &report));
    assert(report.pid_profile == 2U);
    assert(g_vision_config.maximum_speed_rpm == 18U);
    assert(g_vision_config.approach_band_0_1mm == 600);
    assert(g_vision_config.approach_speed_limit_rpm == 10U);
    assert(g_velocity_config.automatic_tilt_limit_um == 4200U);

    press_key(&controller, 200U, 3U);
    assert(PitchTaskController_GetReport(&controller, &report));
    assert(report.pid_profile == 3U);
    assert(g_vision_config.maximum_speed_rpm == 20U);
    assert(g_vision_config.approach_band_0_1mm == 450);
    assert(g_vision_config.approach_speed_limit_rpm == 12U);
    assert(g_velocity_config.automatic_tilt_limit_um == 4400U);
}

int main(void)
{
    test_task3_and_switch_safely();
    test_start_waits_for_task_switch_stop();
    test_task6_captures_latest_coordinate();
    test_task_pid_profiles_are_isolated();
    test_task3_profile_switches_plant_limits();
    puts("PITCH_TASK_CONTROLLER_TEST=PASS");
    return 0;
}
