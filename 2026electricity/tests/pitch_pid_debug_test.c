#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "pitch_pid_debug.h"

static uint8_t g_rx[256];
static size_t g_rx_length;
static size_t g_rx_offset;
static char g_tx[4096];
static size_t g_tx_length;
static size_t g_tx_free_limit;
static size_t g_max_write_length;
static uint32_t g_arm_calls;
static uint32_t g_disarm_calls;
static uint32_t g_reset_calls;
static uint32_t g_zero_calls;

static void feed(const char *text)
{
    size_t length = strlen(text);

    assert(length <= sizeof(g_rx));
    memcpy(g_rx, text, length);
    g_rx_length = length;
    g_rx_offset = 0U;
}

static void clear_output(void)
{
    memset(g_tx, 0, sizeof(g_tx));
    g_tx_length = 0U;
    g_tx_free_limit = 511U;
    g_max_write_length = 0U;
}

static void service_until_input_drained(
    PitchPidDebug *debug,
    uint32_t now_ms)
{
    uint32_t iteration = 0U;

    while ((BspBluetooth_Available(debug->bluetooth) != 0U) &&
           (iteration < 8U))
    {
        PitchPidDebug_Service(debug, now_ms + iteration);
        iteration++;
    }
    assert(BspBluetooth_Available(debug->bluetooth) == 0U);
}

static void service_until_config_drained(
    PitchPidDebug *debug,
    uint32_t now_ms)
{
    uint32_t iteration = 0U;

    while (debug->config_response_pending && (iteration < 16U))
    {
        PitchPidDebug_Service(debug, now_ms + iteration);
        iteration++;
    }
    assert(!debug->config_response_pending);
}

size_t BspBluetooth_Available(const BspBluetooth *port)
{
    (void)port;
    return g_rx_length - g_rx_offset;
}

size_t BspBluetooth_TxFree(const BspBluetooth *port)
{
    (void)port;
    return g_tx_free_limit;
}

BspBluetoothResult BspBluetooth_Write(
    BspBluetooth *port,
    const uint8_t *data,
    size_t length)
{
    (void)port;
    assert(length <= g_tx_free_limit);
    assert(g_tx_length + length < sizeof(g_tx));
    if (length > g_max_write_length)
    {
        g_max_write_length = length;
    }
    memcpy(&g_tx[g_tx_length], data, length);
    g_tx_length += length;
    g_tx[g_tx_length] = '\0';
    return BSP_BLUETOOTH_OK;
}

BspBluetoothResult BspBluetooth_Read(
    BspBluetooth *port,
    uint8_t *data,
    size_t capacity,
    size_t *read_length)
{
    size_t available = g_rx_length - g_rx_offset;
    size_t count = (available < capacity) ? available : capacity;

    (void)port;
    if (count == 0U)
    {
        *read_length = 0U;
        return BSP_BLUETOOTH_EMPTY;
    }
    memcpy(data, &g_rx[g_rx_offset], count);
    g_rx_offset += count;
    *read_length = count;
    return BSP_BLUETOOTH_OK;
}

bool PitchAxisVisionControl_GetConfig(
    const PitchAxisVisionControl *control,
    PitchAxisVisionConfig *config)
{
    *config = control->config;
    return true;
}

bool PitchAxisVisionControl_UpdateConfig(
    PitchAxisVisionControl *control,
    const PitchAxisVisionConfig *config,
    uint32_t now_ms)
{
    (void)now_ms;
    control->config = *config;
    return config->maximum_speed_rpm >= config->minimum_speed_rpm;
}

void PitchAxisVisionControl_ResetController(
    PitchAxisVisionControl *control,
    uint32_t now_ms)
{
    (void)control;
    (void)now_ms;
    g_reset_calls++;
}

bool PitchAxisVisionControl_GetReport(
    const PitchAxisVisionControl *control,
    PitchAxisVisionReport *report)
{
    *report = control->report;
    return true;
}

bool PitchAxisVelocityTest_GetConfig(
    const PitchAxisVelocityTest *test,
    PitchAxisVelocityTestConfig *config)
{
    *config = test->config;
    return true;
}

bool PitchAxisVelocityTest_UpdateConfig(
    PitchAxisVelocityTest *test,
    const PitchAxisVelocityTestConfig *config)
{
    test->config = *config;
    return true;
}

bool PitchAxisVelocityTest_GetReport(
    const PitchAxisVelocityTest *test,
    PitchAxisVelocityTestReport *report)
{
    *report = test->report;
    return true;
}

bool PitchAxisVelocityTest_SetAutomaticArmed(
    PitchAxisVelocityTest *test,
    bool armed,
    uint32_t now_ms)
{
    (void)now_ms;
    if (armed)
    {
        g_arm_calls++;
    }
    else
    {
        g_disarm_calls++;
    }
    test->report.automatic_armed = armed;
    return true;
}

bool PitchAxisVelocityTest_ClearAutomaticHold(PitchAxisVelocityTest *test)
{
    if (!test->report.automatic_hold)
    {
        return false;
    }
    test->report.automatic_hold = false;
    return true;
}

bool PitchAxisVelocityTest_CaptureAutomaticZero(
    PitchAxisVelocityTest *test,
    uint32_t now_ms)
{
    (void)now_ms;
    if (test->report.automatic_armed ||
        (test->report.state != PITCH_VELOCITY_TEST_STATE_ENABLED_STOPPED))
    {
        return false;
    }
    g_zero_calls++;
    return true;
}

bool PitchAxisVelocityTest_SetPidDebugEnabled(
    PitchAxisVelocityTest *test,
    bool enabled,
    uint32_t now_ms)
{
    (void)now_ms;
    test->report.pid_debug_enabled = enabled;
    return true;
}

static PitchAxisVisionConfig default_vision_config(void)
{
    PitchAxisVisionConfig config;

    memset(&config, 0, sizeof(config));
    config.target_position_0_1mm = -50;
    config.minimum_safe_position_0_1mm = -1100;
    config.maximum_safe_position_0_1mm = 1100;
    config.edge_recovery_margin_0_1mm = 100;
    config.deadband_0_1mm = 20;
    config.velocity_deadband_0_1mm_s = 100;
    config.minimum_confidence_permille = 500U;
    config.maximum_observation_age_ms = 150U;
    config.control_period_ms = 50U;
    config.minimum_speed_rpm = 1U;
    config.maximum_speed_rpm = 5U;
    config.kp_rpm_per_mm = 0.03f;
    config.kd_rpm_per_mm_s = 0.01f;
    config.integral_limit_rpm = 1.0f;
    config.integral_separation_band_0_1mm = 150;
    config.approach_band_0_1mm = 60;
    config.approach_speed_limit_rpm = 1U;
    config.velocity_filter_alpha = 0.25f;
    return config;
}

static PitchAxisVelocityTestConfig default_velocity_config(void)
{
    PitchAxisVelocityTestConfig config;

    memset(&config, 0, sizeof(config));
    config.address = 1U;
    config.positive_direction = 0U;
    config.negative_direction = 1U;
    config.speed_rpm = 10U;
    config.acceleration = 100U;
    config.run_ms = 300U;
    config.debounce_ms = 30U;
    config.automatic_max_speed_rpm = 1U;
    config.automatic_position_raw_per_mm = 27760U;
    config.automatic_tilt_scale_um_per_outer_rpm = 67U;
    config.automatic_tilt_limit_um = 2000U;
    config.automatic_position_deadband_um = 30U;
    config.automatic_position_slow_zone_um = 300U;
    config.automatic_position_min_speed_rpm = 1U;
    config.automatic_position_poll_period_ms = 20U;
    config.automatic_decision_timeout_ms = 200U;
    config.automatic_vision_loss_grace_ms = 0U;
    config.automatic_motion_budget_ms = 3000U;
    return config;
}

static void initialize(
    PitchPidDebug *debug,
    BspBluetooth *bluetooth,
    PitchAxisVisionControl *vision,
    PitchAxisVelocityTest *velocity)
{
    memset(bluetooth, 0, sizeof(*bluetooth));
    memset(vision, 0, sizeof(*vision));
    memset(velocity, 0, sizeof(*velocity));
    memset(g_rx, 0, sizeof(g_rx));
    g_rx_length = 0U;
    g_rx_offset = 0U;
    g_arm_calls = 0U;
    g_disarm_calls = 0U;
    g_reset_calls = 0U;
    g_zero_calls = 0U;
    clear_output();
    vision->config = default_vision_config();
    velocity->config = default_velocity_config();
    velocity->report.state = PITCH_VELOCITY_TEST_STATE_DISABLED_READY;
    assert(PitchPidDebug_Init(debug, bluetooth, vision, velocity, NULL, 0U));
    debug->boot_report_pending = false;
}

static void test_ping_pid_on_and_status(void)
{
    PitchPidDebug debug;
    BspBluetooth bluetooth;
    PitchAxisVisionControl vision;
    PitchAxisVelocityTest velocity;

    initialize(&debug, &bluetooth, &vision, &velocity);
    feed("PING\r\nPID ON\r\nPID?\r\n");
    service_until_input_drained(&debug, 10U);
    service_until_config_drained(&debug, 20U);
    assert(strstr(g_tx, "PONG\r\n") != NULL);
    assert(strstr(g_tx, "PID_ON_OK\r\n") != NULL);
    assert(strstr(g_tx, "PID_CONFIG,") != NULL);
    assert(strstr(g_tx, ",pid_debug=1\r\n") != NULL);
    assert(!debug.config_response_pending);
    assert(g_max_write_length <= PITCH_PID_DEBUG_TX_CHUNK_SIZE);
    assert(debug.enabled);
    assert(velocity.report.pid_debug_enabled);
}

static void test_live_pid_and_automatic_limit_settings(void)
{
    PitchPidDebug debug;
    BspBluetooth bluetooth;
    PitchAxisVisionControl vision;
    PitchAxisVelocityTest velocity;

    initialize(&debug, &bluetooth, &vision, &velocity);
    velocity.report.state = PITCH_VELOCITY_TEST_STATE_RUNNING_AUTOMATIC;
    velocity.report.automatic_armed = true;
    velocity.report.automatic_motion_active = true;
    feed("SET KP=0.125\r\nSET ILIM=12.000\r\nSET IBAND=150\r\nSET APPBAND=60\r\nSET APPMAX=1\r\nSET MAXRPM=30\r\nSET AUTOMAX=2\r\nSET LOSSMS=250\r\nSET EDGEMARGIN=200\r\nSET RESCUERPM=20\r\nSET RESCUEMS=1500\r\nSET RESCUE=1\r\nSET ACCEL=0\r\n");
    service_until_input_drained(&debug, 10U);
    assert(vision.config.kp_rpm_per_mm > 0.124f);
    assert(vision.config.kp_rpm_per_mm < 0.126f);
    assert(vision.config.integral_limit_rpm > 11.999f);
    assert(vision.config.integral_limit_rpm < 12.001f);
    assert(vision.config.integral_separation_band_0_1mm == 150);
    assert(vision.config.approach_band_0_1mm == 60);
    assert(vision.config.approach_speed_limit_rpm == 1U);
    assert(vision.config.maximum_speed_rpm == 30U);
    assert(velocity.config.automatic_max_speed_rpm == 2U);
    assert(velocity.config.automatic_vision_loss_grace_ms == 250U);
    assert(vision.config.edge_recovery_margin_0_1mm == 200);
    assert(velocity.config.automatic_edge_recovery_enabled);
    assert(velocity.config.automatic_edge_recovery_speed_rpm == 20U);
    assert(velocity.config.automatic_edge_recovery_max_ms == 1500U);
    assert(velocity.config.acceleration == 0U);
    assert(strstr(g_tx, "PID_SET_OK,name=KP,value=0.125") != NULL);
    assert(strstr(g_tx, "PID_SET_OK,name=IBAND,value=150") != NULL);
    assert(strstr(g_tx, "PID_SET_OK,name=AUTOMAX,value=2") != NULL);
    assert(strstr(g_tx, "PID_SET_OK,name=ACCEL,value=0") != NULL);

    velocity.report.state = PITCH_VELOCITY_TEST_STATE_ENABLED_STOPPED;
    velocity.report.automatic_armed = false;
    velocity.report.automatic_motion_active = false;
    clear_output();
    feed("SET AUTOMAX=300\r\nSET AUTOMAX=301\r\nSET LOSSMS=5000\r\nSET LOSSMS=5001\r\nSET RESCUERPM=51\r\nSET RESCUEMS=3001\r\nSET BUDGET=0\r\nSET BUDGET=600000\r\nSET BUDGET=600001\r\n");
    service_until_input_drained(&debug, 20U);
    assert(velocity.config.automatic_max_speed_rpm == 300U);
    assert(strstr(g_tx, "PID_SET_OK,name=AUTOMAX,value=300") != NULL);
    assert(strstr(g_tx, "PID_SET_REJECTED,name=AUTOMAX,value=301") != NULL);
    assert(velocity.config.automatic_vision_loss_grace_ms == 5000U);
    assert(strstr(g_tx, "PID_SET_OK,name=LOSSMS,value=5000") != NULL);
    assert(strstr(g_tx, "PID_SET_REJECTED,name=LOSSMS,value=5001") != NULL);
    assert(strstr(g_tx, "PID_SET_REJECTED,name=RESCUERPM,value=51") != NULL);
    assert(strstr(g_tx, "PID_SET_REJECTED,name=RESCUEMS,value=3001") != NULL);
    assert(velocity.config.automatic_motion_budget_ms == 600000U);
    assert(strstr(g_tx, "PID_SET_OK,name=BUDGET,value=0") != NULL);
    assert(strstr(g_tx, "PID_SET_OK,name=BUDGET,value=600000") != NULL);
    assert(strstr(g_tx, "PID_SET_REJECTED,name=BUDGET,value=600001") != NULL);
}

static void test_runtime_ball_limits(void)
{
    PitchPidDebug debug;
    BspBluetooth bluetooth;
    PitchAxisVisionControl vision;
    PitchAxisVelocityTest velocity;

    initialize(&debug, &bluetooth, &vision, &velocity);
    feed("SET BALLMIN=-1050\r\nSET BALLMAX=1050\r\nSET TARGET=1200\r\nSET BALLMAX=-1100\r\n");
    service_until_input_drained(&debug, 10U);
    assert(vision.config.minimum_safe_position_0_1mm == -1050);
    assert(vision.config.maximum_safe_position_0_1mm == 1050);
    assert(vision.config.target_position_0_1mm == -50);
    assert(strstr(g_tx, "PID_SET_OK,name=BALLMIN,value=-1050") != NULL);
    assert(strstr(g_tx, "PID_SET_OK,name=BALLMAX,value=1050") != NULL);
    assert(strstr(g_tx, "PID_SET_REJECTED,name=TARGET,value=1200") != NULL);
    assert(strstr(g_tx, "PID_SET_REJECTED,name=BALLMAX,value=-1100") != NULL);
}

static void test_feedforward_settings(void)
{
    PitchPidDebug debug;
    BspBluetooth bluetooth;
    PitchAxisVisionControl vision;
    PitchAxisVelocityTest velocity;

    initialize(&debug, &bluetooth, &vision, &velocity);
    feed("SET FF=0.006\r\nSET FFSIGN=-1\r\nSET FFLIM=4.000\r\nSET FFDB=75\r\nSET FFEN=1\r\nSET FFSIGN=0\r\nSET FF=0.501\r\n");
    service_until_input_drained(&debug, 10U);
    assert(vision.config.feedforward_enabled);
    assert(vision.config.feedforward_sign == -1);
    assert(vision.config.feedforward_gain_rpm_per_mm_s2 > 0.005f);
    assert(vision.config.feedforward_gain_rpm_per_mm_s2 < 0.007f);
    assert(vision.config.feedforward_limit_rpm > 3.999f);
    assert(vision.config.feedforward_limit_rpm < 4.001f);
    assert(vision.config.feedforward_deadband_mm_s2 > 74.999f);
    assert(vision.config.feedforward_deadband_mm_s2 < 75.001f);
    assert(strstr(g_tx, "PID_SET_OK,name=FF,value=0.006") != NULL);
    assert(strstr(g_tx, "PID_SET_OK,name=FFSIGN,value=-1") != NULL);
    assert(strstr(g_tx, "PID_SET_OK,name=FFLIM,value=4.000") != NULL);
    assert(strstr(g_tx, "PID_SET_OK,name=FFDB,value=75") != NULL);
    assert(strstr(g_tx, "PID_SET_OK,name=FFEN,value=1") != NULL);
    assert(strstr(g_tx, "PID_SET_REJECTED,name=FFSIGN,value=0") != NULL);
    assert(strstr(g_tx, "PID_SET_REJECTED,name=FF,value=0.501") != NULL);
}

static void test_fragmented_db_is_not_disarm(void)
{
    PitchPidDebug debug;
    BspBluetooth bluetooth;
    PitchAxisVisionControl vision;
    PitchAxisVelocityTest velocity;

    initialize(&debug, &bluetooth, &vision, &velocity);
    feed("D");
    PitchPidDebug_Service(&debug, 0U);
    assert(g_disarm_calls == 0U);
    feed("B=25\r\n");
    PitchPidDebug_Service(&debug, 10U);
    assert(g_disarm_calls == 0U);
    assert(vision.config.deadband_0_1mm == 25);
}

static void test_bare_arm_timeout_and_resume_reset(void)
{
    PitchPidDebug debug;
    BspBluetooth bluetooth;
    PitchAxisVisionControl vision;
    PitchAxisVelocityTest velocity;

    initialize(&debug, &bluetooth, &vision, &velocity);
    feed("A");
    PitchPidDebug_Service(&debug, 0U);
    assert(g_arm_calls == 0U);
    PitchPidDebug_Service(&debug, 31U);
    assert(g_arm_calls == 1U);
    assert(g_reset_calls == 1U);

    velocity.report.automatic_armed = false;
    velocity.report.automatic_hold = true;
    clear_output();
    feed("RESUME\r\n");
    PitchPidDebug_Service(&debug, 40U);
    assert(!velocity.report.automatic_hold);
    assert(g_reset_calls == 2U);
    assert(strstr(g_tx, "PID_RESUME_READY\r\n") != NULL);
}

static void test_zero_and_position_loop_settings(void)
{
    PitchPidDebug debug;
    BspBluetooth bluetooth;
    PitchAxisVisionControl vision;
    PitchAxisVelocityTest velocity;

    initialize(&debug, &bluetooth, &vision, &velocity);
    velocity.report.state = PITCH_VELOCITY_TEST_STATE_ENABLED_STOPPED;
    feed("ZERO\r\nSET POSTRACK=1\r\nSET RAWSIGN=1\r\nSET RAWPMM=30000\r\nSET TILTSCALE=60\r\nSET TILTLIM=1800\r\nSET POSDB=25\r\nSET SLOWUM=250\r\nSET INNERMIN=1\r\nSET POSPOLL=15\r\n");
    service_until_input_drained(&debug, 50U);

    assert(g_zero_calls == 1U);
    assert(velocity.config.automatic_position_tracking_enabled);
    assert(velocity.config.automatic_direction0_increases_raw);
    assert(velocity.config.automatic_position_raw_per_mm == 30000U);
    assert(velocity.config.automatic_tilt_scale_um_per_outer_rpm == 60U);
    assert(velocity.config.automatic_tilt_limit_um == 1800U);
    assert(velocity.config.automatic_position_deadband_um == 25U);
    assert(velocity.config.automatic_position_slow_zone_um == 250U);
    assert(velocity.config.automatic_position_poll_period_ms == 15U);
    assert(strstr(g_tx, "POSITION_ZERO_REQUESTED") != NULL);
}

static void test_task_parameters_and_status(void)
{
    PitchPidDebug debug;
    BspBluetooth bluetooth;
    PitchAxisVisionControl vision;
    PitchAxisVelocityTest velocity;
    PitchTaskController tasks;
    PitchTaskControllerConfig task_config = {
        .center_position_0_1mm = -50,
        .task3_offset_0_1mm = 500U,
        .task3_tolerance_0_1mm = 100U,
        .task3_velocity_limit_0_1mm_s = 100U,
        .task3_turnaround_dwell_ms = 100U,
        .position_hold_tilt_limit_um = 3900U,
        .task3_tilt_limit_um = 4000U,
        .button_debounce_ms = 30U
    };
    PitchTaskControllerConfig readback;

    initialize(&debug, &bluetooth, &vision, &velocity);
    assert(PitchTaskController_Init(
        &tasks, &vision, &velocity, &task_config, 0U));
    debug.tasks = &tasks;
    feed("SET CENTER=-40\r\nSET T3OFFSET=480\r\nSET T3TOL=80\r\nSET T3VMAX=90\r\nSET T3DWELL=120\r\nSET HOLDTILT=3900\r\nSET T3TILT=4000\r\nPID?\r\n");
    service_until_input_drained(&debug, 10U);
    service_until_config_drained(&debug, 20U);

    assert(PitchTaskController_GetConfig(&tasks, &readback));
    assert(readback.center_position_0_1mm == -40);
    assert(readback.task3_offset_0_1mm == 480U);
    assert(readback.task3_tolerance_0_1mm == 80U);
    assert(readback.task3_velocity_limit_0_1mm_s == 90U);
    assert(readback.task3_turnaround_dwell_ms == 120U);
    assert(readback.position_hold_tilt_limit_um == 3900U);
    assert(readback.task3_tilt_limit_um == 4000U);
    assert(strstr(g_tx, "PID_SET_OK,name=CENTER,value=-40") != NULL);
    assert(strstr(g_tx, "PITCH_TASK,task=2,state=IDLE") != NULL);
    assert(strstr(g_tx, ",task=2,task_state=IDLE") != NULL);
    assert(strstr(
        g_tx,
        ",center=-40,t3offset=480,t3tol=80,t3vmax=90,t3dwell=120,holdtilt=3900,t3tilt=4000") != NULL);
}

static void test_task_mode_rejects_bluetooth_arm_and_disarm(void)
{
    PitchPidDebug debug;
    BspBluetooth bluetooth;
    PitchAxisVisionControl vision;
    PitchAxisVelocityTest velocity;
    PitchTaskController tasks;
    PitchTaskControllerConfig task_config = {
        .center_position_0_1mm = -50,
        .task3_offset_0_1mm = 500U,
        .task3_tolerance_0_1mm = 100U,
        .task3_velocity_limit_0_1mm_s = 100U,
        .task3_turnaround_dwell_ms = 100U,
        .position_hold_tilt_limit_um = 3900U,
        .task3_tilt_limit_um = 4000U,
        .button_debounce_ms = 30U
    };

    initialize(&debug, &bluetooth, &vision, &velocity);
    assert(PitchTaskController_Init(
        &tasks, &vision, &velocity, &task_config, 0U));
    debug.tasks = &tasks;
    feed("A\r\nD\r\n");
    service_until_input_drained(&debug, 10U);

    assert(g_arm_calls == 0U);
    assert(g_disarm_calls == 0U);
    assert(strstr(g_tx, "AUTO_ARM_REJECTED_USE_KEY1\r\n") != NULL);
    assert(strstr(g_tx, "AUTO_DISARM_REJECTED_USE_KEY4\r\n") != NULL);
}

int main(void)
{
    test_ping_pid_on_and_status();
    test_live_pid_and_automatic_limit_settings();
    test_runtime_ball_limits();
    test_feedforward_settings();
    test_fragmented_db_is_not_disarm();
    test_bare_arm_timeout_and_resume_reset();
    test_zero_and_position_loop_settings();
    test_task_parameters_and_status();
    test_task_mode_rejects_bluetooth_arm_and_disarm();
    puts("PITCH_PID_DEBUG_TEST=PASS");
    return 0;
}
