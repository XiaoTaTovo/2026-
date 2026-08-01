#include "pitch_task_controller.h"

#include <stddef.h>
#include <string.h>

static bool time_elapsed(uint32_t now_ms, uint32_t start_ms, uint32_t delay_ms)
{
    return (uint32_t)(now_ms - start_ms) >= delay_ms;
}

static uint16_t abs_i16(int16_t value)
{
    return (value < 0) ? (uint16_t)(-(int32_t)value) : (uint16_t)value;
}

static bool valid_config(const PitchTaskControllerConfig *config)
{
    return (config != NULL) &&
        (config->task3_offset_0_1mm > 0U) &&
        (config->task3_tolerance_0_1mm > 0U) &&
        (config->task3_tolerance_0_1mm <= config->task3_offset_0_1mm) &&
        (config->task3_velocity_limit_0_1mm_s > 0U) &&
        (config->task3_turnaround_dwell_ms <= 5000U) &&
        (config->position_hold_tilt_limit_um > 0U) &&
        (config->task3_tilt_limit_um > 0U) &&
        (config->button_debounce_ms <= 500U);
}

static bool task_targets_fit(
    PitchAxisVisionControl *vision,
    const PitchTaskControllerConfig *config)
{
    PitchAxisVisionConfig vision_config;
    int32_t positive;
    int32_t negative;

    if (!PitchAxisVisionControl_GetConfig(vision, &vision_config))
    {
        return false;
    }
    positive = (int32_t)config->center_position_0_1mm +
        (int32_t)config->task3_offset_0_1mm;
    negative = (int32_t)config->center_position_0_1mm -
        (int32_t)config->task3_offset_0_1mm;
    return (negative >= vision_config.minimum_safe_position_0_1mm) &&
        (positive <= vision_config.maximum_safe_position_0_1mm);
}

static bool velocity_report(
    const PitchTaskController *controller,
    PitchAxisVelocityTestReport *report)
{
    return PitchAxisVelocityTest_GetReport(controller->velocity, report);
}

static bool vision_report(
    const PitchTaskController *controller,
    PitchAxisVisionReport *report)
{
    return PitchAxisVisionControl_GetReport(controller->vision, report);
}

static void sync_report(
    PitchTaskController *controller,
    const PitchAxisVelocityTestReport *velocity,
    uint32_t now_ms)
{
    controller->report.automatic_armed = velocity->automatic_armed;
    controller->report.motor_enabled = velocity->enabled;
    controller->report.fault_latched = velocity->fault_latched;
    if (velocity->fault_latched &&
        (controller->report.state != PITCH_TASK_STATE_FAULT))
    {
        controller->report.state = PITCH_TASK_STATE_FAULT;
        controller->state_since_ms = now_ms;
        controller->target_window_since_ms = 0U;
        controller->report.transition_count++;
    }
    controller->report.task_elapsed_ms =
        now_ms - controller->state_since_ms;
}

static bool set_target(
    PitchTaskController *controller,
    int16_t target_position_0_1mm,
    uint32_t now_ms)
{
    PitchAxisVisionConfig vision_config;

    if (!PitchAxisVisionControl_GetConfig(
            controller->vision,
            &vision_config))
    {
        return false;
    }
    if ((target_position_0_1mm <
         vision_config.minimum_safe_position_0_1mm) ||
        (target_position_0_1mm >
         vision_config.maximum_safe_position_0_1mm))
    {
        return false;
    }
    vision_config.target_position_0_1mm = target_position_0_1mm;
    if (!PitchAxisVisionControl_UpdateConfig(
            controller->vision,
            &vision_config,
            now_ms))
    {
        return false;
    }
    controller->report.target_position_0_1mm = target_position_0_1mm;
    controller->target_window_since_ms = 0U;
    return true;
}

static int16_t center_target(const PitchTaskController *controller)
{
    return controller->config.center_position_0_1mm;
}

static int16_t positive_target(const PitchTaskController *controller)
{
    return (int16_t)((int32_t)controller->config.center_position_0_1mm +
                     (int32_t)controller->config.task3_offset_0_1mm);
}

static int16_t negative_target(const PitchTaskController *controller)
{
    return (int16_t)((int32_t)controller->config.center_position_0_1mm -
                     (int32_t)controller->config.task3_offset_0_1mm);
}

static void enter_state(
    PitchTaskController *controller,
    PitchTaskState state,
    uint32_t now_ms)
{
    controller->report.state = state;
    controller->state_since_ms = now_ms;
    controller->target_window_since_ms = 0U;
    controller->report.transition_count++;
}

static bool disarm_and_reset(
    PitchTaskController *controller,
    uint32_t now_ms)
{
    bool ok = PitchAxisVelocityTest_SetAutomaticArmed(
        controller->velocity,
        false,
        now_ms);
    PitchAxisVisionControl_ResetController(controller->vision, now_ms);
    return ok;
}

static PitchTaskPidProfile *selected_pid_profile_config(
    PitchTaskController *controller);

static bool apply_selected_task_velocity_config(
    PitchTaskController *controller)
{
    PitchAxisVelocityTestConfig velocity_config;
    uint16_t tilt_limit_um;

    if (!PitchAxisVelocityTest_GetConfig(
            controller->velocity,
            &velocity_config))
    {
        return false;
    }
    if (controller->report.selected_task == PITCH_TASK_3)
    {
        tilt_limit_um = selected_pid_profile_config(controller)
            ->task3_tilt_limit_um;
        controller->config.task3_tilt_limit_um = tilt_limit_um;
    }
    else
    {
        tilt_limit_um = controller->config.position_hold_tilt_limit_um;
    }
    velocity_config.automatic_tilt_limit_um = tilt_limit_um;
    return PitchAxisVelocityTest_UpdateConfig(
        controller->velocity,
        &velocity_config);
}

static PitchTaskPidProfile *selected_pid_profile_config(
    PitchTaskController *controller)
{
    return (controller->report.selected_task == PITCH_TASK_3) ?
        &controller->task3_pid_profiles[controller->task3_pid_profile] :
        &controller->position_hold_pid_profiles
            [controller->position_hold_pid_profile];
}

static uint8_t selected_pid_profile(const PitchTaskController *controller)
{
    return (controller->report.selected_task == PITCH_TASK_3) ?
        controller->task3_pid_profile :
        controller->position_hold_pid_profile;
}

static void copy_pid_to_profile(
    PitchTaskPidProfile *profile,
    const PitchAxisVisionConfig *config)
{
    profile->kp_rpm_per_mm = config->kp_rpm_per_mm;
    profile->ki_rpm_per_mm_s = config->ki_rpm_per_mm_s;
    profile->kd_rpm_per_mm_s = config->kd_rpm_per_mm_s;
    profile->integral_limit_rpm = config->integral_limit_rpm;
    profile->integral_separation_band_0_1mm =
        config->integral_separation_band_0_1mm;
    profile->velocity_filter_alpha = config->velocity_filter_alpha;
    profile->approach_band_0_1mm = config->approach_band_0_1mm;
    profile->approach_speed_limit_rpm = config->approach_speed_limit_rpm;
    profile->maximum_speed_rpm = config->maximum_speed_rpm;
}

static void copy_profile_to_pid(
    PitchAxisVisionConfig *config,
    const PitchTaskPidProfile *profile)
{
    config->kp_rpm_per_mm = profile->kp_rpm_per_mm;
    config->ki_rpm_per_mm_s = profile->ki_rpm_per_mm_s;
    config->kd_rpm_per_mm_s = profile->kd_rpm_per_mm_s;
    config->integral_limit_rpm = profile->integral_limit_rpm;
    config->integral_separation_band_0_1mm =
        profile->integral_separation_band_0_1mm;
    config->velocity_filter_alpha = profile->velocity_filter_alpha;
    config->approach_band_0_1mm = profile->approach_band_0_1mm;
    config->approach_speed_limit_rpm = profile->approach_speed_limit_rpm;
    config->maximum_speed_rpm = profile->maximum_speed_rpm;
}

static bool valid_task3_pid_profile(const PitchTaskPidProfile *profile)
{
    return (profile->kp_rpm_per_mm >= 0.0f) &&
        (profile->ki_rpm_per_mm_s >= 0.0f) &&
        (profile->kd_rpm_per_mm_s >= 0.0f) &&
        (profile->integral_limit_rpm >= 0.0f) &&
        (profile->integral_separation_band_0_1mm >= 0) &&
        (profile->velocity_filter_alpha >= 0.0f) &&
        (profile->velocity_filter_alpha <= 1.0f) &&
        (profile->approach_band_0_1mm >= 0) &&
        (profile->approach_speed_limit_rpm > 0U) &&
        (profile->maximum_speed_rpm > 0U) &&
        (profile->task3_tilt_limit_um >= 100U);
}

static bool apply_selected_task_pid_config(
    PitchTaskController *controller,
    uint32_t now_ms)
{
    PitchAxisVisionConfig live_config;
    PitchTaskPidProfile *selected_profile;

    if (!PitchAxisVisionControl_GetConfig(
            controller->vision, &live_config))
    {
        return false;
    }
    selected_profile = selected_pid_profile_config(controller);
    copy_profile_to_pid(&live_config, selected_profile);
    return PitchAxisVisionControl_UpdateConfig(
        controller->vision, &live_config, now_ms);
}

static bool cycle_selected_pid_profile(
    PitchTaskController *controller,
    uint32_t now_ms)
{
    uint8_t *profile =
        (controller->report.selected_task == PITCH_TASK_3) ?
        &controller->task3_pid_profile :
        &controller->position_hold_pid_profile;

    *profile = (uint8_t)((*profile + 1U) % PITCH_TASK_PID_PROFILE_COUNT);
    controller->report.pid_profile = (uint8_t)(*profile + 1U);
    if (!apply_selected_task_pid_config(controller, now_ms) ||
        !apply_selected_task_velocity_config(controller))
    {
        return false;
    }
    controller->report.transition_count++;
    return true;
}

static bool task_is_center_hold(PitchTaskId task)
{
    return (task == PITCH_TASK_2) ||
        (task == PITCH_TASK_4) ||
        (task == PITCH_TASK_5);
}

static void select_next_task(
    PitchTaskController *controller,
    uint32_t now_ms)
{
    PitchTaskId next_task;

    (void)disarm_and_reset(controller, now_ms);
    next_task = (controller->report.selected_task >= PITCH_TASK_6) ?
        PITCH_TASK_2 :
        (PitchTaskId)((uint8_t)controller->report.selected_task + 1U);
    controller->report.selected_task = next_task;
    controller->report.pid_profile =
        (uint8_t)(selected_pid_profile(controller) + 1U);
    controller->start_pending = false;
    controller->report.captured_position_valid = false;
    controller->report.captured_position_0_1mm = 0;
    if (!apply_selected_task_pid_config(controller, now_ms) ||
        !apply_selected_task_velocity_config(controller))
    {
        enter_state(controller, PITCH_TASK_STATE_FAULT, now_ms);
        return;
    }
    if (next_task == PITCH_TASK_6)
    {
        if (!set_target(controller, center_target(controller), now_ms))
        {
            enter_state(controller, PITCH_TASK_STATE_FAULT, now_ms);
            return;
        }
        enter_state(controller, PITCH_TASK_STATE_WAIT_CAPTURE, now_ms);
    }
    else
    {
        if (!set_target(controller, center_target(controller), now_ms))
        {
            enter_state(controller, PITCH_TASK_STATE_FAULT, now_ms);
            return;
        }
        enter_state(controller, PITCH_TASK_STATE_IDLE, now_ms);
    }
}

static void capture_task6_target(
    PitchTaskController *controller,
    uint32_t now_ms)
{
    PitchAxisVisionReport report;

    if (!vision_report(controller, &report) ||
        (report.state != PITCH_VISION_STATE_TRACKING) ||
        !report.observation_fresh ||
        !report.observation.valid)
    {
        controller->report.rejected_key_count++;
        return;
    }
    if (!set_target(controller, report.observation.x_0_1mm, now_ms))
    {
        controller->report.rejected_key_count++;
        return;
    }
    controller->report.captured_position_0_1mm =
        report.observation.x_0_1mm;
    controller->report.captured_position_valid = true;
    enter_state(controller, PITCH_TASK_STATE_IDLE, now_ms);
}

static bool start_selected_task(
    PitchTaskController *controller,
    uint32_t now_ms)
{
    int16_t target;
    bool armed;

    if ((controller->report.state != PITCH_TASK_STATE_IDLE) ||
        controller->report.automatic_armed)
    {
        return false;
    }
    if ((controller->report.selected_task == PITCH_TASK_6) &&
        !controller->report.captured_position_valid)
    {
        return false;
    }

    if (controller->report.selected_task == PITCH_TASK_3)
    {
        target = positive_target(controller);
    }
    else if (controller->report.selected_task == PITCH_TASK_6)
    {
        target = controller->report.captured_position_0_1mm;
    }
    else if (task_is_center_hold(controller->report.selected_task))
    {
        target = center_target(controller);
    }
    else
    {
        return false;
    }
    if (!apply_selected_task_pid_config(controller, now_ms) ||
        !apply_selected_task_velocity_config(controller))
    {
        return false;
    }
    if (!set_target(controller, target, now_ms))
    {
        return false;
    }
    PitchAxisVisionControl_ResetController(controller->vision, now_ms);
    (void)PitchAxisVelocityTest_ClearAutomaticHold(controller->velocity);
    armed = PitchAxisVelocityTest_SetAutomaticArmed(
        controller->velocity,
        true,
        now_ms);
    if (!armed)
    {
        return false;
    }
    enter_state(controller, PITCH_TASK_STATE_STARTING, now_ms);
    return true;
}

static void service_pending_start(
    PitchTaskController *controller,
    const PitchAxisVelocityTestReport *velocity,
    uint32_t now_ms)
{
    if (!controller->start_pending || velocity->fault_latched ||
        velocity->automatic_armed)
    {
        return;
    }
    if ((velocity->state != PITCH_VELOCITY_TEST_STATE_DISABLED_READY) &&
        (velocity->state != PITCH_VELOCITY_TEST_STATE_ENABLED_STOPPED))
    {
        return;
    }
    if (start_selected_task(controller, now_ms))
    {
        controller->start_pending = false;
    }
    else
    {
        controller->start_pending = false;
        controller->report.rejected_key_count++;
    }
}

static uint8_t update_buttons(
    PitchTaskController *controller,
    PitchAxisVelocityTestButtons buttons,
    uint32_t now_ms)
{
    const bool raw[3] = {
        buttons.key1_pressed,
        buttons.key2_pressed,
        buttons.key3_pressed
    };
    uint8_t edges = 0U;
    uint8_t index;

    for (index = 0U; index < 3U; ++index)
    {
        PitchTaskButtonDebouncer *button = &controller->buttons[index];
        if (raw[index] != button->candidate_pressed)
        {
            button->candidate_pressed = raw[index];
            button->candidate_since_ms = now_ms;
        }
        else if ((button->stable_pressed != button->candidate_pressed) &&
                 time_elapsed(
                     now_ms,
                     button->candidate_since_ms,
                     controller->config.button_debounce_ms))
        {
            button->stable_pressed = button->candidate_pressed;
            if (button->stable_pressed)
            {
                edges |= (uint8_t)(1U << index);
            }
        }
    }
    return edges;
}

static void service_trajectory(
    PitchTaskController *controller,
    const PitchAxisVisionReport *vision,
    uint32_t now_ms)
{
    bool settled_at_target;

    if (!controller->report.automatic_armed ||
        !vision->observation_fresh ||
        (vision->state != PITCH_VISION_STATE_TRACKING))
    {
        return;
    }
    if (controller->report.state == PITCH_TASK_STATE_STARTING)
    {
        if (controller->report.selected_task == PITCH_TASK_3)
        {
            enter_state(
                controller,
                PITCH_TASK_STATE_RUNNING_POSITIVE,
                now_ms);
        }
        else
        {
            enter_state(controller, PITCH_TASK_STATE_HOLDING, now_ms);
        }
    }
    if ((controller->report.state != PITCH_TASK_STATE_RUNNING_POSITIVE) &&
        (controller->report.state != PITCH_TASK_STATE_RUNNING_NEGATIVE))
    {
        return;
    }
    if (controller->report.state == PITCH_TASK_STATE_RUNNING_POSITIVE)
    {
        if (vision->observation.x_0_1mm < positive_target(controller))
        {
            return;
        }
        controller->report.target_reached_count++;
        if (set_target(controller, negative_target(controller), now_ms))
        {
            enter_state(
                controller,
                PITCH_TASK_STATE_RUNNING_NEGATIVE,
                now_ms);
        }
        return;
    }

    settled_at_target =
        (abs_i16(vision->error_0_1mm) <=
         controller->config.task3_tolerance_0_1mm) &&
        (abs_i16(vision->ball_velocity_0_1mm_s) <=
         controller->config.task3_velocity_limit_0_1mm_s);
    if (!settled_at_target)
    {
        controller->target_window_since_ms = 0U;
        return;
    }
    if (controller->target_window_since_ms == 0U)
    {
        controller->target_window_since_ms = now_ms;
        return;
    }
    if (!time_elapsed(
            now_ms,
            controller->target_window_since_ms,
            controller->config.task3_turnaround_dwell_ms))
    {
        return;
    }
    controller->report.target_reached_count++;
    enter_state(controller, PITCH_TASK_STATE_HOLDING, now_ms);
}

bool PitchTaskController_Init(
    PitchTaskController *controller,
    PitchAxisVisionControl *vision,
    PitchAxisVelocityTest *velocity,
    const PitchTaskControllerConfig *config,
    uint32_t now_ms)
{
    uint8_t index;
    PitchAxisVisionConfig initial_pid_config;

    if ((controller == NULL) || (vision == NULL) || (velocity == NULL) ||
        !valid_config(config) || !task_targets_fit(vision, config))
    {
        return false;
    }
    memset(controller, 0, sizeof(*controller));
    controller->vision = vision;
    controller->velocity = velocity;
    controller->config = *config;
    if (!PitchAxisVisionControl_GetConfig(vision, &initial_pid_config))
    {
        return false;
    }
    for (index = 0U; index < PITCH_TASK_PID_PROFILE_COUNT; ++index)
    {
        copy_pid_to_profile(
            &controller->position_hold_pid_profiles[index],
            &initial_pid_config);
        if (valid_task3_pid_profile(&config->task3_pid_profiles[index]))
        {
            controller->task3_pid_profiles[index] =
                config->task3_pid_profiles[index];
        }
        else
        {
            copy_pid_to_profile(
                &controller->task3_pid_profiles[index],
                &initial_pid_config);
            controller->task3_pid_profiles[index].task3_tilt_limit_um =
                config->task3_tilt_limit_um;
        }
    }
    controller->report.selected_task = PITCH_TASK_2;
    controller->task3_pid_profile = 1U;
    controller->report.pid_profile = 1U;
    controller->report.state = PITCH_TASK_STATE_IDLE;
    controller->state_since_ms = now_ms;
    for (index = 0U; index < 3U; ++index)
    {
        controller->buttons[index].candidate_since_ms = now_ms;
    }
    controller->initialized = true;
    if (!apply_selected_task_pid_config(controller, now_ms) ||
        !apply_selected_task_velocity_config(controller))
    {
        controller->initialized = false;
        return false;
    }
    if (!set_target(controller, center_target(controller), now_ms))
    {
        controller->initialized = false;
        return false;
    }
    return true;
}

void PitchTaskController_Service(
    PitchTaskController *controller,
    PitchAxisVelocityTestButtons buttons,
    uint32_t now_ms)
{
    PitchAxisVelocityTestReport velocity;
    PitchAxisVisionReport vision;
    uint8_t edges;

    if ((controller == NULL) || !controller->initialized ||
        !velocity_report(controller, &velocity))
    {
        return;
    }
    sync_report(controller, &velocity, now_ms);
    if (controller->report.state == PITCH_TASK_STATE_FAULT)
    {
        return;
    }
    edges = update_buttons(controller, buttons, now_ms);
    if (edges != 0U)
    {
        if ((edges & (uint8_t)(edges - 1U)) != 0U)
        {
            controller->report.rejected_key_count++;
        }
        else if ((edges & 0x02U) != 0U)
        {
            controller->report.last_key = 2U;
            select_next_task(controller, now_ms);
        }
        else if ((edges & 0x01U) != 0U)
        {
            controller->report.last_key = 1U;
            if ((controller->report.state == PITCH_TASK_STATE_IDLE) &&
                !controller->start_pending)
            {
                controller->start_pending = true;
            }
            else
            {
                controller->report.rejected_key_count++;
            }
        }
        else if ((edges & 0x04U) != 0U)
        {
            controller->report.last_key = 3U;
            if (controller->report.selected_task == PITCH_TASK_6 &&
                !controller->report.captured_position_valid &&
                (controller->report.state == PITCH_TASK_STATE_WAIT_CAPTURE ||
                 controller->report.state == PITCH_TASK_STATE_IDLE))
            {
                capture_task6_target(controller, now_ms);
            }
            else if (!cycle_selected_pid_profile(controller, now_ms))
            {
                enter_state(controller, PITCH_TASK_STATE_FAULT, now_ms);
            }
            else
            {
                PitchAxisVisionControl_ResetController(
                    controller->vision, now_ms);
            }
        }
    }
    if (!vision_report(controller, &vision))
    {
        return;
    }
    sync_report(controller, &velocity, now_ms);
    service_pending_start(controller, &velocity, now_ms);
    service_trajectory(controller, &vision, now_ms);
}

bool PitchTaskController_GetReport(
    const PitchTaskController *controller,
    PitchTaskControllerReport *report)
{
    if ((controller == NULL) || (report == NULL) || !controller->initialized)
    {
        return false;
    }
    *report = controller->report;
    return true;
}

bool PitchTaskController_GetConfig(
    const PitchTaskController *controller,
    PitchTaskControllerConfig *config)
{
    if ((controller == NULL) || (config == NULL) || !controller->initialized)
    {
        return false;
    }
    *config = controller->config;
    return true;
}

bool PitchTaskController_UpdateConfig(
    PitchTaskController *controller,
    const PitchTaskControllerConfig *config,
    uint32_t now_ms)
{
    PitchAxisVelocityTestReport velocity;

    if ((controller == NULL) || !controller->initialized ||
        !valid_config(config) || !velocity_report(controller, &velocity) ||
        !task_targets_fit(controller->vision, config) ||
        velocity.automatic_armed ||
        ((velocity.state != PITCH_VELOCITY_TEST_STATE_DISABLED_READY) &&
         (velocity.state != PITCH_VELOCITY_TEST_STATE_ENABLED_STOPPED)))
    {
        return false;
    }
    controller->config = *config;
    if (controller->report.selected_task == PITCH_TASK_3)
    {
        controller->task3_pid_profiles[controller->task3_pid_profile]
            .task3_tilt_limit_um = config->task3_tilt_limit_um;
    }
    if (!apply_selected_task_velocity_config(controller))
    {
        return false;
    }
    if (controller->report.selected_task != PITCH_TASK_6)
    {
        return set_target(controller, center_target(controller), now_ms);
    }
    return true;
}

bool PitchTaskController_UpdateActivePidConfig(
    PitchTaskController *controller,
    const PitchAxisVisionConfig *config,
    uint32_t now_ms)
{
    PitchTaskPidProfile *stored_config;

    if ((controller == NULL) || (config == NULL) ||
        !controller->initialized ||
        !PitchAxisVisionControl_UpdateConfig(
            controller->vision, config, now_ms))
    {
        return false;
    }
    stored_config = selected_pid_profile_config(controller);
    copy_pid_to_profile(stored_config, config);
    return true;
}

const char *PitchTaskController_StateName(PitchTaskState state)
{
    switch (state)
    {
        case PITCH_TASK_STATE_IDLE: return "IDLE";
        case PITCH_TASK_STATE_WAIT_CAPTURE: return "WAIT_CAPTURE";
        case PITCH_TASK_STATE_STARTING: return "STARTING";
        case PITCH_TASK_STATE_RUNNING_POSITIVE: return "TASK3_POS";
        case PITCH_TASK_STATE_RUNNING_NEGATIVE: return "TASK3_NEG";
        case PITCH_TASK_STATE_HOLDING: return "HOLDING";
        case PITCH_TASK_STATE_FAULT: return "FAULT";
        default: return "UNINITIALIZED";
    }
}
