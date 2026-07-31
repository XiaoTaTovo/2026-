#include "pitch_axis_velocity_test.h"

#include <stddef.h>
#include <string.h>

static bool valid_config(const PitchAxisVelocityTestConfig *config)
{
    bool position_config_valid;

    if (config == NULL)
    {
        return false;
    }
    position_config_valid = !config->automatic_position_tracking_enabled ||
        ((config->automatic_position_raw_per_mm != 0U) &&
         (config->automatic_tilt_scale_um_per_outer_rpm != 0U) &&
         (config->automatic_tilt_limit_um != 0U) &&
         (config->automatic_position_deadband_um != 0U) &&
         (config->automatic_position_deadband_um <
          config->automatic_tilt_limit_um) &&
         (config->automatic_position_slow_zone_um >
          config->automatic_position_deadband_um) &&
         (config->automatic_position_slow_zone_um <=
          config->automatic_tilt_limit_um) &&
         (config->automatic_position_min_speed_rpm != 0U) &&
         (config->automatic_position_min_speed_rpm <=
          config->automatic_max_speed_rpm) &&
         (config->automatic_position_poll_period_ms != 0U));

    return
        (config->address != 0U) &&
        (config->positive_direction <= 1U) &&
        (config->negative_direction <= 1U) &&
        (config->positive_direction != config->negative_direction) &&
        (config->speed_rpm != 0U) &&
        (config->speed_rpm <= X42S_EMM_MAX_SPEED_RPM) &&
        (config->run_ms != 0U) &&
        (config->debounce_ms != 0U) &&
        (config->automatic_max_speed_rpm != 0U) &&
        (config->automatic_max_speed_rpm <=
         PITCH_AXIS_VELOCITY_TEST_HARD_AUTO_MAX_RPM) &&
        (config->automatic_decision_timeout_ms != 0U) &&
        (config->automatic_vision_loss_grace_ms <=
         PITCH_AXIS_VELOCITY_TEST_HARD_MAX_VISION_LOSS_GRACE_MS) &&
        (!config->automatic_edge_recovery_enabled ||
         ((config->automatic_edge_recovery_speed_rpm != 0U) &&
           (config->automatic_edge_recovery_speed_rpm <=
           PITCH_AXIS_VELOCITY_TEST_HARD_EDGE_RECOVERY_MAX_RPM) &&
           (config->automatic_edge_recovery_max_ms != 0U) &&
           (config->automatic_edge_recovery_max_ms <=
            PITCH_AXIS_VELOCITY_TEST_HARD_MAX_EDGE_RECOVERY_MS))) &&
        position_config_valid;
}

static bool time_elapsed(uint32_t now_ms, uint32_t start_ms, uint32_t delay_ms)
{
    return (uint32_t)(now_ms - start_ms) >= delay_ms;
}

static bool recoverable_vision_loss(PitchAxisAutomaticDisarmReason reason)
{
    return (reason == PITCH_AUTOMATIC_DISARM_BALL_ESCAPE) ||
        (reason == PITCH_AUTOMATIC_DISARM_VISION_INVALID) ||
        (reason == PITCH_AUTOMATIC_DISARM_VISION_LOW_CONFIDENCE) ||
        (reason == PITCH_AUTOMATIC_DISARM_VISION_STALE);
}

static void clear_automatic_edge_recovery(PitchAxisVelocityTest *test)
{
    test->report.automatic_edge_recovery_active = false;
    test->report.automatic_edge_recovery_age_ms = 0U;
    test->report.automatic_edge_recovery_speed_rpm = 0U;
    test->automatic_edge_recovery_started_ms = 0U;
}

static void clear_automatic_vision_loss(PitchAxisVelocityTest *test)
{
    test->report.automatic_vision_loss_active = false;
    test->report.automatic_vision_loss_age_ms = 0U;
    test->report.automatic_vision_loss_reason =
        PITCH_AUTOMATIC_DISARM_NONE;
    test->automatic_vision_loss_started_ms = 0U;
    clear_automatic_edge_recovery(test);
}

static void queue_event(
    PitchAxisVelocityTest *test,
    PitchAxisVelocityTestEventType type,
    uint8_t key,
    PitchAxisVelocityTestCommand command,
    uint8_t ack_status,
    int64_t value);

static void latch_fault_with_stop(
    PitchAxisVelocityTest *test,
    PitchAxisVelocityTestFailure failure,
    uint32_t now_ms);

static void request_automatic_position(
    PitchAxisVelocityTest *test,
    uint32_t now_ms);

static bool arm_automatic_internal(
    PitchAxisVelocityTest *test,
    uint32_t now_ms);

static void update_automatic_budget(
    PitchAxisVelocityTest *test,
    uint32_t now_ms)
{
    uint32_t elapsed;

    if (!test->automatic_motion_segment_active)
    {
        return;
    }
    elapsed = now_ms - test->automatic_motion_started_ms;
    if (elapsed > (UINT32_MAX - test->automatic_budget_used_ms))
    {
        test->automatic_budget_used_ms = UINT32_MAX;
    }
    else
    {
        test->automatic_budget_used_ms += elapsed;
    }
    test->automatic_motion_started_ms = now_ms;
    test->report.automatic_budget_used_ms = test->automatic_budget_used_ms;
}

static void disarm_automatic(
    PitchAxisVelocityTest *test,
    PitchAxisAutomaticDisarmReason reason)
{
    bool was_armed = test->report.automatic_armed;

    test->report.automatic_armed = false;
    test->automatic_decision_pending = false;
    test->report.automatic_disarm_reason = reason;
    test->automatic_position_target_dirty = false;
    clear_automatic_vision_loss(test);
    test->automatic_edge_recovery_available = false;
    if (!test->automatic_motion_segment_active)
    {
        test->report.automatic_speed_rpm = 0U;
    }
    if (was_armed)
    {
        queue_event(
            test,
            PITCH_VELOCITY_TEST_EVENT_AUTOMATIC_DISARMED,
            0U,
            PITCH_VELOCITY_TEST_COMMAND_AUTOMATIC_STOP,
            0U,
            (int64_t)reason);
    }
}

static int64_t raw_position_to_signed(X42sRawPosition position)
{
    int64_t magnitude = (int64_t)position.magnitude;

    return position.negative ? -magnitude : magnitude;
}

static uint64_t absolute_i64(int64_t value)
{
    return (value < 0) ? (uint64_t)(-(value + 1)) + 1U : (uint64_t)value;
}

static int64_t micrometers_to_raw(
    const PitchAxisVelocityTestConfig *config,
    uint32_t micrometers)
{
    uint64_t scaled =
        (uint64_t)micrometers * config->automatic_position_raw_per_mm;

    return (int64_t)((scaled + 500U) / 1000U);
}

static bool direction_increases_raw(
    const PitchAxisVelocityTestConfig *config,
    uint8_t direction)
{
    return (direction == 0U) ?
        config->automatic_direction0_increases_raw :
        !config->automatic_direction0_increases_raw;
}

static void update_automatic_position_target(PitchAxisVelocityTest *test)
{
    int32_t effort_0_01rpm = test->automatic_decision.outer_control_0_01rpm;
    uint32_t effort_magnitude_0_01rpm;
    uint64_t requested_lift_um;
    int64_t target_offset_raw;

    effort_magnitude_0_01rpm = (effort_0_01rpm < 0) ?
        (uint32_t)(-effort_0_01rpm) : (uint32_t)effort_0_01rpm;
    requested_lift_um =
        (uint64_t)effort_magnitude_0_01rpm *
        test->config.automatic_tilt_scale_um_per_outer_rpm;
    requested_lift_um = (requested_lift_um + 50U) / 100U;
    if (requested_lift_um > test->config.automatic_tilt_limit_um)
    {
        requested_lift_um = test->config.automatic_tilt_limit_um;
    }

    target_offset_raw = micrometers_to_raw(
        &test->config,
        (uint32_t)requested_lift_um);
    if ((target_offset_raw != 0) &&
        !direction_increases_raw(
            &test->config,
            test->automatic_decision.motor_direction))
    {
        target_offset_raw = -target_offset_raw;
    }

    test->report.automatic_target_offset_raw = target_offset_raw;
    test->report.automatic_target_position_raw =
        test->report.automatic_zero_position_raw + target_offset_raw;
    test->automatic_position_target_dirty = true;
}

static uint16_t automatic_position_speed(
    const PitchAxisVelocityTest *test,
    uint64_t error_raw)
{
    const PitchAxisVelocityTestConfig *config = &test->config;
    uint64_t error_um =
        (error_raw * 1000U + config->automatic_position_raw_per_mm / 2U) /
        config->automatic_position_raw_per_mm;
    uint32_t minimum_speed = config->automatic_position_min_speed_rpm;
    uint32_t maximum_speed = config->automatic_max_speed_rpm;
    uint32_t span_um =
        config->automatic_position_slow_zone_um -
        config->automatic_position_deadband_um;
    uint32_t speed;

    if (error_um >= config->automatic_position_slow_zone_um)
    {
        return (uint16_t)maximum_speed;
    }
    if (error_um <= config->automatic_position_deadband_um)
    {
        return 0U;
    }

    speed = minimum_speed +
        (uint32_t)(((error_um - config->automatic_position_deadband_um) *
                    (maximum_speed - minimum_speed) + span_um / 2U) /
                   span_um);
    if (speed > maximum_speed)
    {
        speed = maximum_speed;
    }
    return (uint16_t)speed;
}

static void queue_event(
    PitchAxisVelocityTest *test,
    PitchAxisVelocityTestEventType type,
    uint8_t key,
    PitchAxisVelocityTestCommand command,
    uint8_t ack_status,
    int64_t value)
{
    uint8_t next = (uint8_t)((test->event_head + 1U) %
                             PITCH_AXIS_VELOCITY_TEST_EVENT_QUEUE_SIZE);

    if (next == test->event_tail)
    {
        test->report.event_drop_count++;
        return;
    }

    test->events[test->event_head].type = type;
    test->events[test->event_head].key = key;
    test->events[test->event_head].command = command;
    test->events[test->event_head].ack_status = ack_status;
    test->events[test->event_head].value = value;
    test->event_head = next;
}

static uint8_t update_buttons(
    PitchAxisVelocityTest *test,
    uint32_t now_ms,
    PitchAxisVelocityTestButtons buttons)
{
    const bool raw[4] = {
        buttons.key1_pressed,
        buttons.key2_pressed,
        buttons.key3_pressed,
        buttons.key4_pressed
    };
    uint8_t press_edges = 0U;
    uint8_t index;

    for (index = 0U; index < 4U; ++index)
    {
        PitchAxisVelocityTestDebouncer *debouncer = &test->buttons[index];

        if (raw[index] != debouncer->candidate_pressed)
        {
            debouncer->candidate_pressed = raw[index];
            debouncer->candidate_since_ms = now_ms;
        }
        else if ((debouncer->stable_pressed !=
                  debouncer->candidate_pressed) &&
                 time_elapsed(
                     now_ms,
                     debouncer->candidate_since_ms,
                     test->config.debounce_ms))
        {
            debouncer->stable_pressed = debouncer->candidate_pressed;
            if (debouncer->stable_pressed)
            {
                press_edges |= (uint8_t)(1U << index);
            }
        }
    }

    return press_edges;
}

static void finalize_fault(
    PitchAxisVelocityTest *test,
    PitchAxisVelocityTestFailure failure)
{
    if (!test->report.fault_latched)
    {
        test->report.error_count++;
    }
    test->report.velocity_command_active = false;
    test->automatic_motion_segment_active = false;
    test->report.automatic_motion_active = false;
    test->report.automatic_speed_rpm = 0U;
    test->report.automatic_armed = false;
    clear_automatic_vision_loss(test);
    test->automatic_edge_recovery_available = false;
    test->automatic_start_pending = false;
    test->automatic_position_query_pending = false;
    test->automatic_position_target_dirty = false;
    test->automatic_zero_capture_pending = false;
    test->automatic_stop_after_position_query = false;
    test->report.automatic_disarm_reason = PITCH_AUTOMATIC_DISARM_FAULT;
    test->automatic_decision_pending = false;
    test->report.fault_latched = true;
    test->report.failure = failure;
    test->report.state = PITCH_VELOCITY_TEST_STATE_FAULT_LATCHED;
    queue_event(
        test,
        PITCH_VELOCITY_TEST_EVENT_FAULT_LATCHED,
        0U,
        PITCH_VELOCITY_TEST_COMMAND_STOP,
        0U,
        (int64_t)failure);
}

static void reject(
    PitchAxisVelocityTest *test,
    PitchAxisVelocityTestEventType event,
    uint8_t key)
{
    test->report.reject_count++;
    queue_event(
        test,
        event,
        key,
        PITCH_VELOCITY_TEST_COMMAND_NONE,
        0U,
        0);
}

static void latch_fault_with_stop(
    PitchAxisVelocityTest *test,
    PitchAxisVelocityTestFailure failure,
    uint32_t now_ms)
{
    X42sDriverResult result;

    if (test->report.fault_latched ||
        (test->report.state == PITCH_VELOCITY_TEST_STATE_STOPPING))
    {
        return;
    }

    /* A disabled, idle driver has nothing to stop and may not acknowledge a
     * stop frame. Latch the safe state locally instead of manufacturing a
     * STOP_TIMEOUT. */
    if (!test->report.enabled && !test->report.velocity_command_active)
    {
        finalize_fault(test, failure);
        return;
    }

    test->report.failure = failure;
    result = X42sDriver_RequestStop(
        test->driver,
        test->config.address,
        test->config.synchronize,
        now_ms);
    if (result != X42S_DRIVER_OK)
    {
        finalize_fault(test, failure);
        return;
    }

    test->report.last_command = PITCH_VELOCITY_TEST_COMMAND_STOP;
    test->report.command_count++;
    test->report.state = PITCH_VELOCITY_TEST_STATE_STOPPING;
    queue_event(
        test,
        PITCH_VELOCITY_TEST_EVENT_STOP_SENT,
        0U,
        PITCH_VELOCITY_TEST_COMMAND_STOP,
        0U,
        0);
}

static void request_enable(
    PitchAxisVelocityTest *test,
    bool enable,
    uint32_t now_ms)
{
    X42sDriverResult result = X42sDriver_RequestEnable(
        test->driver,
        test->config.address,
        enable,
        test->config.synchronize,
        now_ms);

    if (result != X42S_DRIVER_OK)
    {
        latch_fault_with_stop(
            test,
            PITCH_VELOCITY_TEST_FAILURE_REQUEST,
            now_ms);
        return;
    }

    test->report.last_command = enable ?
        PITCH_VELOCITY_TEST_COMMAND_ENABLE :
        PITCH_VELOCITY_TEST_COMMAND_DISABLE;
    test->report.command_count++;
    test->report.state = enable ?
        PITCH_VELOCITY_TEST_STATE_WAIT_ENABLE_ACK :
        PITCH_VELOCITY_TEST_STATE_WAIT_DISABLE_ACK;
    queue_event(
        test,
        enable ? PITCH_VELOCITY_TEST_EVENT_ENABLE_SENT :
                 PITCH_VELOCITY_TEST_EVENT_DISABLE_SENT,
        0U,
        test->report.last_command,
        0U,
        0);
}

static void request_position_before(
    PitchAxisVelocityTest *test,
    PitchAxisVelocityTestCommand motion_command,
    uint32_t now_ms)
{
    X42sDriverResult result = X42sDriver_RequestReadPosition(
        test->driver,
        test->config.address,
        now_ms);

    if (result != X42S_DRIVER_OK)
    {
        latch_fault_with_stop(
            test,
            PITCH_VELOCITY_TEST_FAILURE_REQUEST,
            now_ms);
        return;
    }

    test->pending_motion_command = motion_command;
    test->report.state = PITCH_VELOCITY_TEST_STATE_WAIT_POSITION_BEFORE;
    queue_event(
        test,
        PITCH_VELOCITY_TEST_EVENT_POSITION_BEFORE_REQUESTED,
        0U,
        motion_command,
        0U,
        0);
}

static void request_velocity(
    PitchAxisVelocityTest *test,
    uint32_t now_ms)
{
    uint8_t direction =
        (test->pending_motion_command ==
         PITCH_VELOCITY_TEST_COMMAND_RUN_POSITIVE) ?
            test->config.positive_direction : test->config.negative_direction;
    X42sDriverResult result = X42sDriver_RequestEmmVelocity(
        test->driver,
        test->config.address,
        direction,
        test->config.speed_rpm,
        test->config.acceleration,
        test->config.synchronize,
        now_ms);

    if (result != X42S_DRIVER_OK)
    {
        latch_fault_with_stop(
            test,
            PITCH_VELOCITY_TEST_FAILURE_REQUEST,
            now_ms);
        return;
    }

    test->report.last_command = test->pending_motion_command;
    test->report.command_count++;
    test->report.state = PITCH_VELOCITY_TEST_STATE_WAIT_VELOCITY_ACK;
    queue_event(
        test,
        PITCH_VELOCITY_TEST_EVENT_VELOCITY_SENT,
        0U,
        test->pending_motion_command,
        0U,
        (int64_t)test->config.speed_rpm);
}

static void request_automatic_velocity(
    PitchAxisVelocityTest *test,
    uint32_t now_ms)
{
    uint16_t speed_rpm = test->automatic_decision.speed_rpm;
    X42sDriverResult result;

    if (speed_rpm == 0U)
    {
        test->automatic_decision_pending = false;
        return;
    }
    if (speed_rpm > test->config.automatic_max_speed_rpm)
    {
        speed_rpm = test->config.automatic_max_speed_rpm;
    }
    /* A same-direction F6 command updates the running speed. Preserve the
     * elapsed motion budget before the ACK starts the next accounting span. */
    update_automatic_budget(test, now_ms);
    result = X42sDriver_RequestEmmVelocity(
        test->driver,
        test->config.address,
        test->automatic_decision.motor_direction,
        speed_rpm,
        test->config.acceleration,
        test->config.synchronize,
        now_ms);
    if (result != X42S_DRIVER_OK)
    {
        latch_fault_with_stop(
            test,
            PITCH_VELOCITY_TEST_FAILURE_REQUEST,
            now_ms);
        return;
    }

    test->automatic_decision.speed_rpm = speed_rpm;
    test->report.automatic_direction =
        test->automatic_decision.motor_direction;
    test->report.automatic_speed_rpm = speed_rpm;
    test->report.last_command =
        PITCH_VELOCITY_TEST_COMMAND_AUTOMATIC_VELOCITY;
    test->report.command_count++;
    test->report.state =
        PITCH_VELOCITY_TEST_STATE_WAIT_AUTOMATIC_VELOCITY_ACK;
    test->automatic_decision_pending = false;
    queue_event(
        test,
        PITCH_VELOCITY_TEST_EVENT_AUTOMATIC_VELOCITY_SENT,
        0U,
        PITCH_VELOCITY_TEST_COMMAND_AUTOMATIC_VELOCITY,
        0U,
        (int64_t)speed_rpm);
}

static void request_automatic_stop(
    PitchAxisVelocityTest *test,
    uint32_t now_ms)
{
    X42sDriverResult result;

    /* Let an in-flight 0x36 reply finish before switching the single-response
     * parser to the STOP ACK. The measured query path is normally a few ms. */
    if (test->automatic_position_query_pending)
    {
        test->automatic_stop_after_position_query = true;
        return;
    }

    update_automatic_budget(test, now_ms);
    result = X42sDriver_RequestStop(
        test->driver,
        test->config.address,
        test->config.synchronize,
        now_ms);
    if (result != X42S_DRIVER_OK)
    {
        latch_fault_with_stop(
            test,
            PITCH_VELOCITY_TEST_FAILURE_REQUEST,
            now_ms);
        return;
    }

    test->automatic_motion_segment_active = false;
    test->automatic_stop_after_position_query = false;
    test->report.automatic_motion_active = false;
    test->report.automatic_speed_rpm = 0U;
    test->report.last_command =
        PITCH_VELOCITY_TEST_COMMAND_AUTOMATIC_STOP;
    test->report.command_count++;
    test->report.state =
        PITCH_VELOCITY_TEST_STATE_WAIT_AUTOMATIC_STOP_ACK;
    queue_event(
        test,
        PITCH_VELOCITY_TEST_EVENT_AUTOMATIC_STOP_SENT,
        0U,
        PITCH_VELOCITY_TEST_COMMAND_AUTOMATIC_STOP,
        0U,
        (int64_t)test->automatic_budget_used_ms);
}

static void request_timed_stop(
    PitchAxisVelocityTest *test,
    uint32_t now_ms)
{
    X42sDriverResult result = X42sDriver_RequestStop(
        test->driver,
        test->config.address,
        test->config.synchronize,
        now_ms);

    if (result != X42S_DRIVER_OK)
    {
        latch_fault_with_stop(
            test,
            PITCH_VELOCITY_TEST_FAILURE_REQUEST,
            now_ms);
        return;
    }

    test->report.last_command = PITCH_VELOCITY_TEST_COMMAND_STOP;
    test->report.command_count++;
    test->report.state = PITCH_VELOCITY_TEST_STATE_WAIT_STOP_ACK;
    queue_event(
        test,
        PITCH_VELOCITY_TEST_EVENT_AUTO_STOP_SENT,
        0U,
        PITCH_VELOCITY_TEST_COMMAND_STOP,
        0U,
        (int64_t)test->config.run_ms);
}

static void request_position_after(
    PitchAxisVelocityTest *test,
    uint32_t now_ms)
{
    X42sDriverResult result = X42sDriver_RequestReadPosition(
        test->driver,
        test->config.address,
        now_ms);

    if (result != X42S_DRIVER_OK)
    {
        finalize_fault(test, PITCH_VELOCITY_TEST_FAILURE_REQUEST);
        return;
    }

    test->report.state = PITCH_VELOCITY_TEST_STATE_WAIT_POSITION_AFTER;
    queue_event(
        test,
        PITCH_VELOCITY_TEST_EVENT_POSITION_AFTER_REQUESTED,
        0U,
        test->pending_motion_command,
        0U,
        0);
}

static bool automatic_motion_state(const PitchAxisVelocityTest *test)
{
    return test->report.velocity_command_active ||
           (test->report.state ==
            PITCH_VELOCITY_TEST_STATE_WAIT_AUTOMATIC_VELOCITY_ACK);
}

static bool automatic_command_pending(const PitchAxisVelocityTest *test)
{
    return (test->report.state ==
            PITCH_VELOCITY_TEST_STATE_WAIT_AUTOMATIC_VELOCITY_ACK) ||
           (test->report.state ==
            PITCH_VELOCITY_TEST_STATE_WAIT_AUTOMATIC_STOP_ACK);
}

static void request_automatic_position(
    PitchAxisVelocityTest *test,
    uint32_t now_ms)
{
    X42sDriverResult result;

    if (test->automatic_position_query_pending ||
        automatic_command_pending(test))
    {
        return;
    }

    result = X42sDriver_RequestReadPosition(
        test->driver,
        test->config.address,
        now_ms);
    if (result == X42S_DRIVER_BUSY)
    {
        return;
    }
    if (result != X42S_DRIVER_OK)
    {
        latch_fault_with_stop(
            test,
            PITCH_VELOCITY_TEST_FAILURE_REQUEST,
            now_ms);
        return;
    }

    test->automatic_position_query_pending = true;
    test->automatic_last_position_query_ms = now_ms;
    test->report.automatic_position_query_count++;
}

static void service_automatic_position_tracking(
    PitchAxisVelocityTest *test,
    uint32_t now_ms)
{
    int64_t position_error;
    int64_t position_offset;
    int64_t position_limit_raw;
    int64_t hard_margin_raw;
    uint64_t error_magnitude;
    uint16_t speed_rpm;
    uint8_t direction;
    bool motion_active = automatic_motion_state(test);
    bool stop_pending = test->report.state ==
        PITCH_VELOCITY_TEST_STATE_WAIT_AUTOMATIC_STOP_ACK;

    if (!test->report.automatic_zero_valid)
    {
        latch_fault_with_stop(
            test,
            PITCH_VELOCITY_TEST_FAILURE_POSITION_TIMEOUT,
            now_ms);
        return;
    }
    if (test->report.automatic_position_valid)
    {
        test->report.automatic_position_age_ms =
            now_ms - test->automatic_last_position_update_ms;
    }
    if (test->automatic_position_query_pending ||
        automatic_command_pending(test))
    {
        return;
    }
    if (!test->report.automatic_position_valid ||
        test->automatic_position_target_dirty ||
        time_elapsed(
            now_ms,
            test->automatic_last_position_query_ms,
            test->config.automatic_position_poll_period_ms))
    {
        request_automatic_position(test, now_ms);
        return;
    }

    position_limit_raw = micrometers_to_raw(
        &test->config,
        test->config.automatic_tilt_limit_um);
    hard_margin_raw = micrometers_to_raw(
        &test->config,
        test->config.automatic_position_slow_zone_um);
    position_offset = test->report.automatic_position_raw -
        test->report.automatic_zero_position_raw;
    if (absolute_i64(position_offset) >
        (uint64_t)(position_limit_raw + hard_margin_raw))
    {
        test->report.automatic_position_limit_count++;
        latch_fault_with_stop(
            test,
            PITCH_VELOCITY_TEST_FAILURE_POSITION_LIMIT,
            now_ms);
        return;
    }

    position_error = test->report.automatic_target_position_raw -
        test->report.automatic_position_raw;
    test->report.automatic_position_error_raw = position_error;
    error_magnitude = absolute_i64(position_error);
    speed_rpm = automatic_position_speed(test, error_magnitude);
    if (speed_rpm == 0U)
    {
        if (motion_active && !stop_pending)
        {
            request_automatic_stop(test, now_ms);
        }
        return;
    }

    direction = ((position_error > 0) ==
                 test->config.automatic_direction0_increases_raw) ? 0U : 1U;
    test->automatic_decision.motion_requested = true;
    test->automatic_decision.motor_direction = direction;
    test->automatic_decision.speed_rpm = speed_rpm;

    if (test->report.state == PITCH_VELOCITY_TEST_STATE_ENABLED_STOPPED)
    {
        request_automatic_velocity(test, now_ms);
        return;
    }
    if (test->report.state != PITCH_VELOCITY_TEST_STATE_RUNNING_AUTOMATIC)
    {
        return;
    }
    if (test->report.automatic_direction != direction)
    {
        request_automatic_stop(test, now_ms);
    }
    else if (test->report.automatic_speed_rpm != speed_rpm)
    {
        request_automatic_velocity(test, now_ms);
    }
}

static void request_edge_recovery(
    PitchAxisVelocityTest *test,
    uint32_t now_ms)
{
    uint16_t speed_rpm = test->config.automatic_edge_recovery_speed_rpm;

    if (speed_rpm > test->config.automatic_max_speed_rpm)
    {
        speed_rpm = test->config.automatic_max_speed_rpm;
    }
    test->automatic_decision.source_safe = true;
    test->automatic_decision.motion_requested = true;
    test->automatic_decision.motor_direction =
        test->automatic_edge_recovery_direction;
    test->automatic_decision.speed_rpm = speed_rpm;
    test->report.automatic_edge_recovery_active = true;
    test->report.automatic_edge_recovery_direction =
        test->automatic_edge_recovery_direction;
    test->report.automatic_edge_recovery_speed_rpm = speed_rpm;
    test->report.automatic_edge_recovery_age_ms = 0U;
    test->report.automatic_edge_recovery_count++;
    test->automatic_edge_recovery_started_ms = now_ms;
    request_automatic_velocity(test, now_ms);
}

static bool service_edge_recovery(
    PitchAxisVelocityTest *test,
    uint32_t now_ms,
    bool motion_active,
    bool stop_pending)
{
    if (!test->config.automatic_edge_recovery_enabled ||
        !test->automatic_edge_recovery_available)
    {
        return false;
    }

    if (test->report.automatic_edge_recovery_active)
    {
        test->report.automatic_edge_recovery_age_ms =
            now_ms - test->automatic_edge_recovery_started_ms;
        if (test->report.automatic_edge_recovery_age_ms >=
            test->config.automatic_edge_recovery_max_ms)
        {
            clear_automatic_edge_recovery(test);
            if (motion_active && !stop_pending)
            {
                request_automatic_stop(test, now_ms);
            }
        }
        return true;
    }

    if (!motion_active && !stop_pending &&
        (test->report.state == PITCH_VELOCITY_TEST_STATE_ENABLED_STOPPED))
    {
        request_edge_recovery(test, now_ms);
    }
    return true;
}

static void service_automatic_control(
    PitchAxisVelocityTest *test,
    uint32_t now_ms)
{
    bool motion_active;
    bool stop_pending;
    uint32_t budget_used;
    uint32_t motion_elapsed;
    uint16_t requested_speed;

    if (!test->report.automatic_armed || test->report.fault_latched)
    {
        return;
    }

    motion_active = automatic_motion_state(test);
    stop_pending = test->report.state ==
        PITCH_VELOCITY_TEST_STATE_WAIT_AUTOMATIC_STOP_ACK;
    budget_used = test->automatic_budget_used_ms;
    if (test->automatic_motion_segment_active)
    {
        motion_elapsed = now_ms - test->automatic_motion_started_ms;
        budget_used = (motion_elapsed > (UINT32_MAX - budget_used)) ?
            UINT32_MAX : budget_used + motion_elapsed;
    }
    test->report.automatic_budget_used_ms = budget_used;
    if ((test->config.automatic_motion_budget_ms != 0U) &&
        (budget_used >= test->config.automatic_motion_budget_ms))
    {
        disarm_automatic(test, PITCH_AUTOMATIC_DISARM_BUDGET);
        if (motion_active && !stop_pending)
        {
            request_automatic_stop(test, now_ms);
        }
        return;
    }

    if (test->automatic_decision_pending &&
        !test->automatic_decision.source_safe)
    {
        PitchAxisAutomaticDisarmReason reason =
            test->automatic_decision.unsafe_reason;

        if (recoverable_vision_loss(reason) &&
            (test->config.automatic_vision_loss_grace_ms != 0U))
        {
            if (!test->report.automatic_vision_loss_active)
            {
                test->report.automatic_vision_loss_active = true;
                test->report.automatic_vision_loss_reason = reason;
                test->automatic_vision_loss_started_ms = now_ms;
            }
            test->report.automatic_vision_loss_age_ms =
                now_ms - test->automatic_vision_loss_started_ms;
            test->automatic_decision_pending = false;
            if ((test->report.automatic_vision_loss_age_ms >=
                 test->config.automatic_vision_loss_grace_ms) &&
                !(!test->config.automatic_position_tracking_enabled &&
                  test->config.automatic_edge_recovery_enabled &&
                  test->automatic_edge_recovery_available))
            {
                disarm_automatic(test, reason);
                if (motion_active && !stop_pending)
                {
                    request_automatic_stop(test, now_ms);
                }
                return;
            }
            if (motion_active && !stop_pending &&
                !test->report.automatic_edge_recovery_active)
            {
                request_automatic_stop(test, now_ms);
                return;
            }
        }
        else
        {
            disarm_automatic(test, reason);
            if (motion_active && !stop_pending)
            {
                request_automatic_stop(test, now_ms);
            }
            return;
        }
    }

    if (test->automatic_decision_pending &&
        test->automatic_decision.source_safe &&
        test->report.automatic_vision_loss_active)
    {
        clear_automatic_vision_loss(test);
    }

    if (test->report.automatic_vision_loss_active)
    {
        PitchAxisAutomaticDisarmReason reason =
            test->report.automatic_vision_loss_reason;

        test->report.automatic_vision_loss_age_ms =
            now_ms - test->automatic_vision_loss_started_ms;
        if ((test->report.automatic_vision_loss_age_ms >=
             test->config.automatic_vision_loss_grace_ms) &&
            !(!test->config.automatic_position_tracking_enabled &&
              test->config.automatic_edge_recovery_enabled &&
              test->automatic_edge_recovery_available))
        {
            disarm_automatic(test, reason);
            if (motion_active && !stop_pending)
            {
                request_automatic_stop(test, now_ms);
            }
            return;
        }
        if (!test->config.automatic_position_tracking_enabled &&
            service_edge_recovery(
                test,
                now_ms,
                motion_active,
                stop_pending))
        {
            return;
        }
        return;
    }

    if (test->automatic_decision_pending &&
        test->automatic_decision.source_safe &&
        test->config.automatic_position_tracking_enabled)
    {
        update_automatic_position_target(test);
        test->automatic_decision_pending = false;
    }

    if (time_elapsed(
            now_ms,
            test->automatic_last_decision_ms,
            test->config.automatic_decision_timeout_ms))
    {
        disarm_automatic(
            test,
            PITCH_AUTOMATIC_DISARM_DECISION_TIMEOUT);
        if (motion_active && !stop_pending)
        {
            request_automatic_stop(test, now_ms);
        }
        return;
    }

    if (test->config.automatic_position_tracking_enabled)
    {
        service_automatic_position_tracking(test, now_ms);
        return;
    }

    if (!test->automatic_decision_pending)
    {
        return;
    }

    requested_speed = test->automatic_decision.speed_rpm;
    if (requested_speed > test->config.automatic_max_speed_rpm)
    {
        requested_speed = test->config.automatic_max_speed_rpm;
        test->automatic_decision.speed_rpm = requested_speed;
    }

    if (!test->automatic_decision.motion_requested ||
        (requested_speed == 0U))
    {
        test->automatic_decision_pending = false;
        if (motion_active && !stop_pending)
        {
            request_automatic_stop(test, now_ms);
        }
        return;
    }

    if (test->report.state == PITCH_VELOCITY_TEST_STATE_ENABLED_STOPPED)
    {
        request_automatic_velocity(test, now_ms);
        return;
    }

    if (stop_pending)
    {
        return;
    }

    if (test->report.state ==
        PITCH_VELOCITY_TEST_STATE_WAIT_AUTOMATIC_VELOCITY_ACK)
    {
        /* Keep the newest decision pending until the current F6 ACK arrives. */
        return;
    }

    if (test->report.state ==
        PITCH_VELOCITY_TEST_STATE_RUNNING_AUTOMATIC)
    {
        if (test->report.automatic_direction ==
            test->automatic_decision.motor_direction)
        {
            if (test->report.automatic_speed_rpm == requested_speed)
            {
                test->automatic_decision_pending = false;
            }
            else
            {
                request_automatic_velocity(test, now_ms);
            }
        }
        else
        {
            /* A direction reversal still requires an acknowledged stop. */
            request_automatic_stop(test, now_ms);
        }
    }
}

static void process_command_response(
    PitchAxisVelocityTest *test,
    uint32_t now_ms)
{
    X42sDriverState driver_state = X42sDriver_GetState(test->driver);

    if (driver_state == X42S_DRIVER_STATE_COMMAND_TIMEOUT)
    {
        test->report.timeout_count++;
        if (test->report.state == PITCH_VELOCITY_TEST_STATE_STOPPING)
        {
            finalize_fault(test, PITCH_VELOCITY_TEST_FAILURE_STOP_TIMEOUT);
        }
        else
        {
            latch_fault_with_stop(
                test,
                PITCH_VELOCITY_TEST_FAILURE_COMMAND_TIMEOUT,
                now_ms);
        }
        return;
    }
    if (driver_state != X42S_DRIVER_STATE_COMMAND_VALID)
    {
        return;
    }

    test->report.last_ack_status =
        (uint8_t)X42sDriver_GetCommandStatus(test->driver);
    queue_event(
        test,
        PITCH_VELOCITY_TEST_EVENT_COMMAND_ACK,
        0U,
        test->report.last_command,
        test->report.last_ack_status,
        0);

    if (test->report.state == PITCH_VELOCITY_TEST_STATE_STOPPING)
    {
        test->report.velocity_command_active = false;
        finalize_fault(test, test->report.failure);
        return;
    }

    if (test->report.last_ack_status !=
        (uint8_t)X42S_COMMAND_STATUS_ACCEPTED)
    {
        latch_fault_with_stop(
            test,
            PITCH_VELOCITY_TEST_FAILURE_COMMAND_REJECTED,
            now_ms);
        return;
    }

    if (test->report.state == PITCH_VELOCITY_TEST_STATE_WAIT_ENABLE_ACK)
    {
        test->report.enabled = true;
        test->report.state = PITCH_VELOCITY_TEST_STATE_ENABLED_STOPPED;
        if (test->automatic_start_pending)
        {
            test->automatic_start_pending = false;
            (void)arm_automatic_internal(test, now_ms);
        }
    }
    else if (test->report.state ==
             PITCH_VELOCITY_TEST_STATE_WAIT_DISABLE_ACK)
    {
        test->report.enabled = false;
        test->report.velocity_command_active = false;
        test->report.state = PITCH_VELOCITY_TEST_STATE_DISABLED_READY;
    }
    else if (test->report.state ==
             PITCH_VELOCITY_TEST_STATE_WAIT_VELOCITY_ACK)
    {
        test->report.velocity_command_active = true;
        test->velocity_started_ms = now_ms;
        test->report.state = PITCH_VELOCITY_TEST_STATE_RUNNING_TIMED;
    }
    else if (test->report.state == PITCH_VELOCITY_TEST_STATE_WAIT_STOP_ACK)
    {
        test->report.velocity_command_active = false;
        request_position_after(test, now_ms);
    }
    else if (test->report.state ==
             PITCH_VELOCITY_TEST_STATE_WAIT_AUTOMATIC_VELOCITY_ACK)
    {
        test->report.velocity_command_active = true;
        test->automatic_motion_segment_active = true;
        test->automatic_motion_started_ms = now_ms;
        test->report.automatic_motion_active = true;
        test->report.state =
            PITCH_VELOCITY_TEST_STATE_RUNNING_AUTOMATIC;
    }
    else if (test->report.state ==
             PITCH_VELOCITY_TEST_STATE_WAIT_AUTOMATIC_STOP_ACK)
    {
        test->report.velocity_command_active = false;
        test->automatic_motion_segment_active = false;
        test->report.automatic_motion_active = false;
        test->report.state = PITCH_VELOCITY_TEST_STATE_ENABLED_STOPPED;
    }
}

static void process_position_response(
    PitchAxisVelocityTest *test,
    uint32_t now_ms)
{
    X42sDriverState driver_state = X42sDriver_GetState(test->driver);
    int64_t position;

    if (test->automatic_position_query_pending)
    {
        if (driver_state == X42S_DRIVER_STATE_POSITION_TIMEOUT)
        {
            test->automatic_position_query_pending = false;
            test->report.timeout_count++;
            latch_fault_with_stop(
                test,
                PITCH_VELOCITY_TEST_FAILURE_POSITION_TIMEOUT,
                now_ms);
            return;
        }
        if (driver_state != X42S_DRIVER_STATE_POSITION_VALID)
        {
            return;
        }

        position = raw_position_to_signed(X42sDriver_GetPosition(test->driver));
        test->automatic_position_query_pending = false;
        test->automatic_last_position_update_ms = now_ms;
        test->report.automatic_position_valid = true;
        test->report.automatic_position_raw = position;
        test->report.automatic_position_age_ms = 0U;
        test->report.automatic_position_error_raw =
            test->report.automatic_target_position_raw - position;
        test->automatic_position_target_dirty = false;

        if (test->automatic_zero_capture_pending)
        {
            test->automatic_zero_capture_pending = false;
            test->report.automatic_zero_valid = true;
            test->report.automatic_zero_position_raw = position;
            test->report.automatic_target_position_raw = position;
            test->report.automatic_target_offset_raw = 0;
            test->report.automatic_position_error_raw = 0;
            queue_event(
                test,
                PITCH_VELOCITY_TEST_EVENT_POSITION_ZERO,
                0U,
                PITCH_VELOCITY_TEST_COMMAND_NONE,
                0U,
                position);
        }
        if (test->automatic_stop_after_position_query &&
            automatic_motion_state(test) &&
            (test->report.state !=
             PITCH_VELOCITY_TEST_STATE_WAIT_AUTOMATIC_STOP_ACK))
        {
            request_automatic_stop(test, now_ms);
        }
        return;
    }

    if (driver_state == X42S_DRIVER_STATE_POSITION_TIMEOUT)
    {
        test->report.timeout_count++;
        latch_fault_with_stop(
            test,
            PITCH_VELOCITY_TEST_FAILURE_POSITION_TIMEOUT,
            now_ms);
        return;
    }
    if (driver_state != X42S_DRIVER_STATE_POSITION_VALID)
    {
        return;
    }

    position = raw_position_to_signed(X42sDriver_GetPosition(test->driver));
    if (test->report.state ==
        PITCH_VELOCITY_TEST_STATE_WAIT_POSITION_BEFORE)
    {
        test->report.position_before = position;
        queue_event(
            test,
            PITCH_VELOCITY_TEST_EVENT_POSITION_BEFORE,
            0U,
            test->pending_motion_command,
            0U,
            position);
        request_velocity(test, now_ms);
    }
    else if (test->report.state ==
             PITCH_VELOCITY_TEST_STATE_WAIT_POSITION_AFTER)
    {
        test->report.position_after = position;
        test->report.position_delta =
            test->report.position_after - test->report.position_before;
        queue_event(
            test,
            PITCH_VELOCITY_TEST_EVENT_POSITION_AFTER,
            0U,
            test->pending_motion_command,
            0U,
            test->report.position_after);
        queue_event(
            test,
            PITCH_VELOCITY_TEST_EVENT_POSITION_DELTA,
            0U,
            test->pending_motion_command,
            0U,
            test->report.position_delta);
        test->report.state = PITCH_VELOCITY_TEST_STATE_ENABLED_STOPPED;
    }
}

static void enter_ball_escape_hold(
    PitchAxisVelocityTest *test,
    uint32_t now_ms)
{
    bool motion_active = automatic_motion_state(test);
    bool stop_pending = test->report.state ==
        PITCH_VELOCITY_TEST_STATE_WAIT_AUTOMATIC_STOP_ACK;

    if (test->report.automatic_hold)
    {
        return;
    }

    test->report.automatic_hold = true;
    test->report.ball_escape_count++;
    disarm_automatic(test, PITCH_AUTOMATIC_DISARM_BALL_ESCAPE);
    queue_event(
        test,
        PITCH_VELOCITY_TEST_EVENT_BALL_ESCAPE_HOLD,
        2U,
        PITCH_VELOCITY_TEST_COMMAND_AUTOMATIC_STOP,
        0U,
        (int64_t)test->report.ball_escape_count);
    if (motion_active && !stop_pending)
    {
        request_automatic_stop(test, now_ms);
    }
}

static void handle_key_edges(
    PitchAxisVelocityTest *test,
    uint8_t press_edges,
    uint32_t now_ms)
{
    uint8_t key;

    if (press_edges == 0U)
    {
        return;
    }
    if ((press_edges & (uint8_t)(press_edges - 1U)) != 0U)
    {
        reject(test, PITCH_VELOCITY_TEST_EVENT_REJECT_CONFLICT, 0U);
        return;
    }

    key = (press_edges & 0x01U) ? 1U :
          (press_edges & 0x02U) ? 2U : 3U;
    test->report.last_key = key;
    queue_event(
        test,
        PITCH_VELOCITY_TEST_EVENT_KEY_PRESS,
        key,
        PITCH_VELOCITY_TEST_COMMAND_NONE,
        0U,
        0);

    if (!test->report.communication_ready)
    {
        reject(test, PITCH_VELOCITY_TEST_EVENT_REJECT_LOCKED, key);
        return;
    }
    if (key == 1U)
    {
        /* Automatic startup owns motor enable; KEY1 is deliberately ignored. */
        return;
    }
    if ((test->report.pid_debug_enabled ||
         test->report.automatic_armed ||
         test->report.automatic_motion_active ||
         test->report.automatic_hold) && (key == 2U))
    {
        enter_ball_escape_hold(test, now_ms);
        return;
    }
    if ((test->report.pid_debug_enabled ||
         test->report.automatic_armed ||
         test->report.automatic_hold) && (key == 3U))
    {
        if (test->report.automatic_hold)
        {
            test->report.automatic_hold = false;
            queue_event(
                test,
                PITCH_VELOCITY_TEST_EVENT_RESUME_READY,
                3U,
                PITCH_VELOCITY_TEST_COMMAND_NONE,
                0U,
                0);
        }
        else
        {
            reject(test, PITCH_VELOCITY_TEST_EVENT_REJECT_BUSY, key);
        }
        return;
    }
    if (test->report.automatic_armed)
    {
        reject(test, PITCH_VELOCITY_TEST_EVENT_REJECT_BUSY, key);
        return;
    }
    if ((test->report.state != PITCH_VELOCITY_TEST_STATE_DISABLED_READY) &&
        (test->report.state != PITCH_VELOCITY_TEST_STATE_ENABLED_STOPPED))
    {
        reject(test, PITCH_VELOCITY_TEST_EVENT_REJECT_BUSY, key);
        return;
    }

    if (!test->report.enabled)
    {
        reject(test, PITCH_VELOCITY_TEST_EVENT_REJECT_DISABLED, key);
    }
    else
    {
        request_position_before(
            test,
            (key == 2U) ?
                PITCH_VELOCITY_TEST_COMMAND_RUN_POSITIVE :
                PITCH_VELOCITY_TEST_COMMAND_RUN_NEGATIVE,
            now_ms);
    }
}

bool PitchAxisVelocityTest_Init(
    PitchAxisVelocityTest *test,
    X42sDriver *driver,
    const PitchAxisVelocityTestConfig *config,
    uint32_t now_ms)
{
    uint8_t index;

    if ((test == NULL) || (driver == NULL) || !valid_config(config))
    {
        return false;
    }

    memset(test, 0, sizeof(*test));
    test->driver = driver;
    test->config = *config;
    test->report.state = PITCH_VELOCITY_TEST_STATE_LOCKED_WAIT_SELF_TEST;
    for (index = 0U; index < 4U; ++index)
    {
        test->buttons[index].candidate_since_ms = now_ms;
    }
    test->initialized = true;
    return true;
}

bool PitchAxisVelocityTest_SetAutomaticArmed(
    PitchAxisVelocityTest *test,
    bool armed,
    uint32_t now_ms)
{
    if ((test == NULL) || !test->initialized ||
        !test->report.communication_ready || test->report.fault_latched)
    {
        return false;
    }

    if (!armed)
    {
        if (!test->report.automatic_armed)
        {
            return true;
        }
        disarm_automatic(test, PITCH_AUTOMATIC_DISARM_USER);
        if (automatic_motion_state(test) &&
            (test->report.state !=
             PITCH_VELOCITY_TEST_STATE_WAIT_AUTOMATIC_STOP_ACK))
        {
            request_automatic_stop(test, now_ms);
        }
        return true;
    }

    if (test->report.automatic_armed)
    {
        return true;
    }
    if (test->report.automatic_hold ||
        !test->report.enabled ||
        (test->config.automatic_position_tracking_enabled &&
         !test->report.automatic_zero_valid) ||
        (test->report.state != PITCH_VELOCITY_TEST_STATE_ENABLED_STOPPED))
    {
        return false;
    }

    return arm_automatic_internal(test, now_ms);
}

static bool arm_automatic_internal(
    PitchAxisVelocityTest *test,
    uint32_t now_ms)
{
    if ((test == NULL) || test->report.automatic_armed ||
        test->report.automatic_hold || !test->report.enabled ||
        (test->config.automatic_position_tracking_enabled &&
         !test->report.automatic_zero_valid) ||
        (test->report.state != PITCH_VELOCITY_TEST_STATE_ENABLED_STOPPED))
    {
        return false;
    }

    test->report.automatic_armed = true;
    test->report.automatic_disarm_reason = PITCH_AUTOMATIC_DISARM_NONE;
    test->automatic_decision_pending = false;
    test->automatic_last_decision_ms = now_ms;
    test->automatic_budget_used_ms = 0U;
    test->report.automatic_budget_used_ms = 0U;
    test->automatic_motion_segment_active = false;
    test->report.automatic_motion_active = false;
    if (test->config.automatic_position_tracking_enabled)
    {
        test->report.automatic_target_offset_raw = 0;
        test->report.automatic_target_position_raw =
            test->report.automatic_zero_position_raw;
        test->automatic_position_target_dirty = true;
    }
    clear_automatic_vision_loss(test);
    test->automatic_edge_recovery_available = false;
    queue_event(
        test,
        PITCH_VELOCITY_TEST_EVENT_AUTOMATIC_ARMED,
        0U,
        PITCH_VELOCITY_TEST_COMMAND_NONE,
        0U,
        0);
    return true;
}

bool PitchAxisVelocityTest_ClearAutomaticHold(
    PitchAxisVelocityTest *test)
{
    if ((test == NULL) || !test->initialized || test->report.fault_latched ||
        test->report.automatic_armed || automatic_motion_state(test))
    {
        return false;
    }

    if (test->report.automatic_hold)
    {
        test->report.automatic_hold = false;
        queue_event(
            test,
            PITCH_VELOCITY_TEST_EVENT_RESUME_READY,
            0U,
            PITCH_VELOCITY_TEST_COMMAND_NONE,
            0U,
            0);
    }
    return true;
}

bool PitchAxisVelocityTest_CaptureAutomaticZero(
    PitchAxisVelocityTest *test,
    uint32_t now_ms)
{
    if ((test == NULL) || !test->initialized || test->report.fault_latched ||
        test->report.automatic_armed || automatic_motion_state(test) ||
        (test->report.state != PITCH_VELOCITY_TEST_STATE_ENABLED_STOPPED) ||
        test->automatic_position_query_pending)
    {
        return false;
    }

    test->automatic_zero_capture_pending = true;
    request_automatic_position(test, now_ms);
    if (!test->automatic_position_query_pending)
    {
        test->automatic_zero_capture_pending = false;
        return false;
    }
    return true;
}

bool PitchAxisVelocityTest_SetPidDebugEnabled(
    PitchAxisVelocityTest *test,
    bool enabled,
    uint32_t now_ms)
{
    if ((test == NULL) || !test->initialized || test->report.fault_latched)
    {
        return false;
    }

    if (!enabled && test->report.automatic_armed)
    {
        (void)PitchAxisVelocityTest_SetAutomaticArmed(test, false, now_ms);
    }
    test->report.pid_debug_enabled = enabled;
    if (!enabled)
    {
        test->report.automatic_hold = false;
    }
    return true;
}

bool PitchAxisVelocityTest_GetConfig(
    const PitchAxisVelocityTest *test,
    PitchAxisVelocityTestConfig *config)
{
    if ((test == NULL) || (config == NULL) || !test->initialized)
    {
        return false;
    }

    *config = test->config;
    return true;
}

bool PitchAxisVelocityTest_UpdateConfig(
    PitchAxisVelocityTest *test,
    const PitchAxisVelocityTestConfig *config)
{
    bool manual_fields_changed;
    bool busy;

    if ((test == NULL) || !test->initialized || !valid_config(config))
    {
        return false;
    }

    manual_fields_changed =
        (config->address != test->config.address) ||
        (config->positive_direction != test->config.positive_direction) ||
        (config->negative_direction != test->config.negative_direction) ||
        (config->speed_rpm != test->config.speed_rpm) ||
        (config->run_ms != test->config.run_ms) ||
        (config->synchronize != test->config.synchronize) ||
        (config->debounce_ms != test->config.debounce_ms) ||
        (config->automatic_position_tracking_enabled !=
         test->config.automatic_position_tracking_enabled) ||
        (config->automatic_direction0_increases_raw !=
         test->config.automatic_direction0_increases_raw) ||
        (config->automatic_position_raw_per_mm !=
         test->config.automatic_position_raw_per_mm);
    busy = test->report.automatic_armed || automatic_motion_state(test) ||
        ((test->report.state != PITCH_VELOCITY_TEST_STATE_DISABLED_READY) &&
         (test->report.state != PITCH_VELOCITY_TEST_STATE_ENABLED_STOPPED) &&
         (test->report.state != PITCH_VELOCITY_TEST_STATE_FAULT_LATCHED));
    if (manual_fields_changed && busy)
    {
        return false;
    }

    test->config = *config;
    if (config->automatic_position_tracking_enabled)
    {
        test->automatic_position_target_dirty = true;
    }
    return true;
}

void PitchAxisVelocityTest_SubmitAutomaticDecision(
    PitchAxisVelocityTest *test,
    const PitchAxisAutomaticDecision *decision,
    uint32_t now_ms)
{
    if ((test == NULL) || (decision == NULL) || !test->initialized ||
        !test->report.automatic_armed || test->report.fault_latched)
    {
        return;
    }

    test->automatic_decision = *decision;
    if (test->automatic_decision.motor_direction > 1U)
    {
        test->automatic_decision.source_safe = false;
        test->automatic_decision.unsafe_reason =
            PITCH_AUTOMATIC_DISARM_FAULT;
    }
    if (test->automatic_decision.edge_recovery_candidate &&
        (test->automatic_decision.source_safe ||
         (test->automatic_decision.unsafe_reason ==
          PITCH_AUTOMATIC_DISARM_BALL_ESCAPE)) &&
        (test->automatic_decision.edge_recovery_direction > 1U))
    {
        test->automatic_decision.source_safe = false;
        test->automatic_decision.unsafe_reason =
            PITCH_AUTOMATIC_DISARM_FAULT;
    }
    if (test->automatic_decision.source_safe)
    {
        test->automatic_edge_recovery_available =
            test->automatic_decision.edge_recovery_candidate;
        if (test->automatic_edge_recovery_available)
        {
            test->automatic_edge_recovery_direction =
                test->automatic_decision.edge_recovery_direction;
        }
    }
    else if ((test->automatic_decision.unsafe_reason ==
              PITCH_AUTOMATIC_DISARM_BALL_ESCAPE) &&
             test->automatic_decision.edge_recovery_candidate)
    {
        /* A coordinate just beyond the calibrated pipe limit still carries a
         * reliable side and inward direction. Preserve it for recovery. */
        test->automatic_edge_recovery_available = true;
        test->automatic_edge_recovery_direction =
            test->automatic_decision.edge_recovery_direction;
    }
    test->automatic_last_decision_ms = now_ms;
    test->automatic_decision_pending = true;
}

void PitchAxisVelocityTest_SetCommunicationResult(
    PitchAxisVelocityTest *test,
    bool passed,
    uint32_t now_ms)
{
    (void)now_ms;

    if ((test == NULL) || !test->initialized ||
        test->communication_result_set || test->report.fault_latched)
    {
        return;
    }

    test->communication_result_set = true;
    if (!passed)
    {
        finalize_fault(test, PITCH_VELOCITY_TEST_FAILURE_SELF_TEST);
        return;
    }

    test->protocol_error_baseline = test->driver->protocol_error_count;
    test->uart_error_baseline = test->driver->transport.uart_error_count;
    test->rx_overflow_baseline = test->driver->transport.rx_overflow_count;
    test->report.communication_ready = true;
    test->report.state = PITCH_VELOCITY_TEST_STATE_DISABLED_READY;
    if (test->config.automatic_position_tracking_enabled)
    {
        int64_t position = raw_position_to_signed(
            X42sDriver_GetPosition(test->driver));

        test->report.automatic_zero_valid = true;
        test->report.automatic_position_valid = true;
        test->report.automatic_zero_position_raw = position;
        test->report.automatic_position_raw = position;
        test->report.automatic_target_position_raw = position;
        test->report.automatic_target_offset_raw = 0;
        test->report.automatic_position_error_raw = 0;
        test->automatic_last_position_query_ms = now_ms;
        test->automatic_last_position_update_ms = now_ms;
    }
    queue_event(
        test,
        PITCH_VELOCITY_TEST_EVENT_READY,
        0U,
        PITCH_VELOCITY_TEST_COMMAND_NONE,
        0U,
        0);

    /* Self-test success is the only automatic-start trigger. The enable
     * command remains asynchronous; ARM follows only after its ACK. */
    test->automatic_start_pending = true;
    request_enable(test, true, now_ms);
}

void PitchAxisVelocityTest_Service(
    PitchAxisVelocityTest *test,
    uint32_t now_ms,
    PitchAxisVelocityTestButtons buttons)
{
    uint8_t press_edges;

    if ((test == NULL) || !test->initialized)
    {
        return;
    }

    press_edges = update_buttons(test, now_ms, buttons);
    if ((press_edges & 0x08U) != 0U)
    {
        test->report.last_key = 4U;
        queue_event(
            test,
            PITCH_VELOCITY_TEST_EVENT_KEY_PRESS,
            4U,
            PITCH_VELOCITY_TEST_COMMAND_NONE,
            0U,
            0);
        latch_fault_with_stop(
            test,
            PITCH_VELOCITY_TEST_FAILURE_STOP_BUTTON,
            now_ms);
        return;
    }

    if (test->report.state == PITCH_VELOCITY_TEST_STATE_STOPPING)
    {
        X42sDriver_Service(test->driver, now_ms);
        process_command_response(test, now_ms);
        return;
    }
    if (test->report.fault_latched)
    {
        return;
    }

    if (((press_edges & 0x06U) != 0U) &&
        test->buttons[1].stable_pressed &&
        test->buttons[2].stable_pressed)
    {
        reject(test, PITCH_VELOCITY_TEST_EVENT_REJECT_CONFLICT, 0U);
        press_edges &= (uint8_t)~0x06U;
    }
    handle_key_edges(test, (uint8_t)(press_edges & 0x07U), now_ms);
    if (!test->report.communication_ready || test->report.fault_latched)
    {
        return;
    }

    X42sDriver_Service(test->driver, now_ms);

    if (test->driver->protocol_error_count != test->protocol_error_baseline)
    {
        latch_fault_with_stop(
            test,
            PITCH_VELOCITY_TEST_FAILURE_PROTOCOL,
            now_ms);
        return;
    }
    if (test->driver->transport.uart_error_count != test->uart_error_baseline)
    {
        latch_fault_with_stop(
            test,
            PITCH_VELOCITY_TEST_FAILURE_UART,
            now_ms);
        return;
    }
    if (test->driver->transport.rx_overflow_count != test->rx_overflow_baseline)
    {
        latch_fault_with_stop(
            test,
            PITCH_VELOCITY_TEST_FAILURE_RX_OVERFLOW,
            now_ms);
        return;
    }

    if ((test->report.state ==
         PITCH_VELOCITY_TEST_STATE_WAIT_ENABLE_ACK) ||
        (test->report.state ==
         PITCH_VELOCITY_TEST_STATE_WAIT_DISABLE_ACK) ||
        (test->report.state ==
         PITCH_VELOCITY_TEST_STATE_WAIT_VELOCITY_ACK) ||
        (test->report.state == PITCH_VELOCITY_TEST_STATE_WAIT_STOP_ACK) ||
        (test->report.state ==
         PITCH_VELOCITY_TEST_STATE_WAIT_AUTOMATIC_VELOCITY_ACK) ||
        (test->report.state ==
         PITCH_VELOCITY_TEST_STATE_WAIT_AUTOMATIC_STOP_ACK))
    {
        process_command_response(test, now_ms);
    }
    else if (test->automatic_position_query_pending ||
             (test->report.state ==
               PITCH_VELOCITY_TEST_STATE_WAIT_POSITION_BEFORE) ||
             (test->report.state ==
               PITCH_VELOCITY_TEST_STATE_WAIT_POSITION_AFTER))
    {
        process_position_response(test, now_ms);
    }
    else if ((test->report.state ==
              PITCH_VELOCITY_TEST_STATE_RUNNING_TIMED) &&
             time_elapsed(
                 now_ms,
                 test->velocity_started_ms,
                 test->config.run_ms))
    {
        request_timed_stop(test, now_ms);
    }

    service_automatic_control(test, now_ms);
}

PitchAxisVelocityTestState PitchAxisVelocityTest_GetState(
    const PitchAxisVelocityTest *test)
{
    return (test == NULL) ? PITCH_VELOCITY_TEST_STATE_UNINITIALIZED :
        test->report.state;
}

bool PitchAxisVelocityTest_GetReport(
    const PitchAxisVelocityTest *test,
    PitchAxisVelocityTestReport *report)
{
    if ((test == NULL) || (report == NULL) || !test->initialized)
    {
        return false;
    }

    *report = test->report;
    return true;
}

bool PitchAxisVelocityTest_PeekEvent(
    const PitchAxisVelocityTest *test,
    PitchAxisVelocityTestEvent *event)
{
    if ((test == NULL) || (event == NULL) ||
        (test->event_tail == test->event_head))
    {
        return false;
    }

    *event = test->events[test->event_tail];
    return true;
}

void PitchAxisVelocityTest_DropEvent(PitchAxisVelocityTest *test)
{
    if ((test != NULL) && (test->event_tail != test->event_head))
    {
        test->event_tail = (uint8_t)((test->event_tail + 1U) %
                                     PITCH_AXIS_VELOCITY_TEST_EVENT_QUEUE_SIZE);
    }
}
