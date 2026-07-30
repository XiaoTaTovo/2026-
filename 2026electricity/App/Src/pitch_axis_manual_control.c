#include "pitch_axis_manual_control.h"

#include <stddef.h>
#include <string.h>

static bool time_elapsed(uint32_t now_ms, uint32_t start_ms, uint32_t delay_ms)
{
    return (uint32_t)(now_ms - start_ms) >= delay_ms;
}

static int64_t raw_position_to_signed(X42sRawPosition position)
{
    int64_t magnitude = (int64_t)position.magnitude;
    return position.negative ? -magnitude : magnitude;
}

static void queue_event(
    PitchAxisManualControl *control,
    PitchAxisManualEventType type,
    uint8_t key,
    PitchAxisManualCommand command,
    uint8_t ack_status,
    int64_t value)
{
    uint8_t next = (uint8_t)((control->event_head + 1U) %
                             PITCH_AXIS_MANUAL_EVENT_QUEUE_SIZE);

    if (next == control->event_tail)
    {
        control->report.event_drop_count++;
        return;
    }

    control->events[control->event_head].type = type;
    control->events[control->event_head].key = key;
    control->events[control->event_head].command = command;
    control->events[control->event_head].ack_status = ack_status;
    control->events[control->event_head].value = value;
    control->event_head = next;
}

static uint8_t update_buttons(
    PitchAxisManualControl *control,
    uint32_t now_ms,
    PitchAxisManualButtons buttons)
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
        PitchAxisManualDebouncer *debouncer = &control->buttons[index];

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
                     control->config.debounce_ms))
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
    PitchAxisManualControl *control,
    PitchAxisManualFailure failure)
{
    if (!control->report.fault_latched)
    {
        control->report.error_count++;
    }
    control->report.fault_latched = true;
    control->report.failure = failure;
    control->report.state = PITCH_MANUAL_STATE_FAULT_LATCHED;
    queue_event(
        control,
        PITCH_MANUAL_EVENT_FAULT_LATCHED,
        0U,
        control->report.last_command,
        control->report.last_ack_status,
        (int64_t)failure);
}

static void latch_fault_with_stop(
    PitchAxisManualControl *control,
    PitchAxisManualFailure failure,
    uint32_t now_ms)
{
    X42sDriverResult result;

    if (control->report.fault_latched)
    {
        return;
    }

    control->report.fault_latched = true;
    control->report.failure = failure;
    control->report.error_count++;
    control->pending_command = PITCH_MANUAL_COMMAND_STOP;
    control->report.last_command = PITCH_MANUAL_COMMAND_STOP;

    result = X42sDriver_RequestStop(
        control->driver,
        control->config.address,
        control->config.synchronize,
        now_ms);
    if (result == X42S_DRIVER_OK)
    {
        control->report.command_count++;
        control->report.state = PITCH_MANUAL_STATE_STOPPING;
        queue_event(
            control,
            PITCH_MANUAL_EVENT_STOP_SENT,
            0U,
            PITCH_MANUAL_COMMAND_STOP,
            0U,
            0);
        queue_event(
            control,
            PITCH_MANUAL_EVENT_FAULT_LATCHED,
            0U,
            PITCH_MANUAL_COMMAND_STOP,
            0U,
            (int64_t)failure);
    }
    else
    {
        control->report.state = PITCH_MANUAL_STATE_FAULT_LATCHED;
        queue_event(
            control,
            PITCH_MANUAL_EVENT_FAULT_LATCHED,
            0U,
            PITCH_MANUAL_COMMAND_STOP,
            0U,
            (int64_t)failure);
    }
}

static void reject(
    PitchAxisManualControl *control,
    PitchAxisManualEventType type,
    uint8_t key)
{
    control->report.reject_count++;
    queue_event(
        control,
        type,
        key,
        PITCH_MANUAL_COMMAND_NONE,
        0U,
        0);
}

static void request_enable(
    PitchAxisManualControl *control,
    bool enable,
    uint32_t now_ms)
{
    X42sDriverResult result = X42sDriver_RequestEnable(
        control->driver,
        control->config.address,
        enable,
        control->config.synchronize,
        now_ms);

    if (result != X42S_DRIVER_OK)
    {
        latch_fault_with_stop(
            control,
            PITCH_MANUAL_FAILURE_REQUEST,
            now_ms);
        return;
    }

    control->pending_command = enable ?
        PITCH_MANUAL_COMMAND_ENABLE : PITCH_MANUAL_COMMAND_DISABLE;
    control->report.last_command = control->pending_command;
    control->report.command_count++;
    control->report.state = enable ?
        PITCH_MANUAL_STATE_WAIT_ENABLE_ACK :
        PITCH_MANUAL_STATE_WAIT_DISABLE_ACK;
    queue_event(
        control,
        enable ? PITCH_MANUAL_EVENT_ENABLE_SENT :
                 PITCH_MANUAL_EVENT_DISABLE_SENT,
        0U,
        control->pending_command,
        0U,
        0);
}

static void request_position_before(
    PitchAxisManualControl *control,
    PitchAxisManualCommand command,
    uint32_t now_ms)
{
    X42sDriverResult result = X42sDriver_RequestReadPosition(
        control->driver,
        control->config.address,
        now_ms);

    if (result != X42S_DRIVER_OK)
    {
        latch_fault_with_stop(
            control,
            PITCH_MANUAL_FAILURE_REQUEST,
            now_ms);
        return;
    }

    control->pending_command = command;
    control->report.state = PITCH_MANUAL_STATE_WAIT_POSITION_BEFORE;
    queue_event(
        control,
        PITCH_MANUAL_EVENT_POSITION_BEFORE_REQUESTED,
        0U,
        command,
        0U,
        0);
}

static void send_pending_move(
    PitchAxisManualControl *control,
    uint32_t now_ms)
{
    uint8_t direction =
        (control->pending_command == PITCH_MANUAL_COMMAND_MOVE_POSITIVE) ?
        control->config.positive_direction : control->config.negative_direction;
    X42sDriverResult result = X42sDriver_RequestEmmPosition(
        control->driver,
        control->config.address,
        direction,
        control->config.speed_rpm,
        control->config.acceleration,
        control->config.step_pulses,
        control->config.motion_mode,
        control->config.synchronize,
        now_ms);

    if (result != X42S_DRIVER_OK)
    {
        latch_fault_with_stop(
            control,
            PITCH_MANUAL_FAILURE_REQUEST,
            now_ms);
        return;
    }

    control->report.last_command = control->pending_command;
    control->report.command_count++;
    control->report.state = PITCH_MANUAL_STATE_WAIT_MOVE_ACK;
    queue_event(
        control,
        PITCH_MANUAL_EVENT_MOVE_SENT,
        0U,
        control->pending_command,
        0U,
        (int64_t)control->config.step_pulses);
}

static void request_position_after(
    PitchAxisManualControl *control,
    uint32_t now_ms)
{
    X42sDriverResult result = X42sDriver_RequestReadPosition(
        control->driver,
        control->config.address,
        now_ms);

    if (result != X42S_DRIVER_OK)
    {
        latch_fault_with_stop(
            control,
            PITCH_MANUAL_FAILURE_REQUEST,
            now_ms);
        return;
    }

    control->report.state = PITCH_MANUAL_STATE_WAIT_POSITION_AFTER;
    queue_event(
        control,
        PITCH_MANUAL_EVENT_POSITION_AFTER_REQUESTED,
        0U,
        control->pending_command,
        0U,
        0);
}

static void handle_key_edges(
    PitchAxisManualControl *control,
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
        reject(control, PITCH_MANUAL_EVENT_REJECT_CONFLICT, 0U);
        return;
    }

    key = (press_edges & 0x01U) ? 1U :
          (press_edges & 0x02U) ? 2U : 3U;
    control->report.last_key = key;
    queue_event(
        control,
        PITCH_MANUAL_EVENT_KEY_PRESS,
        key,
        PITCH_MANUAL_COMMAND_NONE,
        0U,
        0);

    if (!control->report.communication_ready)
    {
        reject(control, PITCH_MANUAL_EVENT_REJECT_LOCKED, key);
        return;
    }

    if ((control->report.state != PITCH_MANUAL_STATE_DISABLED_READY) &&
        (control->report.state != PITCH_MANUAL_STATE_ENABLED_IDLE))
    {
        reject(control, PITCH_MANUAL_EVENT_REJECT_BUSY, key);
        return;
    }

    if (key == 1U)
    {
        request_enable(control, !control->report.enabled, now_ms);
    }
    else if (!control->report.enabled)
    {
        reject(control, PITCH_MANUAL_EVENT_REJECT_DISABLED, key);
    }
    else
    {
        request_position_before(
            control,
            (key == 2U) ? PITCH_MANUAL_COMMAND_MOVE_POSITIVE :
                          PITCH_MANUAL_COMMAND_MOVE_NEGATIVE,
            now_ms);
    }
}

static void process_command_response(
    PitchAxisManualControl *control,
    uint32_t now_ms)
{
    X42sDriverState driver_state = X42sDriver_GetState(control->driver);

    if (driver_state == X42S_DRIVER_STATE_COMMAND_TIMEOUT)
    {
        control->report.timeout_count++;
        if (control->report.state == PITCH_MANUAL_STATE_STOPPING)
        {
            control->report.state = PITCH_MANUAL_STATE_FAULT_LATCHED;
        }
        else
        {
            latch_fault_with_stop(
                control,
                PITCH_MANUAL_FAILURE_COMMAND_TIMEOUT,
                now_ms);
        }
        return;
    }

    if (driver_state != X42S_DRIVER_STATE_COMMAND_VALID)
    {
        return;
    }

    control->report.last_ack_status =
        (uint8_t)X42sDriver_GetCommandStatus(control->driver);
    queue_event(
        control,
        PITCH_MANUAL_EVENT_COMMAND_ACK,
        0U,
        control->pending_command,
        control->report.last_ack_status,
        0);

    if (control->report.state == PITCH_MANUAL_STATE_STOPPING)
    {
        control->report.state = PITCH_MANUAL_STATE_FAULT_LATCHED;
        return;
    }

    if (control->report.last_ack_status !=
        (uint8_t)X42S_COMMAND_STATUS_ACCEPTED)
    {
        latch_fault_with_stop(
            control,
            PITCH_MANUAL_FAILURE_COMMAND_REJECTED,
            now_ms);
        return;
    }

    if (control->report.state == PITCH_MANUAL_STATE_WAIT_ENABLE_ACK)
    {
        control->report.enabled = true;
        control->report.state = PITCH_MANUAL_STATE_ENABLED_IDLE;
    }
    else if (control->report.state == PITCH_MANUAL_STATE_WAIT_DISABLE_ACK)
    {
        control->report.enabled = false;
        control->report.state = PITCH_MANUAL_STATE_DISABLED_READY;
    }
    else if (control->report.state == PITCH_MANUAL_STATE_WAIT_MOVE_ACK)
    {
        control->settle_started_ms = now_ms;
        control->report.state = PITCH_MANUAL_STATE_WAIT_SETTLE;
    }
}

static void process_position_response(
    PitchAxisManualControl *control,
    uint32_t now_ms)
{
    X42sDriverState driver_state = X42sDriver_GetState(control->driver);
    int64_t position;

    if (driver_state == X42S_DRIVER_STATE_POSITION_TIMEOUT)
    {
        control->report.timeout_count++;
        latch_fault_with_stop(
            control,
            PITCH_MANUAL_FAILURE_POSITION_TIMEOUT,
            now_ms);
        return;
    }
    if (driver_state != X42S_DRIVER_STATE_POSITION_VALID)
    {
        return;
    }

    position = raw_position_to_signed(X42sDriver_GetPosition(control->driver));
    if (control->report.state == PITCH_MANUAL_STATE_WAIT_POSITION_BEFORE)
    {
        control->report.position_before = position;
        queue_event(
            control,
            PITCH_MANUAL_EVENT_POSITION_BEFORE,
            0U,
            control->pending_command,
            0U,
            position);
        send_pending_move(control, now_ms);
    }
    else if (control->report.state == PITCH_MANUAL_STATE_WAIT_POSITION_AFTER)
    {
        control->report.position_after = position;
        control->report.position_delta =
            control->report.position_after - control->report.position_before;
        queue_event(
            control,
            PITCH_MANUAL_EVENT_POSITION_AFTER,
            0U,
            control->pending_command,
            0U,
            control->report.position_after);
        queue_event(
            control,
            PITCH_MANUAL_EVENT_POSITION_DELTA,
            0U,
            control->pending_command,
            0U,
            control->report.position_delta);
        control->report.state = PITCH_MANUAL_STATE_ENABLED_IDLE;
    }
}

bool PitchAxisManualControl_Init(
    PitchAxisManualControl *control,
    X42sDriver *driver,
    const PitchAxisManualConfig *config,
    uint32_t now_ms)
{
    uint8_t index;

    if ((control == NULL) || (driver == NULL) || (config == NULL) ||
        (config->address == 0U) ||
        (config->positive_direction > 1U) ||
        (config->negative_direction > 1U) ||
        (config->positive_direction == config->negative_direction) ||
        (config->speed_rpm > X42S_EMM_MAX_SPEED_RPM) ||
        (config->step_pulses == 0U) || (config->motion_mode > 2U) ||
        (config->debounce_ms == 0U))
    {
        return false;
    }

    memset(control, 0, sizeof(*control));
    control->driver = driver;
    control->config = *config;
    control->report.state = PITCH_MANUAL_STATE_LOCKED_WAIT_SELF_TEST;
    for (index = 0U; index < 4U; ++index)
    {
        control->buttons[index].candidate_since_ms = now_ms;
    }
    control->initialized = true;
    return true;
}

void PitchAxisManualControl_SetCommunicationResult(
    PitchAxisManualControl *control,
    bool passed,
    uint32_t now_ms)
{
    (void)now_ms;

    if ((control == NULL) || !control->initialized ||
        control->communication_result_set || control->report.fault_latched)
    {
        return;
    }

    control->communication_result_set = true;
    if (!passed)
    {
        finalize_fault(control, PITCH_MANUAL_FAILURE_SELF_TEST);
        return;
    }

    control->protocol_error_baseline = control->driver->protocol_error_count;
    control->uart_error_baseline =
        control->driver->transport.uart_error_count;
    control->rx_overflow_baseline =
        control->driver->transport.rx_overflow_count;
    control->report.communication_ready = true;
    control->report.state = PITCH_MANUAL_STATE_DISABLED_READY;
    queue_event(
        control,
        PITCH_MANUAL_EVENT_READY,
        0U,
        PITCH_MANUAL_COMMAND_NONE,
        0U,
        0);
}

void PitchAxisManualControl_Service(
    PitchAxisManualControl *control,
    uint32_t now_ms,
    PitchAxisManualButtons buttons)
{
    uint8_t press_edges;

    if ((control == NULL) || !control->initialized)
    {
        return;
    }

    press_edges = update_buttons(control, now_ms, buttons);
    if ((press_edges & 0x08U) != 0U)
    {
        control->report.last_key = 4U;
        queue_event(
            control,
            PITCH_MANUAL_EVENT_KEY_PRESS,
            4U,
            PITCH_MANUAL_COMMAND_NONE,
            0U,
            0);
        latch_fault_with_stop(
            control,
            PITCH_MANUAL_FAILURE_STOP_BUTTON,
            now_ms);
    }

    if (control->report.state == PITCH_MANUAL_STATE_STOPPING)
    {
        X42sDriver_Service(control->driver, now_ms);
        process_command_response(control, now_ms);
        return;
    }
    if (control->report.fault_latched)
    {
        return;
    }

    if (((press_edges & 0x06U) != 0U) &&
        control->buttons[1].stable_pressed &&
        control->buttons[2].stable_pressed)
    {
        reject(control, PITCH_MANUAL_EVENT_REJECT_CONFLICT, 0U);
        press_edges &= (uint8_t)~0x06U;
    }

    handle_key_edges(control, (uint8_t)(press_edges & 0x07U), now_ms);
    if (!control->report.communication_ready ||
        control->report.fault_latched)
    {
        return;
    }

    X42sDriver_Service(control->driver, now_ms);

    if (control->driver->protocol_error_count !=
        control->protocol_error_baseline)
    {
        latch_fault_with_stop(
            control,
            PITCH_MANUAL_FAILURE_PROTOCOL,
            now_ms);
        return;
    }
    if (control->driver->transport.uart_error_count !=
        control->uart_error_baseline)
    {
        latch_fault_with_stop(
            control,
            PITCH_MANUAL_FAILURE_UART,
            now_ms);
        return;
    }
    if (control->driver->transport.rx_overflow_count !=
        control->rx_overflow_baseline)
    {
        latch_fault_with_stop(
            control,
            PITCH_MANUAL_FAILURE_RX_OVERFLOW,
            now_ms);
        return;
    }

    if ((control->report.state == PITCH_MANUAL_STATE_WAIT_ENABLE_ACK) ||
        (control->report.state == PITCH_MANUAL_STATE_WAIT_DISABLE_ACK) ||
        (control->report.state == PITCH_MANUAL_STATE_WAIT_MOVE_ACK))
    {
        process_command_response(control, now_ms);
    }
    else if ((control->report.state ==
              PITCH_MANUAL_STATE_WAIT_POSITION_BEFORE) ||
             (control->report.state ==
              PITCH_MANUAL_STATE_WAIT_POSITION_AFTER))
    {
        process_position_response(control, now_ms);
    }
    else if ((control->report.state == PITCH_MANUAL_STATE_WAIT_SETTLE) &&
             time_elapsed(
                 now_ms,
                 control->settle_started_ms,
                 control->config.settle_ms))
    {
        request_position_after(control, now_ms);
    }
}

PitchAxisManualState PitchAxisManualControl_GetState(
    const PitchAxisManualControl *control)
{
    return (control == NULL) ? PITCH_MANUAL_STATE_UNINITIALIZED :
        control->report.state;
}

bool PitchAxisManualControl_GetReport(
    const PitchAxisManualControl *control,
    PitchAxisManualReport *report)
{
    if ((control == NULL) || (report == NULL) || !control->initialized)
    {
        return false;
    }
    *report = control->report;
    return true;
}

bool PitchAxisManualControl_PeekEvent(
    const PitchAxisManualControl *control,
    PitchAxisManualEvent *event)
{
    if ((control == NULL) || (event == NULL) ||
        (control->event_tail == control->event_head))
    {
        return false;
    }
    *event = control->events[control->event_tail];
    return true;
}

void PitchAxisManualControl_DropEvent(PitchAxisManualControl *control)
{
    if ((control != NULL) && (control->event_tail != control->event_head))
    {
        control->event_tail = (uint8_t)((control->event_tail + 1U) %
                                        PITCH_AXIS_MANUAL_EVENT_QUEUE_SIZE);
    }
}
