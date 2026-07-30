#ifndef PITCH_AXIS_MANUAL_CONTROL_H
#define PITCH_AXIS_MANUAL_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "x42s_driver.h"

#define PITCH_AXIS_MANUAL_EVENT_QUEUE_SIZE 32U

typedef enum
{
    PITCH_MANUAL_STATE_UNINITIALIZED = 0,
    PITCH_MANUAL_STATE_LOCKED_WAIT_SELF_TEST,
    PITCH_MANUAL_STATE_DISABLED_READY,
    PITCH_MANUAL_STATE_WAIT_ENABLE_ACK,
    PITCH_MANUAL_STATE_ENABLED_IDLE,
    PITCH_MANUAL_STATE_WAIT_DISABLE_ACK,
    PITCH_MANUAL_STATE_WAIT_POSITION_BEFORE,
    PITCH_MANUAL_STATE_WAIT_MOVE_ACK,
    PITCH_MANUAL_STATE_WAIT_SETTLE,
    PITCH_MANUAL_STATE_WAIT_POSITION_AFTER,
    PITCH_MANUAL_STATE_STOPPING,
    PITCH_MANUAL_STATE_FAULT_LATCHED
} PitchAxisManualState;

typedef enum
{
    PITCH_MANUAL_COMMAND_NONE = 0,
    PITCH_MANUAL_COMMAND_ENABLE,
    PITCH_MANUAL_COMMAND_DISABLE,
    PITCH_MANUAL_COMMAND_MOVE_POSITIVE,
    PITCH_MANUAL_COMMAND_MOVE_NEGATIVE,
    PITCH_MANUAL_COMMAND_STOP
} PitchAxisManualCommand;

typedef enum
{
    PITCH_MANUAL_FAILURE_NONE = 0,
    PITCH_MANUAL_FAILURE_SELF_TEST,
    PITCH_MANUAL_FAILURE_STOP_BUTTON,
    PITCH_MANUAL_FAILURE_REQUEST,
    PITCH_MANUAL_FAILURE_POSITION_TIMEOUT,
    PITCH_MANUAL_FAILURE_COMMAND_TIMEOUT,
    PITCH_MANUAL_FAILURE_COMMAND_REJECTED,
    PITCH_MANUAL_FAILURE_PROTOCOL,
    PITCH_MANUAL_FAILURE_UART,
    PITCH_MANUAL_FAILURE_RX_OVERFLOW
} PitchAxisManualFailure;

typedef enum
{
    PITCH_MANUAL_EVENT_READY = 0,
    PITCH_MANUAL_EVENT_KEY_PRESS,
    PITCH_MANUAL_EVENT_REJECT_LOCKED,
    PITCH_MANUAL_EVENT_REJECT_DISABLED,
    PITCH_MANUAL_EVENT_REJECT_BUSY,
    PITCH_MANUAL_EVENT_REJECT_CONFLICT,
    PITCH_MANUAL_EVENT_ENABLE_SENT,
    PITCH_MANUAL_EVENT_DISABLE_SENT,
    PITCH_MANUAL_EVENT_POSITION_BEFORE_REQUESTED,
    PITCH_MANUAL_EVENT_MOVE_SENT,
    PITCH_MANUAL_EVENT_POSITION_AFTER_REQUESTED,
    PITCH_MANUAL_EVENT_COMMAND_ACK,
    PITCH_MANUAL_EVENT_POSITION_BEFORE,
    PITCH_MANUAL_EVENT_POSITION_AFTER,
    PITCH_MANUAL_EVENT_POSITION_DELTA,
    PITCH_MANUAL_EVENT_STOP_SENT,
    PITCH_MANUAL_EVENT_FAULT_LATCHED
} PitchAxisManualEventType;

typedef struct
{
    bool key1_pressed;
    bool key2_pressed;
    bool key3_pressed;
    bool key4_pressed;
} PitchAxisManualButtons;

typedef struct
{
    uint8_t address;
    uint8_t positive_direction;
    uint8_t negative_direction;
    uint16_t speed_rpm;
    uint8_t acceleration;
    uint32_t step_pulses;
    uint8_t motion_mode;
    bool synchronize;
    uint32_t debounce_ms;
    uint32_t settle_ms;
} PitchAxisManualConfig;

typedef struct
{
    PitchAxisManualEventType type;
    PitchAxisManualCommand command;
    uint8_t key;
    uint8_t ack_status;
    int64_t value;
} PitchAxisManualEvent;

typedef struct
{
    PitchAxisManualState state;
    PitchAxisManualFailure failure;
    PitchAxisManualCommand last_command;
    bool communication_ready;
    bool enabled;
    bool fault_latched;
    uint8_t last_key;
    uint8_t last_ack_status;
    int64_t position_before;
    int64_t position_after;
    int64_t position_delta;
    uint32_t command_count;
    uint32_t reject_count;
    uint32_t timeout_count;
    uint32_t error_count;
    uint32_t event_drop_count;
} PitchAxisManualReport;

typedef struct
{
    bool stable_pressed;
    bool candidate_pressed;
    uint32_t candidate_since_ms;
} PitchAxisManualDebouncer;

typedef struct
{
    X42sDriver *driver;
    PitchAxisManualConfig config;
    PitchAxisManualReport report;
    PitchAxisManualDebouncer buttons[4];
    PitchAxisManualEvent events[PITCH_AXIS_MANUAL_EVENT_QUEUE_SIZE];
    uint8_t event_head;
    uint8_t event_tail;
    PitchAxisManualCommand pending_command;
    uint32_t settle_started_ms;
    uint32_t protocol_error_baseline;
    uint32_t uart_error_baseline;
    uint32_t rx_overflow_baseline;
    bool communication_result_set;
    bool initialized;
} PitchAxisManualControl;

bool PitchAxisManualControl_Init(
    PitchAxisManualControl *control,
    X42sDriver *driver,
    const PitchAxisManualConfig *config,
    uint32_t now_ms);

void PitchAxisManualControl_SetCommunicationResult(
    PitchAxisManualControl *control,
    bool passed,
    uint32_t now_ms);

void PitchAxisManualControl_Service(
    PitchAxisManualControl *control,
    uint32_t now_ms,
    PitchAxisManualButtons buttons);

PitchAxisManualState PitchAxisManualControl_GetState(
    const PitchAxisManualControl *control);

bool PitchAxisManualControl_GetReport(
    const PitchAxisManualControl *control,
    PitchAxisManualReport *report);

bool PitchAxisManualControl_PeekEvent(
    const PitchAxisManualControl *control,
    PitchAxisManualEvent *event);

void PitchAxisManualControl_DropEvent(PitchAxisManualControl *control);

#endif
