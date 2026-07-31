#ifndef PITCH_AXIS_VELOCITY_TEST_H
#define PITCH_AXIS_VELOCITY_TEST_H

#include <stdbool.h>
#include <stdint.h>

#include "x42s_driver.h"

#define PITCH_AXIS_VELOCITY_TEST_EVENT_QUEUE_SIZE 32U
/* Position feedback and a relative soft limit bound automatic travel. */
#define PITCH_AXIS_VELOCITY_TEST_HARD_AUTO_MAX_RPM 300U
#define PITCH_AXIS_VELOCITY_TEST_HARD_EDGE_RECOVERY_MAX_RPM 50U
#define PITCH_AXIS_VELOCITY_TEST_HARD_MAX_VISION_LOSS_GRACE_MS 5000U
#define PITCH_AXIS_VELOCITY_TEST_HARD_MAX_EDGE_RECOVERY_MS 3000U

typedef enum
{
    PITCH_VELOCITY_TEST_STATE_UNINITIALIZED = 0,
    PITCH_VELOCITY_TEST_STATE_LOCKED_WAIT_SELF_TEST,
    PITCH_VELOCITY_TEST_STATE_DISABLED_READY,
    PITCH_VELOCITY_TEST_STATE_WAIT_ENABLE_ACK,
    PITCH_VELOCITY_TEST_STATE_ENABLED_STOPPED,
    PITCH_VELOCITY_TEST_STATE_WAIT_DISABLE_ACK,
    PITCH_VELOCITY_TEST_STATE_WAIT_POSITION_BEFORE,
    PITCH_VELOCITY_TEST_STATE_WAIT_VELOCITY_ACK,
    PITCH_VELOCITY_TEST_STATE_RUNNING_TIMED,
    PITCH_VELOCITY_TEST_STATE_WAIT_STOP_ACK,
    PITCH_VELOCITY_TEST_STATE_WAIT_POSITION_AFTER,
    PITCH_VELOCITY_TEST_STATE_WAIT_AUTOMATIC_VELOCITY_ACK,
    PITCH_VELOCITY_TEST_STATE_RUNNING_AUTOMATIC,
    PITCH_VELOCITY_TEST_STATE_WAIT_AUTOMATIC_STOP_ACK,
    PITCH_VELOCITY_TEST_STATE_STOPPING,
    PITCH_VELOCITY_TEST_STATE_FAULT_LATCHED
} PitchAxisVelocityTestState;

typedef enum
{
    PITCH_VELOCITY_TEST_COMMAND_NONE = 0,
    PITCH_VELOCITY_TEST_COMMAND_ENABLE,
    PITCH_VELOCITY_TEST_COMMAND_DISABLE,
    PITCH_VELOCITY_TEST_COMMAND_RUN_POSITIVE,
    PITCH_VELOCITY_TEST_COMMAND_RUN_NEGATIVE,
    PITCH_VELOCITY_TEST_COMMAND_STOP,
    PITCH_VELOCITY_TEST_COMMAND_AUTOMATIC_VELOCITY,
    PITCH_VELOCITY_TEST_COMMAND_AUTOMATIC_STOP
} PitchAxisVelocityTestCommand;

typedef enum
{
    PITCH_VELOCITY_TEST_FAILURE_NONE = 0,
    PITCH_VELOCITY_TEST_FAILURE_SELF_TEST,
    PITCH_VELOCITY_TEST_FAILURE_STOP_BUTTON,
    PITCH_VELOCITY_TEST_FAILURE_REQUEST,
    PITCH_VELOCITY_TEST_FAILURE_POSITION_TIMEOUT,
    PITCH_VELOCITY_TEST_FAILURE_COMMAND_TIMEOUT,
    PITCH_VELOCITY_TEST_FAILURE_STOP_TIMEOUT,
    PITCH_VELOCITY_TEST_FAILURE_COMMAND_REJECTED,
    PITCH_VELOCITY_TEST_FAILURE_PROTOCOL,
    PITCH_VELOCITY_TEST_FAILURE_UART,
    PITCH_VELOCITY_TEST_FAILURE_RX_OVERFLOW,
    PITCH_VELOCITY_TEST_FAILURE_POSITION_LIMIT
} PitchAxisVelocityTestFailure;

typedef enum
{
    PITCH_AUTOMATIC_DISARM_NONE = 0,
    PITCH_AUTOMATIC_DISARM_USER,
    PITCH_AUTOMATIC_DISARM_BALL_ESCAPE,
    PITCH_AUTOMATIC_DISARM_VISION_INVALID,
    PITCH_AUTOMATIC_DISARM_VISION_LOW_CONFIDENCE,
    PITCH_AUTOMATIC_DISARM_VISION_STALE,
    PITCH_AUTOMATIC_DISARM_DECISION_TIMEOUT,
    PITCH_AUTOMATIC_DISARM_EDGE_RECOVERY_TIMEOUT,
    PITCH_AUTOMATIC_DISARM_BUDGET,
    PITCH_AUTOMATIC_DISARM_FAULT
} PitchAxisAutomaticDisarmReason;

typedef enum
{
    PITCH_VELOCITY_TEST_EVENT_READY = 0,
    PITCH_VELOCITY_TEST_EVENT_KEY_PRESS,
    PITCH_VELOCITY_TEST_EVENT_REJECT_LOCKED,
    PITCH_VELOCITY_TEST_EVENT_REJECT_DISABLED,
    PITCH_VELOCITY_TEST_EVENT_REJECT_BUSY,
    PITCH_VELOCITY_TEST_EVENT_REJECT_CONFLICT,
    PITCH_VELOCITY_TEST_EVENT_ENABLE_SENT,
    PITCH_VELOCITY_TEST_EVENT_DISABLE_SENT,
    PITCH_VELOCITY_TEST_EVENT_POSITION_BEFORE_REQUESTED,
    PITCH_VELOCITY_TEST_EVENT_POSITION_BEFORE,
    PITCH_VELOCITY_TEST_EVENT_VELOCITY_SENT,
    PITCH_VELOCITY_TEST_EVENT_COMMAND_ACK,
    PITCH_VELOCITY_TEST_EVENT_AUTO_STOP_SENT,
    PITCH_VELOCITY_TEST_EVENT_POSITION_AFTER_REQUESTED,
    PITCH_VELOCITY_TEST_EVENT_POSITION_AFTER,
    PITCH_VELOCITY_TEST_EVENT_POSITION_DELTA,
    PITCH_VELOCITY_TEST_EVENT_STOP_SENT,
    PITCH_VELOCITY_TEST_EVENT_FAULT_LATCHED,
    PITCH_VELOCITY_TEST_EVENT_AUTOMATIC_ARMED,
    PITCH_VELOCITY_TEST_EVENT_AUTOMATIC_DISARMED,
    PITCH_VELOCITY_TEST_EVENT_AUTOMATIC_VELOCITY_SENT,
    PITCH_VELOCITY_TEST_EVENT_AUTOMATIC_STOP_SENT,
    PITCH_VELOCITY_TEST_EVENT_BALL_ESCAPE_HOLD,
    PITCH_VELOCITY_TEST_EVENT_RESUME_READY,
    PITCH_VELOCITY_TEST_EVENT_POSITION_ZERO
} PitchAxisVelocityTestEventType;

typedef struct
{
    bool key1_pressed;
    bool key2_pressed;
    bool key3_pressed;
    bool key4_pressed;
} PitchAxisVelocityTestButtons;

typedef struct
{
    uint8_t address;
    uint8_t positive_direction;
    uint8_t negative_direction;
    uint16_t speed_rpm;
    uint8_t acceleration;
    uint32_t run_ms;
    bool synchronize;
    uint32_t debounce_ms;
    uint16_t automatic_max_speed_rpm;
    bool automatic_position_tracking_enabled;
    bool automatic_direction0_increases_raw;
    uint32_t automatic_position_raw_per_mm;
    uint16_t automatic_tilt_scale_um_per_outer_rpm;
    uint16_t automatic_tilt_limit_um;
    uint16_t automatic_position_deadband_um;
    uint16_t automatic_position_slow_zone_um;
    uint16_t automatic_position_min_speed_rpm;
    uint32_t automatic_position_poll_period_ms;
    uint32_t automatic_decision_timeout_ms;
    uint32_t automatic_vision_loss_grace_ms;
    bool automatic_edge_recovery_enabled;
    uint16_t automatic_edge_recovery_speed_rpm;
    /* Maximum duration of one recovery pulse before an acknowledged retry. */
    uint32_t automatic_edge_recovery_max_ms;
    /* Zero keeps automatic control armed without a cumulative time limit. */
    uint32_t automatic_motion_budget_ms;
} PitchAxisVelocityTestConfig;

typedef struct
{
    bool source_safe;
    bool motion_requested;
    uint8_t motor_direction;
    uint16_t speed_rpm;
    bool edge_recovery_candidate;
    uint8_t edge_recovery_direction;
    int16_t outer_control_0_01rpm;
    uint8_t sequence;
    PitchAxisAutomaticDisarmReason unsafe_reason;
} PitchAxisAutomaticDecision;

typedef struct
{
    PitchAxisVelocityTestEventType type;
    PitchAxisVelocityTestCommand command;
    uint8_t key;
    uint8_t ack_status;
    int64_t value;
} PitchAxisVelocityTestEvent;

typedef struct
{
    PitchAxisVelocityTestState state;
    PitchAxisVelocityTestFailure failure;
    PitchAxisVelocityTestCommand last_command;
    bool communication_ready;
    bool enabled;
    bool velocity_command_active;
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
    bool automatic_armed;
    bool automatic_motion_active;
    uint8_t automatic_direction;
    uint16_t automatic_speed_rpm;
    uint32_t automatic_budget_used_ms;
    PitchAxisAutomaticDisarmReason automatic_disarm_reason;
    bool automatic_vision_loss_active;
    uint32_t automatic_vision_loss_age_ms;
    PitchAxisAutomaticDisarmReason automatic_vision_loss_reason;
    bool automatic_edge_recovery_active;
    uint8_t automatic_edge_recovery_direction;
    uint16_t automatic_edge_recovery_speed_rpm;
    uint32_t automatic_edge_recovery_age_ms;
    uint32_t automatic_edge_recovery_count;
    bool pid_debug_enabled;
    bool automatic_hold;
    uint32_t ball_escape_count;
    bool automatic_zero_valid;
    bool automatic_position_valid;
    int64_t automatic_zero_position_raw;
    int64_t automatic_position_raw;
    int64_t automatic_target_position_raw;
    int64_t automatic_target_offset_raw;
    int64_t automatic_position_error_raw;
    uint32_t automatic_position_age_ms;
    uint32_t automatic_position_query_count;
    uint32_t automatic_position_limit_count;
} PitchAxisVelocityTestReport;

typedef struct
{
    bool stable_pressed;
    bool candidate_pressed;
    uint32_t candidate_since_ms;
} PitchAxisVelocityTestDebouncer;

typedef struct
{
    X42sDriver *driver;
    PitchAxisVelocityTestConfig config;
    PitchAxisVelocityTestReport report;
    PitchAxisVelocityTestDebouncer buttons[4];
    PitchAxisVelocityTestEvent events[PITCH_AXIS_VELOCITY_TEST_EVENT_QUEUE_SIZE];
    uint8_t event_head;
    uint8_t event_tail;
    PitchAxisVelocityTestCommand pending_motion_command;
    uint32_t velocity_started_ms;
    uint32_t protocol_error_baseline;
    uint32_t uart_error_baseline;
    uint32_t rx_overflow_baseline;
    PitchAxisAutomaticDecision automatic_decision;
    uint32_t automatic_last_decision_ms;
    uint32_t automatic_motion_started_ms;
    uint32_t automatic_budget_used_ms;
    uint32_t automatic_vision_loss_started_ms;
    uint32_t automatic_edge_recovery_started_ms;
    uint32_t automatic_last_position_query_ms;
    uint32_t automatic_last_position_update_ms;
    bool automatic_decision_pending;
    bool automatic_motion_segment_active;
    bool automatic_edge_recovery_available;
    uint8_t automatic_edge_recovery_direction;
    bool automatic_position_query_pending;
    bool automatic_position_target_dirty;
    bool automatic_zero_capture_pending;
    bool automatic_stop_after_position_query;
    bool automatic_start_pending;
    bool communication_result_set;
    bool initialized;
} PitchAxisVelocityTest;

bool PitchAxisVelocityTest_Init(
    PitchAxisVelocityTest *test,
    X42sDriver *driver,
    const PitchAxisVelocityTestConfig *config,
    uint32_t now_ms);

void PitchAxisVelocityTest_SetCommunicationResult(
    PitchAxisVelocityTest *test,
    bool passed,
    uint32_t now_ms);

void PitchAxisVelocityTest_Service(
    PitchAxisVelocityTest *test,
    uint32_t now_ms,
    PitchAxisVelocityTestButtons buttons);

/* Automatic control is armed after a successful communication self-test.
 * KEY1 remains in the input structure for wiring compatibility but has no
 * enable/disable side effect. */
bool PitchAxisVelocityTest_SetAutomaticArmed(
    PitchAxisVelocityTest *test,
    bool armed,
    uint32_t now_ms);

bool PitchAxisVelocityTest_ClearAutomaticHold(
    PitchAxisVelocityTest *test);

bool PitchAxisVelocityTest_CaptureAutomaticZero(
    PitchAxisVelocityTest *test,
    uint32_t now_ms);

bool PitchAxisVelocityTest_SetPidDebugEnabled(
    PitchAxisVelocityTest *test,
    bool enabled,
    uint32_t now_ms);

bool PitchAxisVelocityTest_GetConfig(
    const PitchAxisVelocityTest *test,
    PitchAxisVelocityTestConfig *config);

bool PitchAxisVelocityTest_UpdateConfig(
    PitchAxisVelocityTest *test,
    const PitchAxisVelocityTestConfig *config);

/* Submit exactly one decision produced by the vision controller. */
void PitchAxisVelocityTest_SubmitAutomaticDecision(
    PitchAxisVelocityTest *test,
    const PitchAxisAutomaticDecision *decision,
    uint32_t now_ms);

PitchAxisVelocityTestState PitchAxisVelocityTest_GetState(
    const PitchAxisVelocityTest *test);

bool PitchAxisVelocityTest_GetReport(
    const PitchAxisVelocityTest *test,
    PitchAxisVelocityTestReport *report);

bool PitchAxisVelocityTest_PeekEvent(
    const PitchAxisVelocityTest *test,
    PitchAxisVelocityTestEvent *event);

void PitchAxisVelocityTest_DropEvent(PitchAxisVelocityTest *test);

#endif
