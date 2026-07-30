#include "pitch_axis_self_test.h"

#include <string.h>

static bool time_reached(uint32_t now_ms, uint32_t deadline_ms)
{
    return (int32_t)(now_ms - deadline_ms) >= 0;
}

static int64_t raw_position_to_signed(X42sRawPosition position)
{
    int64_t magnitude = (int64_t)position.magnitude;

    return position.negative ? -magnitude : magnitude;
}

static uint64_t absolute_step(int64_t current, int64_t previous)
{
    int64_t difference = current - previous;

    return (difference < 0) ? (uint64_t)(-difference) : (uint64_t)difference;
}

static void update_protocol_error_count(PitchAxisSelfTest *self_test)
{
    self_test->report.protocol_error_count =
        self_test->driver->protocol_error_count -
        self_test->protocol_error_baseline;
}

static void fail_self_test(
    PitchAxisSelfTest *self_test,
    PitchAxisSelfTestFailure failure,
    uint32_t now_ms)
{
    update_protocol_error_count(self_test);
    self_test->failure = failure;
    self_test->report.completed_ms = now_ms;
    self_test->state = PITCH_AXIS_SELF_TEST_STATE_FAILED;
}

static void record_status(PitchAxisSelfTest *self_test, uint8_t status)
{
    if (!self_test->report.status_valid)
    {
        self_test->report.first_status = status;
        self_test->report.status_valid = true;
    }
    else if (status != self_test->report.last_status)
    {
        self_test->report.status_change_count++;
    }

    self_test->report.last_status = status;
    self_test->report.address_echo_valid_count++;
    self_test->report.status_valid_count++;
}

static void record_position(
    PitchAxisSelfTest *self_test,
    X42sRawPosition position,
    uint32_t now_ms)
{
    int64_t signed_position = raw_position_to_signed(position);

    if (!self_test->report.position_valid)
    {
        self_test->report.first_position_raw = signed_position;
        self_test->report.minimum_position_raw = signed_position;
        self_test->report.maximum_position_raw = signed_position;
        self_test->report.position_valid = true;
    }
    else
    {
        uint64_t step = absolute_step(
            signed_position,
            self_test->report.latest_position_raw);

        if (signed_position < self_test->report.minimum_position_raw)
        {
            self_test->report.minimum_position_raw = signed_position;
        }
        if (signed_position > self_test->report.maximum_position_raw)
        {
            self_test->report.maximum_position_raw = signed_position;
        }
        if (step > self_test->report.maximum_position_step_raw)
        {
            self_test->report.maximum_position_step_raw = step;
        }
    }

    self_test->report.latest_position_raw = signed_position;
    self_test->report.last_position_received_ms = now_ms;
    self_test->report.address_echo_valid_count++;
    self_test->report.position_valid_count++;
}

PitchAxisSelfTestResult PitchAxisSelfTest_Init(
    PitchAxisSelfTest *self_test,
    X42sDriver *driver,
    uint8_t expected_address,
    uint32_t target_cycles,
    uint32_t cycle_period_ms,
    uint32_t start_delay_ms)
{
    if ((self_test == NULL) || (driver == NULL) ||
        (expected_address == 0U) || (target_cycles == 0U) ||
        (cycle_period_ms == 0U))
    {
        return PITCH_AXIS_SELF_TEST_INVALID_ARGUMENT;
    }

    memset(self_test, 0, sizeof(*self_test));
    self_test->driver = driver;
    self_test->expected_address = expected_address;
    self_test->report.target_cycles = target_cycles;
    self_test->cycle_period_ms = cycle_period_ms;
    self_test->start_delay_ms = start_delay_ms;
    self_test->state = PITCH_AXIS_SELF_TEST_STATE_IDLE;
    self_test->failure = PITCH_AXIS_SELF_TEST_FAILURE_NONE;
    self_test->last_driver_result = X42S_DRIVER_OK;
    self_test->initialized = true;
    return PITCH_AXIS_SELF_TEST_OK;
}

PitchAxisSelfTestResult PitchAxisSelfTest_Start(
    PitchAxisSelfTest *self_test,
    uint32_t now_ms)
{
    uint32_t target_cycles;

    if ((self_test == NULL) || !self_test->initialized)
    {
        return PITCH_AXIS_SELF_TEST_INVALID_ARGUMENT;
    }
    if (!self_test->driver->started)
    {
        return PITCH_AXIS_SELF_TEST_NOT_READY;
    }

    target_cycles = self_test->report.target_cycles;
    memset(&self_test->report, 0, sizeof(self_test->report));
    self_test->report.target_cycles = target_cycles;
    self_test->report.started_ms = now_ms;
    self_test->protocol_error_baseline =
        self_test->driver->protocol_error_count;
    self_test->next_action_ms = now_ms + self_test->start_delay_ms;
    self_test->failure = PITCH_AXIS_SELF_TEST_FAILURE_NONE;
    self_test->last_driver_result = X42S_DRIVER_OK;
    self_test->state = PITCH_AXIS_SELF_TEST_STATE_WAITING_TO_START;
    return PITCH_AXIS_SELF_TEST_OK;
}

void PitchAxisSelfTest_Service(
    PitchAxisSelfTest *self_test,
    uint32_t now_ms)
{
    X42sDriverState driver_state;

    if ((self_test == NULL) || !self_test->initialized ||
        (self_test->state == PITCH_AXIS_SELF_TEST_STATE_IDLE) ||
        (self_test->state == PITCH_AXIS_SELF_TEST_STATE_COMM_PASS) ||
        (self_test->state == PITCH_AXIS_SELF_TEST_STATE_FAILED))
    {
        return;
    }

    X42sDriver_Service(self_test->driver, now_ms);
    update_protocol_error_count(self_test);
    if (self_test->report.protocol_error_count != 0U)
    {
        fail_self_test(
            self_test,
            PITCH_AXIS_SELF_TEST_FAILURE_PROTOCOL,
            now_ms);
        return;
    }

    driver_state = X42sDriver_GetState(self_test->driver);

    switch (self_test->state)
    {
        case PITCH_AXIS_SELF_TEST_STATE_WAITING_TO_START:
        case PITCH_AXIS_SELF_TEST_STATE_WAITING_FOR_NEXT_CYCLE:
            if (time_reached(now_ms, self_test->next_action_ms))
            {
                self_test->state =
                    PITCH_AXIS_SELF_TEST_STATE_REQUESTING_STATUS;
            }
            break;

        case PITCH_AXIS_SELF_TEST_STATE_REQUESTING_STATUS:
            self_test->last_driver_result =
                X42sDriver_RequestReadStatus(
                    self_test->driver,
                    self_test->expected_address,
                    now_ms);
            if (self_test->last_driver_result == X42S_DRIVER_OK)
            {
                self_test->state =
                    PITCH_AXIS_SELF_TEST_STATE_WAITING_FOR_STATUS;
            }
            else
            {
                self_test->report.request_error_count++;
                fail_self_test(
                    self_test,
                    PITCH_AXIS_SELF_TEST_FAILURE_STATUS_REQUEST,
                    now_ms);
            }
            break;

        case PITCH_AXIS_SELF_TEST_STATE_WAITING_FOR_STATUS:
            if (driver_state == X42S_DRIVER_STATE_STATUS_VALID)
            {
                record_status(
                    self_test,
                    X42sDriver_GetStatus(self_test->driver));
                self_test->state =
                    PITCH_AXIS_SELF_TEST_STATE_REQUESTING_POSITION;
            }
            else if (driver_state == X42S_DRIVER_STATE_STATUS_TIMEOUT)
            {
                fail_self_test(
                    self_test,
                    PITCH_AXIS_SELF_TEST_FAILURE_STATUS_TIMEOUT,
                    now_ms);
            }
            break;

        case PITCH_AXIS_SELF_TEST_STATE_REQUESTING_POSITION:
            self_test->last_driver_result =
                X42sDriver_RequestReadPosition(
                    self_test->driver,
                    self_test->expected_address,
                    now_ms);
            if (self_test->last_driver_result == X42S_DRIVER_OK)
            {
                self_test->state =
                    PITCH_AXIS_SELF_TEST_STATE_WAITING_FOR_POSITION;
            }
            else
            {
                self_test->report.request_error_count++;
                fail_self_test(
                    self_test,
                    PITCH_AXIS_SELF_TEST_FAILURE_POSITION_REQUEST,
                    now_ms);
            }
            break;

        case PITCH_AXIS_SELF_TEST_STATE_WAITING_FOR_POSITION:
            if (driver_state == X42S_DRIVER_STATE_POSITION_VALID)
            {
                record_position(
                    self_test,
                    X42sDriver_GetPosition(self_test->driver),
                    now_ms);
                self_test->report.completed_cycles++;

                if (self_test->report.completed_cycles >=
                    self_test->report.target_cycles)
                {
                    self_test->report.completed_ms = now_ms;
                    self_test->state =
                        PITCH_AXIS_SELF_TEST_STATE_COMM_PASS;
                }
                else
                {
                    self_test->next_action_ms =
                        now_ms + self_test->cycle_period_ms;
                    self_test->state =
                        PITCH_AXIS_SELF_TEST_STATE_WAITING_FOR_NEXT_CYCLE;
                }
            }
            else if (driver_state == X42S_DRIVER_STATE_POSITION_TIMEOUT)
            {
                fail_self_test(
                    self_test,
                    PITCH_AXIS_SELF_TEST_FAILURE_POSITION_TIMEOUT,
                    now_ms);
            }
            break;

        default:
            break;
    }
}

PitchAxisSelfTestState PitchAxisSelfTest_GetState(
    const PitchAxisSelfTest *self_test)
{
    return (self_test == NULL) ?
        PITCH_AXIS_SELF_TEST_STATE_UNINITIALIZED : self_test->state;
}

PitchAxisSelfTestFailure PitchAxisSelfTest_GetFailure(
    const PitchAxisSelfTest *self_test)
{
    return (self_test == NULL) ?
        PITCH_AXIS_SELF_TEST_FAILURE_NONE : self_test->failure;
}

bool PitchAxisSelfTest_GetReport(
    const PitchAxisSelfTest *self_test,
    PitchAxisSelfTestReport *report)
{
    if ((self_test == NULL) || (report == NULL) ||
        !self_test->initialized)
    {
        return false;
    }

    *report = self_test->report;
    return true;
}
