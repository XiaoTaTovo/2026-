#ifndef PITCH_AXIS_SELF_TEST_H
#define PITCH_AXIS_SELF_TEST_H

#include <stdbool.h>
#include <stdint.h>

#include "x42s_driver.h"

#define PITCH_AXIS_SELF_TEST_DEFAULT_CYCLES 1000U
#define PITCH_AXIS_SELF_TEST_DEFAULT_PERIOD_MS 10U
#define PITCH_AXIS_SELF_TEST_DEFAULT_START_DELAY_MS 500U

typedef enum
{
    PITCH_AXIS_SELF_TEST_OK = 0,
    PITCH_AXIS_SELF_TEST_INVALID_ARGUMENT,
    PITCH_AXIS_SELF_TEST_NOT_READY
} PitchAxisSelfTestResult;

typedef enum
{
    PITCH_AXIS_SELF_TEST_STATE_UNINITIALIZED = 0,
    PITCH_AXIS_SELF_TEST_STATE_IDLE,
    PITCH_AXIS_SELF_TEST_STATE_WAITING_TO_START,
    PITCH_AXIS_SELF_TEST_STATE_REQUESTING_STATUS,
    PITCH_AXIS_SELF_TEST_STATE_WAITING_FOR_STATUS,
    PITCH_AXIS_SELF_TEST_STATE_REQUESTING_POSITION,
    PITCH_AXIS_SELF_TEST_STATE_WAITING_FOR_POSITION,
    PITCH_AXIS_SELF_TEST_STATE_WAITING_FOR_NEXT_CYCLE,
    PITCH_AXIS_SELF_TEST_STATE_COMM_PASS,
    PITCH_AXIS_SELF_TEST_STATE_FAILED
} PitchAxisSelfTestState;

typedef enum
{
    PITCH_AXIS_SELF_TEST_FAILURE_NONE = 0,
    PITCH_AXIS_SELF_TEST_FAILURE_PROTOCOL,
    PITCH_AXIS_SELF_TEST_FAILURE_STATUS_REQUEST,
    PITCH_AXIS_SELF_TEST_FAILURE_STATUS_TIMEOUT,
    PITCH_AXIS_SELF_TEST_FAILURE_POSITION_REQUEST,
    PITCH_AXIS_SELF_TEST_FAILURE_POSITION_TIMEOUT
} PitchAxisSelfTestFailure;

typedef struct
{
    uint32_t target_cycles;
    uint32_t completed_cycles;
    uint32_t address_echo_valid_count;
    uint32_t status_valid_count;
    uint32_t position_valid_count;
    uint32_t request_error_count;
    uint32_t protocol_error_count;
    uint32_t status_change_count;
    uint32_t started_ms;
    uint32_t completed_ms;
    uint32_t last_position_received_ms;
    uint8_t first_status;
    uint8_t last_status;
    int64_t first_position_raw;
    int64_t minimum_position_raw;
    int64_t maximum_position_raw;
    int64_t latest_position_raw;
    uint64_t maximum_position_step_raw;
    bool status_valid;
    bool position_valid;
} PitchAxisSelfTestReport;

typedef struct
{
    X42sDriver *driver;
    PitchAxisSelfTestState state;
    PitchAxisSelfTestFailure failure;
    X42sDriverResult last_driver_result;
    PitchAxisSelfTestReport report;
    uint32_t cycle_period_ms;
    uint32_t start_delay_ms;
    uint32_t next_action_ms;
    uint32_t protocol_error_baseline;
    uint8_t expected_address;
    bool initialized;
} PitchAxisSelfTest;

PitchAxisSelfTestResult PitchAxisSelfTest_Init(
    PitchAxisSelfTest *self_test,
    X42sDriver *driver,
    uint8_t expected_address,
    uint32_t target_cycles,
    uint32_t cycle_period_ms,
    uint32_t start_delay_ms);

PitchAxisSelfTestResult PitchAxisSelfTest_Start(
    PitchAxisSelfTest *self_test,
    uint32_t now_ms);

void PitchAxisSelfTest_Service(
    PitchAxisSelfTest *self_test,
    uint32_t now_ms);

PitchAxisSelfTestState PitchAxisSelfTest_GetState(
    const PitchAxisSelfTest *self_test);

PitchAxisSelfTestFailure PitchAxisSelfTest_GetFailure(
    const PitchAxisSelfTest *self_test);

bool PitchAxisSelfTest_GetReport(
    const PitchAxisSelfTest *self_test,
    PitchAxisSelfTestReport *report);

#endif
