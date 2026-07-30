#ifndef PITCH_AXIS_ANGLE_SELF_TEST_H
#define PITCH_AXIS_ANGLE_SELF_TEST_H

#include <stdbool.h>
#include <stdint.h>

#include "as5600.h"

#define PITCH_AXIS_ANGLE_SELF_TEST_DEFAULT_SAMPLES 500U
#define PITCH_AXIS_ANGLE_SELF_TEST_DEFAULT_PERIOD_MS 20U
#define PITCH_AXIS_ANGLE_SELF_TEST_DEFAULT_START_DELAY_MS 500U

typedef enum
{
    PITCH_AXIS_ANGLE_SELF_TEST_OK = 0,
    PITCH_AXIS_ANGLE_SELF_TEST_INVALID_ARGUMENT,
    PITCH_AXIS_ANGLE_SELF_TEST_NOT_READY
} PitchAxisAngleSelfTestResult;

typedef enum
{
    PITCH_AXIS_ANGLE_SELF_TEST_STATE_UNINITIALIZED = 0,
    PITCH_AXIS_ANGLE_SELF_TEST_STATE_IDLE,
    PITCH_AXIS_ANGLE_SELF_TEST_STATE_WAITING_TO_START,
    PITCH_AXIS_ANGLE_SELF_TEST_STATE_PROBING,
    PITCH_AXIS_ANGLE_SELF_TEST_STATE_WAITING_FOR_NEXT_SAMPLE,
    PITCH_AXIS_ANGLE_SELF_TEST_STATE_PASS,
    PITCH_AXIS_ANGLE_SELF_TEST_STATE_FAILED
} PitchAxisAngleSelfTestState;

typedef enum
{
    PITCH_AXIS_ANGLE_SELF_TEST_FAILURE_NONE = 0,
    PITCH_AXIS_ANGLE_SELF_TEST_FAILURE_I2C_NACK,
    PITCH_AXIS_ANGLE_SELF_TEST_FAILURE_I2C_TIMEOUT,
    PITCH_AXIS_ANGLE_SELF_TEST_FAILURE_I2C_TRANSPORT,
    PITCH_AXIS_ANGLE_SELF_TEST_FAILURE_MAGNET_MISSING,
    PITCH_AXIS_ANGLE_SELF_TEST_FAILURE_MAGNET_WEAK,
    PITCH_AXIS_ANGLE_SELF_TEST_FAILURE_MAGNET_STRONG
} PitchAxisAngleSelfTestFailure;

typedef struct
{
    uint32_t target_samples;
    uint32_t completed_samples;
    uint32_t valid_sample_count;
    uint32_t i2c_error_count;
    uint32_t magnet_error_count;
    uint32_t started_ms;
    uint32_t completed_ms;
    uint16_t first_raw_count;
    uint16_t latest_raw_count;
    uint8_t first_status;
    uint8_t latest_status;
    int32_t minimum_continuous_count;
    int32_t maximum_continuous_count;
    int32_t latest_continuous_count;
    uint32_t maximum_step_count;
    bool sample_valid;
} PitchAxisAngleSelfTestReport;

typedef struct
{
    As5600 sensor;
    I2C_HandleTypeDef *i2c;
    PitchAxisAngleSelfTestState state;
    PitchAxisAngleSelfTestFailure failure;
    As5600Result last_sensor_result;
    PitchAxisAngleSelfTestReport report;
    uint32_t sample_period_ms;
    uint32_t start_delay_ms;
    uint32_t next_action_ms;
    bool initialized;
} PitchAxisAngleSelfTest;

PitchAxisAngleSelfTestResult PitchAxisAngleSelfTest_Init(
    PitchAxisAngleSelfTest *self_test,
    I2C_HandleTypeDef *i2c,
    uint32_t target_samples,
    uint32_t sample_period_ms,
    uint32_t start_delay_ms);

PitchAxisAngleSelfTestResult PitchAxisAngleSelfTest_Start(
    PitchAxisAngleSelfTest *self_test,
    uint32_t now_ms);

void PitchAxisAngleSelfTest_Service(
    PitchAxisAngleSelfTest *self_test,
    uint32_t now_ms);

PitchAxisAngleSelfTestState PitchAxisAngleSelfTest_GetState(
    const PitchAxisAngleSelfTest *self_test);

PitchAxisAngleSelfTestFailure PitchAxisAngleSelfTest_GetFailure(
    const PitchAxisAngleSelfTest *self_test);

bool PitchAxisAngleSelfTest_GetReport(
    const PitchAxisAngleSelfTest *self_test,
    PitchAxisAngleSelfTestReport *report);

#endif
