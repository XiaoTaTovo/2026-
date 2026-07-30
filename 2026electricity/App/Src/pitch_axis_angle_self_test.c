#include "pitch_axis_angle_self_test.h"

#include <string.h>

static bool time_reached(uint32_t now_ms, uint32_t deadline_ms)
{
    return (int32_t)(now_ms - deadline_ms) >= 0;
}

static uint32_t absolute_step(int32_t current, int32_t previous)
{
    int64_t difference = (int64_t)current - (int64_t)previous;

    return (difference < 0) ? (uint32_t)(-difference) : (uint32_t)difference;
}

static PitchAxisAngleSelfTestFailure failure_from_sensor(
    const PitchAxisAngleSelfTest *self_test,
    As5600Result result)
{
    if ((result == AS5600_TRANSPORT_ERROR) &&
        (self_test->sensor.last_bus_result == BSP_I2C_NACK))
    {
        return PITCH_AXIS_ANGLE_SELF_TEST_FAILURE_I2C_NACK;
    }
    if ((result == AS5600_TRANSPORT_ERROR) &&
        (self_test->sensor.last_bus_result == BSP_I2C_TIMEOUT))
    {
        return PITCH_AXIS_ANGLE_SELF_TEST_FAILURE_I2C_TIMEOUT;
    }

    switch (result)
    {
        case AS5600_MAGNET_NOT_DETECTED:
            return PITCH_AXIS_ANGLE_SELF_TEST_FAILURE_MAGNET_MISSING;
        case AS5600_MAGNET_TOO_WEAK:
            return PITCH_AXIS_ANGLE_SELF_TEST_FAILURE_MAGNET_WEAK;
        case AS5600_MAGNET_TOO_STRONG:
            return PITCH_AXIS_ANGLE_SELF_TEST_FAILURE_MAGNET_STRONG;
        default:
            return PITCH_AXIS_ANGLE_SELF_TEST_FAILURE_I2C_TRANSPORT;
    }
}

static void fail_self_test(
    PitchAxisAngleSelfTest *self_test,
    PitchAxisAngleSelfTestFailure failure,
    uint32_t now_ms)
{
    if ((failure == PITCH_AXIS_ANGLE_SELF_TEST_FAILURE_I2C_NACK) ||
        (failure == PITCH_AXIS_ANGLE_SELF_TEST_FAILURE_I2C_TIMEOUT) ||
        (failure == PITCH_AXIS_ANGLE_SELF_TEST_FAILURE_I2C_TRANSPORT))
    {
        self_test->report.i2c_error_count++;
    }
    else
    {
        self_test->report.magnet_error_count++;
    }

    self_test->failure = failure;
    self_test->report.completed_ms = now_ms;
    self_test->state = PITCH_AXIS_ANGLE_SELF_TEST_STATE_FAILED;
}

static void record_sample(
    PitchAxisAngleSelfTest *self_test,
    const As5600Sample *sample)
{
    uint32_t step;

    if (!self_test->report.sample_valid)
    {
        self_test->report.first_raw_count = sample->raw_count;
        self_test->report.first_status = sample->status;
        self_test->report.minimum_continuous_count = sample->continuous_count;
        self_test->report.maximum_continuous_count = sample->continuous_count;
        self_test->report.sample_valid = true;
    }
    else
    {
        step = absolute_step(
            sample->continuous_count,
            self_test->report.latest_continuous_count);
        if (sample->continuous_count <
            self_test->report.minimum_continuous_count)
        {
            self_test->report.minimum_continuous_count =
                sample->continuous_count;
        }
        if (sample->continuous_count >
            self_test->report.maximum_continuous_count)
        {
            self_test->report.maximum_continuous_count =
                sample->continuous_count;
        }
        if (step > self_test->report.maximum_step_count)
        {
            self_test->report.maximum_step_count = step;
        }
    }

    self_test->report.latest_raw_count = sample->raw_count;
    self_test->report.latest_status = sample->status;
    self_test->report.latest_continuous_count = sample->continuous_count;
    self_test->report.valid_sample_count++;
    self_test->report.completed_samples++;
}

PitchAxisAngleSelfTestResult PitchAxisAngleSelfTest_Init(
    PitchAxisAngleSelfTest *self_test,
    I2C_HandleTypeDef *i2c,
    uint32_t target_samples,
    uint32_t sample_period_ms,
    uint32_t start_delay_ms)
{
    As5600Result result;

    if ((self_test == NULL) || (i2c == NULL) ||
        (target_samples == 0U) || (sample_period_ms == 0U))
    {
        return PITCH_AXIS_ANGLE_SELF_TEST_INVALID_ARGUMENT;
    }

    memset(self_test, 0, sizeof(*self_test));
    self_test->i2c = i2c;
    self_test->sample_period_ms = sample_period_ms;
    self_test->start_delay_ms = start_delay_ms;
    self_test->report.target_samples = target_samples;
    result = As5600_Init(&self_test->sensor, i2c);
    self_test->last_sensor_result = result;
    if (result != AS5600_OK)
    {
        return PITCH_AXIS_ANGLE_SELF_TEST_NOT_READY;
    }

    self_test->state = PITCH_AXIS_ANGLE_SELF_TEST_STATE_IDLE;
    self_test->failure = PITCH_AXIS_ANGLE_SELF_TEST_FAILURE_NONE;
    self_test->initialized = true;
    return PITCH_AXIS_ANGLE_SELF_TEST_OK;
}

PitchAxisAngleSelfTestResult PitchAxisAngleSelfTest_Start(
    PitchAxisAngleSelfTest *self_test,
    uint32_t now_ms)
{
    As5600Result result;
    uint32_t target_samples;

    if ((self_test == NULL) || !self_test->initialized)
    {
        return PITCH_AXIS_ANGLE_SELF_TEST_INVALID_ARGUMENT;
    }

    result = As5600_Init(&self_test->sensor, self_test->i2c);
    self_test->last_sensor_result = result;
    if (result != AS5600_OK)
    {
        return PITCH_AXIS_ANGLE_SELF_TEST_NOT_READY;
    }

    target_samples = self_test->report.target_samples;
    memset(&self_test->report, 0, sizeof(self_test->report));
    self_test->report.target_samples = target_samples;
    self_test->report.started_ms = now_ms;
    self_test->next_action_ms = now_ms + self_test->start_delay_ms;
    self_test->failure = PITCH_AXIS_ANGLE_SELF_TEST_FAILURE_NONE;
    self_test->state = PITCH_AXIS_ANGLE_SELF_TEST_STATE_WAITING_TO_START;
    return PITCH_AXIS_ANGLE_SELF_TEST_OK;
}

void PitchAxisAngleSelfTest_Service(
    PitchAxisAngleSelfTest *self_test,
    uint32_t now_ms)
{
    As5600Sample sample;

    if ((self_test == NULL) || !self_test->initialized ||
        (self_test->state == PITCH_AXIS_ANGLE_SELF_TEST_STATE_IDLE) ||
        (self_test->state == PITCH_AXIS_ANGLE_SELF_TEST_STATE_PASS) ||
        (self_test->state == PITCH_AXIS_ANGLE_SELF_TEST_STATE_FAILED))
    {
        return;
    }

    switch (self_test->state)
    {
        case PITCH_AXIS_ANGLE_SELF_TEST_STATE_WAITING_TO_START:
            if (time_reached(now_ms, self_test->next_action_ms))
            {
                self_test->state = PITCH_AXIS_ANGLE_SELF_TEST_STATE_PROBING;
            }
            break;

        case PITCH_AXIS_ANGLE_SELF_TEST_STATE_PROBING:
            self_test->last_sensor_result = As5600_Probe(&self_test->sensor);
            if (self_test->last_sensor_result != AS5600_OK)
            {
                fail_self_test(
                    self_test,
                    failure_from_sensor(
                        self_test,
                        self_test->last_sensor_result),
                    now_ms);
            }
            else
            {
                self_test->next_action_ms = now_ms;
                self_test->state =
                    PITCH_AXIS_ANGLE_SELF_TEST_STATE_WAITING_FOR_NEXT_SAMPLE;
            }
            break;

        case PITCH_AXIS_ANGLE_SELF_TEST_STATE_WAITING_FOR_NEXT_SAMPLE:
            if (!time_reached(now_ms, self_test->next_action_ms))
            {
                break;
            }

            self_test->last_sensor_result = As5600_ReadSample(
                &self_test->sensor,
                now_ms);
            if (self_test->last_sensor_result != AS5600_OK)
            {
                fail_self_test(
                    self_test,
                    failure_from_sensor(
                        self_test,
                        self_test->last_sensor_result),
                    now_ms);
                break;
            }
            if (!As5600_GetLatestSample(&self_test->sensor, &sample) ||
                !sample.valid)
            {
                fail_self_test(
                    self_test,
                    PITCH_AXIS_ANGLE_SELF_TEST_FAILURE_I2C_TRANSPORT,
                    now_ms);
                break;
            }

            record_sample(self_test, &sample);
            if (self_test->report.completed_samples >=
                self_test->report.target_samples)
            {
                self_test->report.completed_ms = now_ms;
                self_test->state = PITCH_AXIS_ANGLE_SELF_TEST_STATE_PASS;
            }
            else
            {
                self_test->next_action_ms = now_ms +
                    self_test->sample_period_ms;
            }
            break;

        default:
            break;
    }
}

PitchAxisAngleSelfTestState PitchAxisAngleSelfTest_GetState(
    const PitchAxisAngleSelfTest *self_test)
{
    return (self_test == NULL) ?
        PITCH_AXIS_ANGLE_SELF_TEST_STATE_UNINITIALIZED : self_test->state;
}

PitchAxisAngleSelfTestFailure PitchAxisAngleSelfTest_GetFailure(
    const PitchAxisAngleSelfTest *self_test)
{
    return (self_test == NULL) ?
        PITCH_AXIS_ANGLE_SELF_TEST_FAILURE_NONE : self_test->failure;
}

bool PitchAxisAngleSelfTest_GetReport(
    const PitchAxisAngleSelfTest *self_test,
    PitchAxisAngleSelfTestReport *report)
{
    if ((self_test == NULL) || (report == NULL) ||
        !self_test->initialized)
    {
        return false;
    }

    *report = self_test->report;
    return true;
}
