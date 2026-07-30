#include <assert.h>
#include <stdint.h>
#include <string.h>

#include "pitch_axis_angle_self_test.h"

typedef struct
{
    As5600Result result;
    uint16_t raw_count;
    int32_t continuous_count;
    uint8_t status;
} FakeSample;

static I2C_HandleTypeDef g_i2c;
static As5600Result g_init_result;
static As5600Result g_probe_result;
static BspI2cResult g_probe_bus_result;
static const FakeSample *g_samples;
static uint32_t g_sample_count;
static uint32_t g_sample_index;

static void set_samples(const FakeSample *samples, uint32_t sample_count)
{
    g_samples = samples;
    g_sample_count = sample_count;
    g_sample_index = 0U;
}

As5600Result As5600_Init(As5600 *encoder, I2C_HandleTypeDef *i2c)
{
    assert(encoder != NULL);
    assert(i2c == &g_i2c);
    memset(encoder, 0, sizeof(*encoder));
    encoder->initialized = (g_init_result == AS5600_OK);
    return g_init_result;
}

As5600Result As5600_Probe(As5600 *encoder)
{
    assert(encoder != NULL);
    encoder->last_bus_result = g_probe_bus_result;
    return g_probe_result;
}

As5600Result As5600_ReadSample(As5600 *encoder, uint32_t timestamp_ms)
{
    const FakeSample *sample;

    assert(encoder != NULL);
    assert(g_sample_index < g_sample_count);
    sample = &g_samples[g_sample_index++];
    encoder->last_bus_result = BSP_I2C_OK;
    encoder->latest.valid = (sample->result == AS5600_OK);
    encoder->latest.raw_count = sample->raw_count;
    encoder->latest.continuous_count = sample->continuous_count;
    encoder->latest.status = sample->status;
    encoder->latest.timestamp_ms = timestamp_ms;
    encoder->sample_count++;
    return sample->result;
}

bool As5600_GetLatestSample(const As5600 *encoder, As5600Sample *sample)
{
    if ((encoder == NULL) || (sample == NULL) ||
        (encoder->sample_count == 0U))
    {
        return false;
    }

    *sample = encoder->latest;
    return true;
}

static PitchAxisAngleSelfTestState run_until_terminal(
    PitchAxisAngleSelfTest *self_test,
    uint32_t start_ms,
    uint32_t iterations)
{
    uint32_t index;
    uint32_t now_ms = start_ms;

    for (index = 0U; index < iterations; ++index)
    {
        PitchAxisAngleSelfTest_Service(self_test, now_ms++);
        if ((PitchAxisAngleSelfTest_GetState(self_test) ==
             PITCH_AXIS_ANGLE_SELF_TEST_STATE_PASS) ||
            (PitchAxisAngleSelfTest_GetState(self_test) ==
             PITCH_AXIS_ANGLE_SELF_TEST_STATE_FAILED))
        {
            break;
        }
    }

    return PitchAxisAngleSelfTest_GetState(self_test);
}

static void reset_fake(void)
{
    g_init_result = AS5600_OK;
    g_probe_result = AS5600_OK;
    g_probe_bus_result = BSP_I2C_OK;
    g_samples = NULL;
    g_sample_count = 0U;
    g_sample_index = 0U;
}

static void test_normal_samples_track_wrap(void)
{
    static const FakeSample samples[] =
    {
        {AS5600_OK, 4090U, 4090, 0x20U},
        {AS5600_OK, 5U, 4101, 0x20U},
        {AS5600_OK, 6U, 4102, 0x20U}
    };
    PitchAxisAngleSelfTest self_test;
    PitchAxisAngleSelfTestReport report;

    reset_fake();
    set_samples(samples, 3U);
    assert(PitchAxisAngleSelfTest_Init(
               &self_test,
               &g_i2c,
               3U,
               2U,
               1U) == PITCH_AXIS_ANGLE_SELF_TEST_OK);
    assert(PitchAxisAngleSelfTest_Start(&self_test, 0U) ==
           PITCH_AXIS_ANGLE_SELF_TEST_OK);
    assert(run_until_terminal(&self_test, 0U, 20U) ==
           PITCH_AXIS_ANGLE_SELF_TEST_STATE_PASS);
    assert(PitchAxisAngleSelfTest_GetReport(&self_test, &report));
    assert(report.completed_samples == 3U);
    assert(report.valid_sample_count == 3U);
    assert(report.i2c_error_count == 0U);
    assert(report.magnet_error_count == 0U);
    assert(report.first_raw_count == 4090U);
    assert(report.latest_raw_count == 6U);
    assert(report.minimum_continuous_count == 4090);
    assert(report.maximum_continuous_count == 4102);
    assert(report.maximum_step_count == 11U);
}

static void test_probe_nack_fails_closed(void)
{
    PitchAxisAngleSelfTest self_test;
    PitchAxisAngleSelfTestReport report;

    reset_fake();
    g_probe_result = AS5600_TRANSPORT_ERROR;
    g_probe_bus_result = BSP_I2C_NACK;
    assert(PitchAxisAngleSelfTest_Init(
               &self_test,
               &g_i2c,
               1U,
               1U,
               0U) == PITCH_AXIS_ANGLE_SELF_TEST_OK);
    assert(PitchAxisAngleSelfTest_Start(&self_test, 0U) ==
           PITCH_AXIS_ANGLE_SELF_TEST_OK);
    assert(run_until_terminal(&self_test, 0U, 4U) ==
           PITCH_AXIS_ANGLE_SELF_TEST_STATE_FAILED);
    assert(PitchAxisAngleSelfTest_GetFailure(&self_test) ==
           PITCH_AXIS_ANGLE_SELF_TEST_FAILURE_I2C_NACK);
    assert(PitchAxisAngleSelfTest_GetReport(&self_test, &report));
    assert(report.i2c_error_count == 1U);
}

static void test_magnet_error_fails_closed(void)
{
    static const FakeSample samples[] =
    {
        {AS5600_MAGNET_TOO_WEAK, 123U, 123, 0x10U}
    };
    PitchAxisAngleSelfTest self_test;

    reset_fake();
    set_samples(samples, 1U);
    assert(PitchAxisAngleSelfTest_Init(
               &self_test,
               &g_i2c,
               1U,
               1U,
               0U) == PITCH_AXIS_ANGLE_SELF_TEST_OK);
    assert(PitchAxisAngleSelfTest_Start(&self_test, 0U) ==
           PITCH_AXIS_ANGLE_SELF_TEST_OK);
    assert(run_until_terminal(&self_test, 0U, 4U) ==
           PITCH_AXIS_ANGLE_SELF_TEST_STATE_FAILED);
    assert(PitchAxisAngleSelfTest_GetFailure(&self_test) ==
           PITCH_AXIS_ANGLE_SELF_TEST_FAILURE_MAGNET_WEAK);
}

static void test_tick_wrap_starts_and_completes(void)
{
    static const FakeSample samples[] =
    {
        {AS5600_OK, 1U, 1, 0x20U}
    };
    PitchAxisAngleSelfTest self_test;

    reset_fake();
    set_samples(samples, 1U);
    assert(PitchAxisAngleSelfTest_Init(
               &self_test,
               &g_i2c,
               1U,
               1U,
               2U) == PITCH_AXIS_ANGLE_SELF_TEST_OK);
    assert(PitchAxisAngleSelfTest_Start(&self_test, UINT32_MAX - 1U) ==
           PITCH_AXIS_ANGLE_SELF_TEST_OK);
    assert(run_until_terminal(&self_test, UINT32_MAX - 1U, 6U) ==
           PITCH_AXIS_ANGLE_SELF_TEST_STATE_PASS);
}

int main(void)
{
    test_normal_samples_track_wrap();
    test_probe_nack_fails_closed();
    test_magnet_error_fails_closed();
    test_tick_wrap_starts_and_completes();
    return 0;
}
