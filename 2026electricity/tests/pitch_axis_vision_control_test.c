#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "pitch_axis_vision_control.h"

static const PitchAxisVisionConfig config = {
    .target_position_0_1mm = 0,
    .deadband_0_1mm = 5,
    .minimum_confidence_permille = 700U,
    .maximum_observation_age_ms = 150U,
    .control_period_ms = 50U,
    .pulses_per_mm = 960U,
    .minimum_command_pulses = 8U,
    .kp_mm_per_mm = 0.10f,
    .ki_per_s = 0.0f,
    .kd_s = 0.0f,
    .integral_limit_mm_s = 10.0f,
    .output_limit_mm = 0.20f,
    .positive_error_uses_positive_direction = true
};

static BallObservation observation(
    int16_t x_0_1mm,
    uint16_t confidence_permille,
    uint8_t sequence,
    uint32_t rx_complete_ms)
{
    BallObservation result;

    memset(&result, 0, sizeof(result));
    result.x_0_1mm = x_0_1mm;
    result.confidence_permille = confidence_permille;
    result.capture_age_ms = 10U;
    result.sequence = sequence;
    result.reason = BALL_OBSERVATION_REASON_OK;
    result.rx_complete_ms = rx_complete_ms;
    result.valid = true;
    return result;
}

static void test_pulse_calibration_and_direction(void)
{
    PitchAxisVisionControl control;
    PitchAxisVisionReport report;
    BallObservation frame;

    assert(PitchAxisVisionControl_Init(&control, &config, 0U));
    frame = observation(-100, 900U, 1U, 0U); /* +10.0 mm error */
    PitchAxisVisionControl_OnObservation(&control, &frame);
    PitchAxisVisionControl_Service(&control, 50U);
    assert(PitchAxisVisionControl_GetReport(&control, &report));
    assert(report.state == PITCH_VISION_STATE_TRACKING);
    assert(report.observation_fresh);
    assert(report.error_0_1mm == 100);
    assert(report.output_0_001mm == 200); /* output cap: +0.200 mm */
    assert(report.command_pulses == 192U); /* 0.200 mm * 960 pulse/mm */
    assert(report.command_positive_direction);
    assert(report.command_ready);
}

static void test_deadband_duplicate_and_stale_rejection(void)
{
    PitchAxisVisionControl control;
    PitchAxisVisionReport report;
    BallObservation frame;

    assert(PitchAxisVisionControl_Init(&control, &config, 0U));
    frame = observation(3, 900U, 1U, 0U);
    PitchAxisVisionControl_OnObservation(&control, &frame);
    PitchAxisVisionControl_Service(&control, 50U);
    assert(PitchAxisVisionControl_GetReport(&control, &report));
    assert(report.command_pulses == 0U);
    assert(!report.command_ready);

    frame = observation(-10, 900U, 2U, 60U);
    PitchAxisVisionControl_OnObservation(&control, &frame);
    PitchAxisVisionControl_Service(&control, 100U);
    assert(PitchAxisVisionControl_GetReport(&control, &report));
    assert(report.command_ready);

    PitchAxisVisionControl_OnObservation(&control, &frame);
    PitchAxisVisionControl_Service(&control, 150U);
    assert(PitchAxisVisionControl_GetReport(&control, &report));
    assert(!report.command_ready);
    assert(report.duplicate_sequence_count == 1U);

    PitchAxisVisionControl_Service(&control, 250U);
    assert(PitchAxisVisionControl_GetReport(&control, &report));
    assert(report.state == PITCH_VISION_STATE_REJECT_STALE);
    assert(!report.observation_fresh);
}

static void test_invalid_and_low_confidence_rejection(void)
{
    PitchAxisVisionControl control;
    PitchAxisVisionReport report;
    BallObservation frame;

    assert(PitchAxisVisionControl_Init(&control, &config, 0U));
    frame = observation(-10, 650U, 1U, 0U);
    PitchAxisVisionControl_OnObservation(&control, &frame);
    PitchAxisVisionControl_Service(&control, 50U);
    assert(PitchAxisVisionControl_GetReport(&control, &report));
    assert(report.state == PITCH_VISION_STATE_REJECT_LOW_CONFIDENCE);
    assert(!report.command_ready);

    frame.valid = false;
    frame.reason = BALL_OBSERVATION_REASON_NO_BALL;
    frame.x_0_1mm = BALL_OBSERVATION_INVALID_POSITION;
    PitchAxisVisionControl_OnObservation(&control, &frame);
    PitchAxisVisionControl_Service(&control, 100U);
    assert(PitchAxisVisionControl_GetReport(&control, &report));
    assert(report.state == PITCH_VISION_STATE_REJECT_INVALID);
    assert(report.invalid_observation_count == 1U);
}

int main(void)
{
    test_pulse_calibration_and_direction();
    test_deadband_duplicate_and_stale_rejection();
    test_invalid_and_low_confidence_rejection();
    puts("PITCH_AXIS_VISION_CONTROL_TEST=PASS");
    return 0;
}
