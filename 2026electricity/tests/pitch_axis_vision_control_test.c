#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "pitch_axis_vision_control.h"

static const PitchAxisVisionConfig config = {
    .target_position_0_1mm = 0,
    .minimum_safe_position_0_1mm = -1100,
    .maximum_safe_position_0_1mm = 1100,
    .edge_recovery_margin_0_1mm = 100,
    .deadband_0_1mm = 20,
    .velocity_deadband_0_1mm_s = 100,
    .minimum_confidence_permille = 700U,
    .maximum_observation_age_ms = 150U,
    .control_period_ms = 50U,
    .minimum_speed_rpm = 1U,
    .maximum_speed_rpm = 5U,
    .kp_rpm_per_mm = 0.03f,
    .kd_rpm_per_mm_s = 0.01f,
    .velocity_filter_alpha = 0.25f,
    .positive_error_uses_positive_direction = false
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
    result.tx_uptime_ms = rx_complete_ms + 10U;
    result.confidence_permille = confidence_permille;
    result.capture_age_ms = 10U;
    result.sequence = sequence;
    result.reason = BALL_OBSERVATION_REASON_OK;
    result.rx_complete_ms = rx_complete_ms;
    result.valid = true;
    return result;
}

static void test_velocity_candidate_and_direction(void)
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
    assert(report.ball_velocity_0_1mm_s == 0);
    assert(report.control_output_0_01rpm == 30); /* 10 mm * 0.03 RPM/mm */
    assert(report.command_speed_rpm == 1U); /* minimum realizable command */
    assert(!report.command_positive_direction); /* mapped to KEY3 / CCW */
    assert(report.decision_ready);
    assert(report.command_ready);
}

static void test_velocity_term_brakes_motion_toward_center(void)
{
    PitchAxisVisionControl control;
    PitchAxisVisionReport report;
    BallObservation frame;

    assert(PitchAxisVisionControl_Init(&control, &config, 0U));
    frame = observation(-100, 900U, 1U, 0U);
    PitchAxisVisionControl_OnObservation(&control, &frame);
    PitchAxisVisionControl_Service(&control, 50U);

    frame = observation(-50, 900U, 2U, 50U);
    PitchAxisVisionControl_OnObservation(&control, &frame);
    PitchAxisVisionControl_Service(&control, 100U);
    assert(PitchAxisVisionControl_GetReport(&control, &report));
    assert(report.ball_velocity_0_1mm_s == 250); /* filtered +25 mm/s */
    assert(report.control_output_0_01rpm == -10); /* reverse to brake */
    assert(report.command_speed_rpm == 1U);
    assert(report.command_positive_direction); /* mapped to KEY2 / CW */
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
    assert(report.command_speed_rpm == 0U);
    assert(report.decision_ready);
    assert(!report.command_ready);

    frame = observation(-30, 900U, 2U, 60U);
    PitchAxisVisionControl_OnObservation(&control, &frame);
    PitchAxisVisionControl_Service(&control, 100U);
    assert(PitchAxisVisionControl_GetReport(&control, &report));
    assert(report.command_ready);

    PitchAxisVisionControl_OnObservation(&control, &frame);
    PitchAxisVisionControl_Service(&control, 150U);
    assert(PitchAxisVisionControl_GetReport(&control, &report));
    assert(!report.decision_ready);
    assert(!report.command_ready);
    assert(report.duplicate_sequence_count == 1U);

    PitchAxisVisionControl_Service(&control, 250U);
    assert(PitchAxisVisionControl_GetReport(&control, &report));
    assert(report.state == PITCH_VISION_STATE_REJECT_STALE);
    assert(report.decision_ready);
    assert(!report.observation_fresh);
    assert(report.command_speed_rpm == 0U);
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
    assert(report.decision_ready);
    assert(!report.command_ready);
    assert(report.command_speed_rpm == 0U);

    frame.valid = false;
    frame.reason = BALL_OBSERVATION_REASON_NO_BALL;
    frame.x_0_1mm = BALL_OBSERVATION_INVALID_POSITION;
    PitchAxisVisionControl_OnObservation(&control, &frame);
    PitchAxisVisionControl_Service(&control, 100U);
    assert(PitchAxisVisionControl_GetReport(&control, &report));
    assert(report.state == PITCH_VISION_STATE_REJECT_INVALID);
    assert(report.decision_ready);
    assert(report.invalid_observation_count == 1U);
    assert(report.command_speed_rpm == 0U);
}

static void test_runtime_ball_position_limits(void)
{
    PitchAxisVisionControl control;
    PitchAxisVisionReport report;
    PitchAxisVisionConfig tuned = config;
    BallObservation frame;

    assert(PitchAxisVisionControl_Init(&control, &tuned, 0U));
    frame = observation(1101, 900U, 1U, 0U);
    PitchAxisVisionControl_OnObservation(&control, &frame);
    PitchAxisVisionControl_Service(&control, 50U);
    assert(PitchAxisVisionControl_GetReport(&control, &report));
    assert(report.ball_position_outside_limits);

    tuned.maximum_safe_position_0_1mm = 1200;
    assert(PitchAxisVisionControl_UpdateConfig(&control, &tuned, 50U));
    frame = observation(1101, 900U, 2U, 50U);
    PitchAxisVisionControl_OnObservation(&control, &frame);
    PitchAxisVisionControl_Service(&control, 100U);
    assert(PitchAxisVisionControl_GetReport(&control, &report));
    assert(!report.ball_position_outside_limits);

    frame.valid = false;
    frame.x_0_1mm = BALL_OBSERVATION_INVALID_POSITION;
    PitchAxisVisionControl_OnObservation(&control, &frame);
    PitchAxisVisionControl_Service(&control, 150U);
    assert(PitchAxisVisionControl_GetReport(&control, &report));
    assert(!report.ball_position_outside_limits);
}

static void test_runtime_pid_config_and_integral_term(void)
{
    PitchAxisVisionControl control;
    PitchAxisVisionReport report;
    PitchAxisVisionConfig tuned = config;
    PitchAxisVisionConfig readback;
    BallObservation frame;

    tuned.deadband_0_1mm = 0;
    tuned.ki_rpm_per_mm_s = 0.1f;
    tuned.integral_limit_rpm = 2.0f;
    assert(PitchAxisVisionControl_Init(&control, &tuned, 0U));

    frame = observation(-100, 900U, 1U, 0U);
    PitchAxisVisionControl_OnObservation(&control, &frame);
    PitchAxisVisionControl_Service(&control, 50U);
    frame = observation(-100, 900U, 2U, 50U);
    PitchAxisVisionControl_OnObservation(&control, &frame);
    PitchAxisVisionControl_Service(&control, 100U);
    assert(PitchAxisVisionControl_GetReport(&control, &report));
    assert(report.p_term_0_01rpm == 30);
    assert(report.i_term_0_01rpm == 5);
    assert(report.d_term_0_01rpm == 0);
    assert(report.control_output_0_01rpm == 35);

    tuned.kp_rpm_per_mm = 0.05f;
    assert(PitchAxisVisionControl_UpdateConfig(&control, &tuned, 100U));
    assert(PitchAxisVisionControl_GetConfig(&control, &readback));
    assert(readback.kp_rpm_per_mm == 0.05f);
    assert(readback.ki_rpm_per_mm_s == 0.1f);
    PitchAxisVisionControl_ResetController(&control, 100U);
    assert(PitchAxisVisionControl_GetReport(&control, &report));
    assert(report.i_term_0_01rpm == 0);
    assert(!report.command_ready);
}

static void test_edge_recovery_candidate_uses_position_direction(void)
{
    PitchAxisVisionControl control;
    PitchAxisVisionReport report;
    BallObservation frame;

    assert(PitchAxisVisionControl_Init(&control, &config, 0U));
    frame = observation(-1001, 900U, 1U, 0U);
    PitchAxisVisionControl_OnObservation(&control, &frame);
    PitchAxisVisionControl_Service(&control, 50U);
    assert(PitchAxisVisionControl_GetReport(&control, &report));
    assert(report.edge_recovery_candidate);
    assert(!report.edge_recovery_direction);

    frame = observation(0, 900U, 2U, 50U);
    PitchAxisVisionControl_OnObservation(&control, &frame);
    PitchAxisVisionControl_Service(&control, 100U);
    assert(PitchAxisVisionControl_GetReport(&control, &report));
    assert(!report.edge_recovery_candidate);

    frame = observation(1001, 900U, 3U, 100U);
    PitchAxisVisionControl_OnObservation(&control, &frame);
    PitchAxisVisionControl_Service(&control, 150U);
    assert(PitchAxisVisionControl_GetReport(&control, &report));
    assert(report.edge_recovery_candidate);
    assert(report.edge_recovery_direction);
}

static void test_decision_is_consumed_once_before_next_control_period(void)
{
    PitchAxisVisionControl control;
    PitchAxisVisionReport decision;
    BallObservation frame;

    assert(PitchAxisVisionControl_Init(&control, &config, 0U));
    frame = observation(-100, 900U, 1U, 0U);
    PitchAxisVisionControl_OnObservation(&control, &frame);
    PitchAxisVisionControl_Service(&control, 50U);

    assert(PitchAxisVisionControl_TakeDecision(&control, &decision));
    assert(decision.decision_sequence == 1U);
    assert(decision.command_ready);

    frame = observation(-90, 900U, 2U, 60U);
    PitchAxisVisionControl_OnObservation(&control, &frame);
    PitchAxisVisionControl_Service(&control, 75U);

    assert(!PitchAxisVisionControl_TakeDecision(&control, &decision));

    PitchAxisVisionControl_Service(&control, 100U);
    assert(PitchAxisVisionControl_TakeDecision(&control, &decision));
    assert(decision.decision_sequence == 2U);
    assert(decision.command_ready);
    assert(!PitchAxisVisionControl_TakeDecision(&control, &decision));
}

int main(void)
{
    test_velocity_candidate_and_direction();
    test_velocity_term_brakes_motion_toward_center();
    test_deadband_duplicate_and_stale_rejection();
    test_invalid_and_low_confidence_rejection();
    test_runtime_ball_position_limits();
    test_runtime_pid_config_and_integral_term();
    test_edge_recovery_candidate_uses_position_direction();
    test_decision_is_consumed_once_before_next_control_period();
    puts("PITCH_AXIS_VISION_CONTROL_TEST=PASS");
    return 0;
}
