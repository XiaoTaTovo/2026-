#include "pitch_axis_vision_control.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static bool time_elapsed(uint32_t now_ms, uint32_t start_ms, uint32_t delay_ms)
{
    return (uint32_t)(now_ms - start_ms) >= delay_ms;
}

static float clamp_float(float value, float lower, float upper)
{
    if (value < lower)
    {
        return lower;
    }
    if (value > upper)
    {
        return upper;
    }
    return value;
}

static int16_t round_to_i16(float value)
{
    if (value >= 32767.0f)
    {
        return 32767;
    }
    if (value <= -32768.0f)
    {
        return -32768;
    }
    return (int16_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

static bool valid_config(const PitchAxisVisionConfig *config)
{
    return (config != NULL) &&
        (config->minimum_safe_position_0_1mm <
         config->maximum_safe_position_0_1mm) &&
        (config->target_position_0_1mm >=
         config->minimum_safe_position_0_1mm) &&
        (config->target_position_0_1mm <=
         config->maximum_safe_position_0_1mm) &&
        (config->edge_recovery_margin_0_1mm >= 0) &&
        (((int32_t)config->edge_recovery_margin_0_1mm * 2) <
         ((int32_t)config->maximum_safe_position_0_1mm -
          (int32_t)config->minimum_safe_position_0_1mm)) &&
        (config->deadband_0_1mm >= 0) &&
        (config->velocity_deadband_0_1mm_s >= 0) &&
        (config->control_period_ms != 0U) &&
        (config->maximum_observation_age_ms != 0U) &&
        (config->minimum_speed_rpm != 0U) &&
        (config->maximum_speed_rpm >= config->minimum_speed_rpm) &&
        (config->kp_rpm_per_mm >= 0.0f) &&
        (config->ki_rpm_per_mm_s >= 0.0f) &&
        (config->kd_rpm_per_mm_s >= 0.0f) &&
        (config->integral_limit_rpm >= 0.0f) &&
        (config->integral_separation_band_0_1mm >= 0) &&
        (config->approach_band_0_1mm >= 0) &&
        ((config->approach_band_0_1mm == 0) ||
        ((config->approach_speed_limit_rpm >= config->minimum_speed_rpm) &&
         (config->approach_speed_limit_rpm <= config->maximum_speed_rpm))) &&
        (config->velocity_filter_alpha > 0.0f) &&
        (config->velocity_filter_alpha <= 1.0f) &&
        (!config->feedforward_enabled ||
         (((config->feedforward_sign == 1) ||
           (config->feedforward_sign == -1)) &&
          (config->feedforward_gain_rpm_per_mm_s2 >= 0.0f) &&
          (config->feedforward_limit_rpm >= 0.0f) &&
          (config->feedforward_deadband_mm_s2 >= 0.0f)));
}

static uint32_t observation_age_ms(
    const BallObservation *observation,
    uint32_t now_ms)
{
    uint32_t receive_age = now_ms - observation->rx_complete_ms;
    uint32_t capture_age = observation->capture_age_ms;

    if (receive_age > (UINT32_MAX - capture_age))
    {
        return UINT32_MAX;
    }
    return receive_age + capture_age;
}

static void suspend_control_state(
    PitchAxisVisionControl *control,
    bool clear_integral)
{
    control->previous_error_mm = 0.0f;
    if (clear_integral)
    {
        control->integral_error_mm_s = 0.0f;
    }
    control->filtered_error_velocity_mm_s = 0.0f;
    control->previous_sample_ms = 0U;
    control->have_previous_sample = false;
    control->report.ball_velocity_0_1mm_s = 0;
    control->report.p_term_0_01rpm = 0;
    control->report.i_term_0_01rpm = 0;
    control->report.d_term_0_01rpm = 0;
    control->report.unsaturated_output_0_01rpm = 0;
    control->report.control_output_0_01rpm = 0;
    control->report.feedforward_0_01rpm = 0;
    control->report.feedforward_input_mm_s2 = 0;
    control->report.feedforward_valid = false;
    control->report.feedforward_saturated = false;
    control->report.integral_active = false;
    control->report.approach_limited = false;
    control->report.output_saturated = false;
    control->report.command_speed_rpm = 0U;
    control->report.command_positive_direction = false;
    control->report.command_ready = false;
}

static void clear_control_state(PitchAxisVisionControl *control)
{
    suspend_control_state(control, true);
    control->have_trusted_observation = false;
    control->last_trusted_observation_rx_ms = 0U;
}

static bool trusted_observation_timed_out(
    const PitchAxisVisionControl *control,
    const BallObservation *observation)
{
    return control->have_trusted_observation &&
        time_elapsed(
            observation->rx_complete_ms,
            control->last_trusted_observation_rx_ms,
            control->config.maximum_observation_age_ms);
}

bool PitchAxisVisionControl_Init(
    PitchAxisVisionControl *control,
    const PitchAxisVisionConfig *config,
    uint32_t now_ms)
{
    if ((control == NULL) || !valid_config(config))
    {
        return false;
    }

    memset(control, 0, sizeof(*control));
    control->config = *config;
    control->last_control_ms = now_ms;
    control->report.state = PITCH_VISION_STATE_WAITING_FOR_FRAME;
    control->initialized = true;
    return true;
}

bool PitchAxisVisionControl_GetConfig(
    const PitchAxisVisionControl *control,
    PitchAxisVisionConfig *config)
{
    if ((control == NULL) || (config == NULL) || !control->initialized)
    {
        return false;
    }

    *config = control->config;
    return true;
}

bool PitchAxisVisionControl_UpdateConfig(
    PitchAxisVisionControl *control,
    const PitchAxisVisionConfig *config,
    uint32_t now_ms)
{
    if ((control == NULL) || !control->initialized || !valid_config(config))
    {
        return false;
    }

    control->config = *config;
    control->last_control_ms = now_ms;
    clear_control_state(control);
    return true;
}

void PitchAxisVisionControl_ResetController(
    PitchAxisVisionControl *control,
    uint32_t now_ms)
{
    if ((control == NULL) || !control->initialized)
    {
        return;
    }

    control->last_control_ms = now_ms;
    clear_control_state(control);
}

void PitchAxisVisionControl_SetFeedforwardInput(
    PitchAxisVisionControl *control,
    float acceleration_mm_s2,
    bool valid)
{
    if ((control == NULL) || !control->initialized)
    {
        return;
    }
    control->feedforward_input_mm_s2 = acceleration_mm_s2;
    control->feedforward_input_valid = valid;
}

void PitchAxisVisionControl_OnObservation(
    PitchAxisVisionControl *control,
    const BallObservation *observation)
{
    if ((control == NULL) || (observation == NULL) || !control->initialized)
    {
        return;
    }

    control->pending_observation = *observation;
    control->pending_observation_present = true;
    control->pending_observation_generation++;
    control->report.observation_present = true;
    control->report.command_ready = false;
    if (control->have_last_received_sequence &&
        (observation->sequence == control->last_received_sequence))
    {
        control->report.duplicate_sequence_count++;
    }
    control->last_received_sequence = observation->sequence;
    control->have_last_received_sequence = true;
    if (!observation->valid)
    {
        control->report.invalid_observation_count++;
        suspend_control_state(
            control,
            trusted_observation_timed_out(control, observation));
    }
    else
    {
        control->report.accepted_observation_count++;
        if (observation->confidence_permille <
            control->config.minimum_confidence_permille)
        {
            control->report.low_confidence_count++;
            suspend_control_state(
                control,
                trusted_observation_timed_out(control, observation));
        }
        else
        {
            control->last_trusted_observation_rx_ms =
                observation->rx_complete_ms;
            control->have_trusted_observation = true;
        }
    }
}

void PitchAxisVisionControl_Service(
    PitchAxisVisionControl *control,
    uint32_t now_ms)
{
    const PitchAxisVisionConfig *config;
    BallObservation *observation;
    float error_mm;
    float error_velocity_mm_s = 0.0f;
    float ball_velocity_mm_s;
    float p_output_rpm;
    float i_output_rpm;
    float d_output_rpm;
    float feedforward_output_rpm;
    float unsaturated_output_rpm;
    float output_rpm;
    float integral_candidate_mm_s;
    float sample_delta_s = 0.0f;
    float speed_rpm;
    uint32_t sample_ms;
    uint32_t sample_delta_ms;
    bool new_observation;

    if ((control == NULL) || !control->initialized)
    {
        return;
    }
    if (!time_elapsed(
            now_ms,
            control->last_control_ms,
            control->config.control_period_ms))
    {
        return;
    }

    control->last_control_ms = now_ms;
    control->report.decision_ready = false;
    control->report.command_ready = false;
    config = &control->config;
    observation = &control->pending_observation;

    if (!control->pending_observation_present)
    {
        control->report.state = PITCH_VISION_STATE_WAITING_FOR_FRAME;
        control->report.observation_fresh = false;
        control->report.ball_position_outside_limits = false;
        suspend_control_state(control, false);
        return;
    }

    control->report.observation = *observation;
    control->report.ball_position_outside_limits = observation->valid &&
        ((observation->x_0_1mm < config->minimum_safe_position_0_1mm) ||
         (observation->x_0_1mm > config->maximum_safe_position_0_1mm));
    control->report.observation_age_ms = observation_age_ms(observation, now_ms);
    new_observation =
        control->pending_observation_generation !=
        control->last_decision_generation;
    if (control->report.observation_age_ms >
        control->report.maximum_observation_age_ms)
    {
        control->report.maximum_observation_age_ms =
            control->report.observation_age_ms;
    }
    if (!observation->valid)
    {
        control->report.state = PITCH_VISION_STATE_REJECT_INVALID;
        control->report.observation_fresh = false;
        suspend_control_state(
            control,
            trusted_observation_timed_out(control, observation));
        if (new_observation)
        {
            control->last_decision_generation =
                control->pending_observation_generation;
            control->report.decision_sequence = observation->sequence;
            control->report.decision_ready = true;
        }
        return;
    }
    if (observation->confidence_permille < config->minimum_confidence_permille)
    {
        control->report.state = PITCH_VISION_STATE_REJECT_LOW_CONFIDENCE;
        control->report.observation_fresh = false;
        suspend_control_state(
            control,
            trusted_observation_timed_out(control, observation));
        if (new_observation)
        {
            control->last_decision_generation =
                control->pending_observation_generation;
            control->report.decision_sequence = observation->sequence;
            control->report.decision_ready = true;
        }
        return;
    }
    if (control->report.observation_age_ms > config->maximum_observation_age_ms)
    {
        if (control->report.state != PITCH_VISION_STATE_REJECT_STALE)
        {
            control->report.stale_observation_count++;
            control->report.decision_sequence = observation->sequence;
            control->report.decision_ready = true;
        }
        control->report.state = PITCH_VISION_STATE_REJECT_STALE;
        control->report.observation_fresh = false;
        suspend_control_state(control, true);
        if (new_observation)
        {
            control->last_decision_generation =
                control->pending_observation_generation;
        }
        return;
    }

    control->report.observation_fresh = true;
    control->report.state = PITCH_VISION_STATE_TRACKING;
    control->report.feedforward_input_mm_s2 = round_to_i16(
        control->feedforward_input_mm_s2);
    control->report.feedforward_valid =
        control->feedforward_input_valid && config->feedforward_enabled;
    control->report.feedforward_saturated = false;
    error_mm = ((float)config->target_position_0_1mm -
                (float)observation->x_0_1mm) / 10.0f;
    control->report.error_0_1mm = round_to_i16(error_mm * 10.0f);
    control->report.edge_recovery_candidate =
        (observation->x_0_1mm <=
         (config->minimum_safe_position_0_1mm +
          config->edge_recovery_margin_0_1mm)) ||
        (observation->x_0_1mm >=
         (config->maximum_safe_position_0_1mm -
          config->edge_recovery_margin_0_1mm));
    control->report.edge_recovery_direction = (error_mm >= 0.0f) ?
        config->positive_error_uses_positive_direction :
        !config->positive_error_uses_positive_direction;

    if (!new_observation ||
        (control->have_last_command_sequence &&
        (observation->sequence == control->last_command_sequence))
       )
    {
        if (new_observation)
        {
            control->last_decision_generation =
                control->pending_observation_generation;
        }
        return;
    }
    control->last_decision_generation =
        control->pending_observation_generation;
    control->last_command_sequence = observation->sequence;
    control->have_last_command_sequence = true;
    control->report.decision_sequence = observation->sequence;
    control->report.decision_ready = true;

    sample_ms = observation->tx_uptime_ms - observation->capture_age_ms;
    if (control->have_previous_sample)
    {
        sample_delta_ms = sample_ms - control->previous_sample_ms;
        if ((sample_delta_ms >= 20U) && (sample_delta_ms <= 500U))
        {
            sample_delta_s = (float)sample_delta_ms / 1000.0f;
            float raw_error_velocity =
                (error_mm - control->previous_error_mm) * 1000.0f /
                (float)sample_delta_ms;
            error_velocity_mm_s =
                config->velocity_filter_alpha * raw_error_velocity +
                (1.0f - config->velocity_filter_alpha) *
                    control->filtered_error_velocity_mm_s;
        }
    }
    control->filtered_error_velocity_mm_s = error_velocity_mm_s;
    control->previous_error_mm = error_mm;
    control->previous_sample_ms = sample_ms;
    control->have_previous_sample = true;
    ball_velocity_mm_s = -error_velocity_mm_s;
    control->report.ball_velocity_0_1mm_s =
        round_to_i16(ball_velocity_mm_s * 10.0f);

    integral_candidate_mm_s = control->integral_error_mm_s;
    control->report.integral_active = false;
    if ((config->ki_rpm_per_mm_s > 0.0f) &&
        ((config->integral_separation_band_0_1mm == 0) ||
         (fabsf(error_mm) <=
          ((float)config->integral_separation_band_0_1mm / 10.0f))))
    {
        control->report.integral_active = true;
        if (sample_delta_s > 0.0f)
        {
            integral_candidate_mm_s += error_mm * sample_delta_s;
        }
        i_output_rpm = config->ki_rpm_per_mm_s * integral_candidate_mm_s;
        i_output_rpm = clamp_float(
            i_output_rpm,
            -config->integral_limit_rpm,
            config->integral_limit_rpm);
        integral_candidate_mm_s = i_output_rpm /
            config->ki_rpm_per_mm_s;
    }
    else
    {
        integral_candidate_mm_s = 0.0f;
        i_output_rpm = 0.0f;
    }
    control->integral_error_mm_s = integral_candidate_mm_s;

    p_output_rpm = config->kp_rpm_per_mm * error_mm;
    d_output_rpm = config->kd_rpm_per_mm_s * error_velocity_mm_s;
    feedforward_output_rpm = 0.0f;
    if (control->report.feedforward_valid &&
        (fabsf(control->feedforward_input_mm_s2) >=
         config->feedforward_deadband_mm_s2))
    {
        feedforward_output_rpm =
            (float)config->feedforward_sign *
            config->feedforward_gain_rpm_per_mm_s2 *
            control->feedforward_input_mm_s2;
        if (feedforward_output_rpm > config->feedforward_limit_rpm)
        {
            feedforward_output_rpm = config->feedforward_limit_rpm;
            control->report.feedforward_saturated = true;
        }
        else if (feedforward_output_rpm < -config->feedforward_limit_rpm)
        {
            feedforward_output_rpm = -config->feedforward_limit_rpm;
            control->report.feedforward_saturated = true;
        }
    }
    control->report.feedforward_0_01rpm = round_to_i16(
        feedforward_output_rpm * 100.0f);
    unsaturated_output_rpm = p_output_rpm + i_output_rpm + d_output_rpm +
        feedforward_output_rpm;
    control->report.p_term_0_01rpm = round_to_i16(p_output_rpm * 100.0f);
    control->report.i_term_0_01rpm = round_to_i16(i_output_rpm * 100.0f);
    control->report.d_term_0_01rpm = round_to_i16(d_output_rpm * 100.0f);
    control->report.unsaturated_output_0_01rpm =
        round_to_i16(unsaturated_output_rpm * 100.0f);

    if ((fabsf(error_mm) <= ((float)config->deadband_0_1mm / 10.0f)) &&
        (fabsf(ball_velocity_mm_s) <=
         ((float)config->velocity_deadband_0_1mm_s / 10.0f)))
    {
        output_rpm = clamp_float(
            i_output_rpm + feedforward_output_rpm,
            -(float)config->maximum_speed_rpm,
            (float)config->maximum_speed_rpm);
        control->report.control_output_0_01rpm =
            round_to_i16(output_rpm * 100.0f);
        speed_rpm = fabsf(output_rpm);
        if ((speed_rpm > 0.0f) &&
            (speed_rpm < (float)config->minimum_speed_rpm))
        {
            speed_rpm = (float)config->minimum_speed_rpm;
        }
        control->report.command_speed_rpm = (uint16_t)(speed_rpm + 0.5f);
        control->report.command_positive_direction =
            (output_rpm >= 0.0f) ?
                config->positive_error_uses_positive_direction :
                !config->positive_error_uses_positive_direction;
        control->report.approach_limited = false;
        control->report.output_saturated = false;
        if (control->report.command_speed_rpm != 0U)
        {
            control->report.command_ready = true;
            control->report.command_candidate_count++;
        }
        return;
    }

    output_rpm = unsaturated_output_rpm;
    control->report.approach_limited = false;
    if ((config->approach_band_0_1mm > 0) &&
        (fabsf(error_mm) <=
         ((float)config->approach_band_0_1mm / 10.0f)))
    {
        bool moving_toward_target =
            (error_mm * ball_velocity_mm_s) > 0.0f;
        bool accelerating_toward_target =
            (error_mm * output_rpm) > 0.0f;
        if (moving_toward_target && accelerating_toward_target)
        {
            float limited_output = clamp_float(
                output_rpm,
                -(float)config->approach_speed_limit_rpm,
                (float)config->approach_speed_limit_rpm);
            control->report.approach_limited =
                fabsf(limited_output - output_rpm) > 0.0001f;
            output_rpm = limited_output;
        }
    }
    output_rpm = clamp_float(
        output_rpm,
        -(float)config->maximum_speed_rpm,
        (float)config->maximum_speed_rpm);
    control->report.output_saturated = control->report.approach_limited ||
        (fabsf(output_rpm - unsaturated_output_rpm) > 0.0001f);
    control->report.control_output_0_01rpm =
        round_to_i16(output_rpm * 100.0f);
    speed_rpm = fabsf(output_rpm);
    if ((speed_rpm > 0.0f) &&
        (speed_rpm < (float)config->minimum_speed_rpm))
    {
        speed_rpm = (float)config->minimum_speed_rpm;
    }
    control->report.command_speed_rpm = (uint16_t)(speed_rpm + 0.5f);
    control->report.command_positive_direction =
        (output_rpm >= 0.0f) ? config->positive_error_uses_positive_direction :
                              !config->positive_error_uses_positive_direction;
    if (control->report.command_speed_rpm != 0U)
    {
        control->report.command_ready = true;
        control->report.command_candidate_count++;
    }
}

bool PitchAxisVisionControl_GetReport(
    const PitchAxisVisionControl *control,
    PitchAxisVisionReport *report)
{
    if ((control == NULL) || (report == NULL) || !control->initialized)
    {
        return false;
    }
    *report = control->report;
    return true;
}

bool PitchAxisVisionControl_TakeDecision(
    PitchAxisVisionControl *control,
    PitchAxisVisionReport *report)
{
    if ((control == NULL) || (report == NULL) || !control->initialized ||
        !control->report.decision_ready)
    {
        return false;
    }

    *report = control->report;
    control->report.decision_ready = false;
    return true;
}
