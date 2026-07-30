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

static void clear_pid_terms(PitchAxisVisionControl *control)
{
    control->integral_mm_s = 0.0f;
    control->previous_error_mm = 0.0f;
    control->first_control = true;
}

bool PitchAxisVisionControl_Init(
    PitchAxisVisionControl *control,
    const PitchAxisVisionConfig *config,
    uint32_t now_ms)
{
    if ((control == NULL) || (config == NULL) ||
        (config->deadband_0_1mm < 0) ||
        (config->control_period_ms == 0U) ||
        (config->pulses_per_mm == 0U) ||
        (config->minimum_command_pulses == 0U) ||
        (config->maximum_observation_age_ms == 0U) ||
        (config->integral_limit_mm_s < 0.0f) ||
        (config->output_limit_mm <= 0.0f))
    {
        return false;
    }

    memset(control, 0, sizeof(*control));
    control->config = *config;
    control->last_control_ms = now_ms;
    control->first_control = true;
    control->report.state = PITCH_VISION_STATE_WAITING_FOR_FRAME;
    control->initialized = true;
    return true;
}

void PitchAxisVisionControl_OnObservation(
    PitchAxisVisionControl *control,
    const BallObservation *observation)
{
    if ((control == NULL) || (observation == NULL) || !control->initialized)
    {
        return;
    }

    control->report.observation = *observation;
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
        control->report.state = PITCH_VISION_STATE_REJECT_INVALID;
        clear_pid_terms(control);
    }
    else
    {
        control->report.accepted_observation_count++;
        if (observation->confidence_permille <
            control->config.minimum_confidence_permille)
        {
            control->report.low_confidence_count++;
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
    float output_mm;
    float derivative_mm_per_s = 0.0f;
    float dt_s;
    uint32_t pulses;

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

    dt_s = (float)(now_ms - control->last_control_ms) / 1000.0f;
    control->last_control_ms = now_ms;
    control->report.command_ready = false;
    config = &control->config;
    observation = &control->report.observation;

    if (!control->report.observation_present)
    {
        control->report.state = PITCH_VISION_STATE_WAITING_FOR_FRAME;
        control->report.observation_fresh = false;
        clear_pid_terms(control);
        return;
    }

    control->report.observation_age_ms = observation_age_ms(observation, now_ms);
    if (!observation->valid)
    {
        control->report.state = PITCH_VISION_STATE_REJECT_INVALID;
        control->report.observation_fresh = false;
        clear_pid_terms(control);
        return;
    }
    if (observation->confidence_permille < config->minimum_confidence_permille)
    {
        control->report.state = PITCH_VISION_STATE_REJECT_LOW_CONFIDENCE;
        control->report.observation_fresh = false;
        clear_pid_terms(control);
        return;
    }
    if (control->report.observation_age_ms > config->maximum_observation_age_ms)
    {
        if (control->report.state != PITCH_VISION_STATE_REJECT_STALE)
        {
            control->report.stale_observation_count++;
        }
        control->report.state = PITCH_VISION_STATE_REJECT_STALE;
        control->report.observation_fresh = false;
        clear_pid_terms(control);
        return;
    }

    control->report.observation_fresh = true;
    control->report.state = PITCH_VISION_STATE_TRACKING;
    error_mm = ((float)config->target_position_0_1mm -
                (float)observation->x_0_1mm) / 10.0f;
    control->report.error_0_1mm = round_to_i16(error_mm * 10.0f);

    if (fabsf(error_mm) <= ((float)config->deadband_0_1mm / 10.0f))
    {
        control->integral_mm_s = 0.0f;
        control->previous_error_mm = error_mm;
        control->first_control = false;
        control->report.output_0_001mm = 0;
        control->report.command_pulses = 0U;
        return;
    }

    if (!control->first_control && (dt_s > 0.0f))
    {
        derivative_mm_per_s =
            (error_mm - control->previous_error_mm) / dt_s;
    }

    control->integral_mm_s = clamp_float(
        control->integral_mm_s + error_mm * dt_s,
        -config->integral_limit_mm_s,
        config->integral_limit_mm_s);
    output_mm = config->kp_mm_per_mm * error_mm +
                config->ki_per_s * control->integral_mm_s +
                config->kd_s * derivative_mm_per_s;
    output_mm = clamp_float(
        output_mm,
        -config->output_limit_mm,
        config->output_limit_mm);
    control->previous_error_mm = error_mm;
    control->first_control = false;
    control->report.output_0_001mm = round_to_i16(output_mm * 1000.0f);

    if (control->have_last_command_sequence &&
        (observation->sequence == control->last_command_sequence))
    {
        return;
    }

    pulses = (uint32_t)(fabsf(output_mm) *
                        (float)config->pulses_per_mm + 0.5f);
    if ((pulses != 0U) && (pulses < config->minimum_command_pulses))
    {
        pulses = config->minimum_command_pulses;
    }

    control->last_command_sequence = observation->sequence;
    control->have_last_command_sequence = true;
    control->report.command_pulses = pulses;
    control->report.command_positive_direction =
        (output_mm >= 0.0f) ? config->positive_error_uses_positive_direction :
                             !config->positive_error_uses_positive_direction;
    if (pulses != 0U)
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
