#ifndef PITCH_AXIS_VISION_CONTROL_H
#define PITCH_AXIS_VISION_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "ball_observation_protocol.h"

typedef enum
{
    PITCH_VISION_STATE_WAITING_FOR_FRAME = 0,
    PITCH_VISION_STATE_TRACKING,
    PITCH_VISION_STATE_REJECT_INVALID,
    PITCH_VISION_STATE_REJECT_STALE,
    PITCH_VISION_STATE_REJECT_LOW_CONFIDENCE
} PitchAxisVisionState;

/* error = target_ball_position - measured_ball_position. */
typedef struct
{
    int16_t target_position_0_1mm;
    int16_t minimum_safe_position_0_1mm;
    int16_t maximum_safe_position_0_1mm;
    int16_t edge_recovery_margin_0_1mm;
    int16_t deadband_0_1mm;
    int16_t velocity_deadband_0_1mm_s;
    uint16_t minimum_confidence_permille;
    uint16_t maximum_observation_age_ms;
    uint32_t control_period_ms;
    uint16_t minimum_speed_rpm;
    uint16_t maximum_speed_rpm;
    float kp_rpm_per_mm;
    float ki_rpm_per_mm_s;
    float kd_rpm_per_mm_s;
    float integral_limit_rpm;
    float velocity_filter_alpha;
    bool positive_error_uses_positive_direction;
} PitchAxisVisionConfig;

typedef struct
{
    PitchAxisVisionState state;
    bool observation_present;
    bool observation_fresh;
    bool ball_position_outside_limits;
    bool edge_recovery_candidate;
    bool edge_recovery_direction;
    bool decision_ready;
    bool command_ready;
    uint8_t decision_sequence;
    BallObservation observation;
    int16_t error_0_1mm;
    int16_t ball_velocity_0_1mm_s;
    int16_t p_term_0_01rpm;
    int16_t i_term_0_01rpm;
    int16_t d_term_0_01rpm;
    int16_t unsaturated_output_0_01rpm;
    int16_t control_output_0_01rpm;
    bool output_saturated;
    uint32_t observation_age_ms;
    uint32_t maximum_observation_age_ms;
    uint16_t command_speed_rpm;
    bool command_positive_direction;
    uint32_t accepted_observation_count;
    uint32_t invalid_observation_count;
    uint32_t stale_observation_count;
    uint32_t low_confidence_count;
    uint32_t duplicate_sequence_count;
    uint32_t command_candidate_count;
} PitchAxisVisionReport;

typedef struct
{
    PitchAxisVisionConfig config;
    PitchAxisVisionReport report;
    BallObservation pending_observation;
    float previous_error_mm;
    float integral_error_mm_s;
    float filtered_error_velocity_mm_s;
    uint32_t previous_sample_ms;
    uint32_t last_control_ms;
    uint8_t last_command_sequence;
    uint8_t last_received_sequence;
    uint32_t pending_observation_generation;
    uint32_t last_decision_generation;
    bool have_last_command_sequence;
    bool have_last_received_sequence;
    bool have_previous_sample;
    bool pending_observation_present;
    bool initialized;
} PitchAxisVisionControl;

bool PitchAxisVisionControl_Init(
    PitchAxisVisionControl *control,
    const PitchAxisVisionConfig *config,
    uint32_t now_ms);

bool PitchAxisVisionControl_GetConfig(
    const PitchAxisVisionControl *control,
    PitchAxisVisionConfig *config);

bool PitchAxisVisionControl_UpdateConfig(
    PitchAxisVisionControl *control,
    const PitchAxisVisionConfig *config,
    uint32_t now_ms);

void PitchAxisVisionControl_ResetController(
    PitchAxisVisionControl *control,
    uint32_t now_ms);

/* The caller supplies only complete frames returned by BallObservationParser. */
void PitchAxisVisionControl_OnObservation(
    PitchAxisVisionControl *control,
    const BallObservation *observation);

/* Produces at most one bounded velocity candidate per new camera frame. */
void PitchAxisVisionControl_Service(
    PitchAxisVisionControl *control,
    uint32_t now_ms);

bool PitchAxisVisionControl_GetReport(
    const PitchAxisVisionControl *control,
    PitchAxisVisionReport *report);

/* Returns each control decision once while GetReport remains a snapshot API. */
bool PitchAxisVisionControl_TakeDecision(
    PitchAxisVisionControl *control,
    PitchAxisVisionReport *report);

#endif
