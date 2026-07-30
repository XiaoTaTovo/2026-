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

/*
 * The controller's coordinate convention is fixed here:
 * error = target_ball_position - measured_ball_position.
 * A positive control output is translated to the configured positive EMM
 * direction by the application after the low-energy direction test.
 */
typedef struct
{
    int16_t target_position_0_1mm;
    int16_t deadband_0_1mm;
    uint16_t minimum_confidence_permille;
    uint16_t maximum_observation_age_ms;
    uint32_t control_period_ms;
    uint32_t pulses_per_mm;
    uint32_t minimum_command_pulses;
    float kp_mm_per_mm;
    float ki_per_s;
    float kd_s;
    float integral_limit_mm_s;
    float output_limit_mm;
    bool positive_error_uses_positive_direction;
} PitchAxisVisionConfig;

typedef struct
{
    PitchAxisVisionState state;
    bool observation_present;
    bool observation_fresh;
    bool command_ready;
    BallObservation observation;
    int16_t error_0_1mm;
    int16_t output_0_001mm;
    uint32_t observation_age_ms;
    uint32_t command_pulses;
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
    float integral_mm_s;
    float previous_error_mm;
    uint32_t last_control_ms;
    uint8_t last_command_sequence;
    uint8_t last_received_sequence;
    bool have_last_command_sequence;
    bool have_last_received_sequence;
    bool first_control;
    bool initialized;
} PitchAxisVisionControl;

bool PitchAxisVisionControl_Init(
    PitchAxisVisionControl *control,
    const PitchAxisVisionConfig *config,
    uint32_t now_ms);

/* The caller supplies only complete frames returned by BallObservationParser. */
void PitchAxisVisionControl_OnObservation(
    PitchAxisVisionControl *control,
    const BallObservation *observation);

/* Produces at most one bounded relative-motion candidate per new camera frame. */
void PitchAxisVisionControl_Service(
    PitchAxisVisionControl *control,
    uint32_t now_ms);

bool PitchAxisVisionControl_GetReport(
    const PitchAxisVisionControl *control,
    PitchAxisVisionReport *report);

#endif
