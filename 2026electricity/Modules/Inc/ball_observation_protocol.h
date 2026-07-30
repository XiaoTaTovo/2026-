#ifndef BALL_OBSERVATION_PROTOCOL_H
#define BALL_OBSERVATION_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Source: vision/maixcam_pro_ball/ball_protocol.py and the frozen
 * 20-byte BallObservation v2 frame supplied by the vision subsystem.
 */
#define BALL_OBSERVATION_PROTOCOL_VERSION 2U
#define BALL_OBSERVATION_MESSAGE_TYPE 0x20U
#define BALL_OBSERVATION_PAYLOAD_SIZE 8U
#define BALL_OBSERVATION_FRAME_SIZE 20U
#define BALL_OBSERVATION_INVALID_POSITION ((int16_t)-32768)
#define BALL_OBSERVATION_UNKNOWN_CAPTURE_AGE_MS 0xFFFFU

typedef enum
{
    BALL_OBSERVATION_REASON_OK = 0,
    BALL_OBSERVATION_REASON_NO_BALL = 1,
    BALL_OBSERVATION_REASON_LOW_CONFIDENCE = 2,
    BALL_OBSERVATION_REASON_AMBIGUOUS = 3,
    BALL_OBSERVATION_REASON_OUT_OF_ROI = 4,
    BALL_OBSERVATION_REASON_CALIBRATION_INVALID = 5,
    BALL_OBSERVATION_REASON_FRAME_STALE = 6,
    BALL_OBSERVATION_REASON_CAMERA_ERROR = 7,
    BALL_OBSERVATION_REASON_MODEL_ERROR = 8,
    BALL_OBSERVATION_REASON_POSITION_RANGE = 9,
    BALL_OBSERVATION_REASON_WARMUP = 10
} BallObservationReason;

typedef struct
{
    uint32_t tx_uptime_ms;
    uint32_t rx_complete_ms;
    int16_t x_0_1mm;
    uint16_t confidence_permille;
    uint16_t capture_age_ms;
    uint8_t sequence;
    BallObservationReason reason;
    bool valid;
} BallObservation;

typedef struct
{
    uint8_t buffer[BALL_OBSERVATION_FRAME_SIZE];
    uint8_t buffer_length;
    uint32_t accepted_frame_count;
    uint32_t crc_error_count;
    uint32_t format_error_count;
    uint32_t semantic_error_count;
    uint32_t resynchronization_count;
} BallObservationParser;

void BallObservationParser_Init(BallObservationParser *parser);

/*
 * Returns true only when one complete, CRC-valid, semantically valid frame is
 * decoded. A returned observation may have valid=false: that is a legitimate
 * camera failure frame and must not be replaced with an older coordinate.
 */
bool BallObservationParser_OnByte(
    BallObservationParser *parser,
    uint8_t byte,
    uint32_t rx_complete_ms,
    BallObservation *observation);

#endif
