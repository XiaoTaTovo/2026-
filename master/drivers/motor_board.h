#ifndef H2024_MOTOR_BOARD_H
#define H2024_MOTOR_BOARD_H

#include <stdbool.h>
#include <stdint.h>

#include "car_types.h"

#define MOTOR_BOARD_CHANNEL_COUNT (4U)
#define MOTOR_BOARD_MAX_FRAME_SIZE (34U)

typedef enum {
    MOTOR_BOARD_CHANNEL_A = 0,
    MOTOR_BOARD_CHANNEL_B,
    MOTOR_BOARD_CHANNEL_C,
    MOTOR_BOARD_CHANNEL_D
} MotorBoardChannel;

typedef struct {
    float kp;
    float ki;
    float kd;
} MotorBoardPid;

typedef bool (*MotorBoardSendFn)(const uint8_t *data,
                                 uint8_t length,
                                 void *context);

/* Optional direct motor backend. The serial protocol remains the default. */
typedef bool (*MotorBoardDirectSetWheelSpeedsFn)(int16_t left,
                                                 int16_t right,
                                                 void *context);
typedef bool (*MotorBoardDirectGetEncoderFn)(int16_t *left,
                                             int16_t *right,
                                             uint32_t *timestamp_ms,
                                             void *context);

typedef struct {
    MotorBoardSendFn send;
    void *context;
    MotorBoardChannel left_channel;
    MotorBoardChannel right_channel;
    bool left_inverted;
    bool right_inverted;
    uint32_t rx_inter_byte_timeout_ms;
    MotorBoardDirectSetWheelSpeedsFn direct_set_wheel_speeds;
    MotorBoardDirectGetEncoderFn direct_get_encoder;
    void *direct_context;
} MotorBoardConfig;

typedef struct {
    MotorBoardConfig config;
    uint8_t rx[MOTOR_BOARD_MAX_FRAME_SIZE];
    uint8_t rx_length;
    uint8_t expected_length;
    uint32_t last_rx_byte_ms;
    int16_t channel_counts[MOTOR_BOARD_CHANNEL_COUNT];
    int16_t channel_speeds[MOTOR_BOARD_CHANNEL_COUNT];
    uint32_t encoder_timestamp_ms;
    uint32_t tx_frames;
    uint32_t rx_frames;
    uint32_t crc_errors;
    uint32_t format_errors;
    bool encoder_valid;
    bool speed_valid;
    bool closed_loop_enabled;
} MotorBoard;

void MotorBoard_Init(MotorBoard *board, const MotorBoardConfig *config);
bool MotorBoard_SetClosedLoop(MotorBoard *board, bool enable);
bool MotorBoard_SetWheelSpeeds(MotorBoard *board,
                               int16_t left,
                               int16_t right);
bool MotorBoard_Stop(MotorBoard *board);
bool MotorBoard_EmergencyStop(MotorBoard *board);
bool MotorBoard_ClearEncoders(MotorBoard *board);
bool MotorBoard_SetEncoderPolarity(MotorBoard *board,
                                   MotorBoardChannel channel,
                                   bool inverted);
bool MotorBoard_SetAllPid(
    MotorBoard *board,
    const MotorBoardPid pid[MOTOR_BOARD_CHANNEL_COUNT]);
void MotorBoard_OnRxByte(MotorBoard *board, uint8_t byte, uint32_t now_ms);
bool MotorBoard_GetEncoderSample(const MotorBoard *board,
                                 CarEncoderSample *sample);
bool MotorBoard_GetWheelSpeeds(const MotorBoard *board,
                               int16_t *left,
                               int16_t *right,
                               uint32_t *timestamp_ms);
uint16_t MotorBoard_Crc16(const uint8_t *data, uint16_t length);

#endif
