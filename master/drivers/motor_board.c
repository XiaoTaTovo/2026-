#include "drivers/motor_board.h"

#include <limits.h>
#include <string.h>

#define MOTOR_BOARD_ADDRESS (0x0AU)
#define MOTOR_BOARD_FUNC_READ (0x03U)
#define MOTOR_BOARD_FUNC_WRITE_SINGLE (0x06U)
#define MOTOR_BOARD_FUNC_WRITE_MULTIPLE (0x10U)
#define MOTOR_BOARD_REG_SPEED (0x0000U)
#define MOTOR_BOARD_REG_ENCODER_CLEAR (0x0004U)
#define MOTOR_BOARD_REG_CLOSED_LOOP (0x0008U)
#define MOTOR_BOARD_REG_ENCODER_POLARITY (0x0009U)
#define MOTOR_BOARD_REG_PID (0x0015U)
#define MOTOR_BOARD_MAX_WRITE_REGISTERS (12U)
#define MOTOR_BOARD_PID_SCALE (1000.0f)
#define MOTOR_BOARD_ENCODER_ONLY_BYTES (8U)
#define MOTOR_BOARD_ENCODER_SPEED_BYTES (16U)

static bool MotorBoard_Send(MotorBoard *board,
                            const uint8_t *data,
                            uint8_t length)
{
    if ((board == 0) || (data == 0) || (length == 0U) ||
        (board->config.send == 0)) {
        return false;
    }
    if (!board->config.send(data, length, board->config.context)) {
        return false;
    }
    board->tx_frames++;
    return true;
}

static void MotorBoard_AppendU16(uint8_t *frame, uint8_t *index, uint16_t value)
{
    frame[(*index)++] = (uint8_t)(value >> 8);
    frame[(*index)++] = (uint8_t)value;
}

static void MotorBoard_AppendCrc(uint8_t *frame, uint8_t *index)
{
    uint16_t crc = MotorBoard_Crc16(frame, *index);
    frame[(*index)++] = (uint8_t)crc;
    frame[(*index)++] = (uint8_t)(crc >> 8);
}

static bool MotorBoard_WriteSingle(MotorBoard *board,
                                   uint16_t reg,
                                   uint16_t value)
{
    uint8_t frame[8];
    uint8_t index = 0U;

    frame[index++] = MOTOR_BOARD_ADDRESS;
    frame[index++] = MOTOR_BOARD_FUNC_WRITE_SINGLE;
    MotorBoard_AppendU16(frame, &index, reg);
    MotorBoard_AppendU16(frame, &index, value);
    MotorBoard_AppendCrc(frame, &index);
    return MotorBoard_Send(board, frame, index);
}

static bool MotorBoard_WriteRegisters(MotorBoard *board,
                                      uint16_t start_register,
                                      const uint16_t *values,
                                      uint8_t count)
{
    uint8_t frame[MOTOR_BOARD_MAX_FRAME_SIZE];
    uint8_t index = 0U;

    if ((board == 0) || (values == 0) || (count == 0U) ||
        (count > MOTOR_BOARD_MAX_WRITE_REGISTERS)) {
        return false;
    }
    frame[index++] = MOTOR_BOARD_ADDRESS;
    frame[index++] = MOTOR_BOARD_FUNC_WRITE_MULTIPLE;
    MotorBoard_AppendU16(frame, &index, start_register);
    MotorBoard_AppendU16(frame, &index, count);
    frame[index++] = (uint8_t)(count * 2U);
    for (uint8_t register_index = 0U; register_index < count;
         register_index++) {
        MotorBoard_AppendU16(frame, &index, values[register_index]);
    }
    MotorBoard_AppendCrc(frame, &index);
    return MotorBoard_Send(board, frame, index);
}

static bool MotorBoard_WriteFour(
    MotorBoard *board,
    uint16_t start_register,
    const int16_t values[MOTOR_BOARD_CHANNEL_COUNT])
{
    uint16_t raw[MOTOR_BOARD_CHANNEL_COUNT];

    for (uint8_t channel = 0U; channel < MOTOR_BOARD_CHANNEL_COUNT; channel++) {
        raw[channel] = (uint16_t)values[channel];
    }
    return MotorBoard_WriteRegisters(board, start_register, raw,
                                     MOTOR_BOARD_CHANNEL_COUNT);
}

static int16_t MotorBoard_ApplyInvert(int16_t value, bool inverted)
{
    if (!inverted) {
        return value;
    }
    return (value == INT16_MIN) ? INT16_MAX : (int16_t)(-value);
}

static uint16_t MotorBoard_PidToRaw(float value)
{
    if (value <= 0.0f) {
        return 0U;
    }
    if (value >= 65.535f) {
        return UINT16_MAX;
    }
    return (uint16_t)(value * MOTOR_BOARD_PID_SCALE + 0.5f);
}

static int16_t MotorBoard_ReadS16(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[0] << 8) | data[1]);
}

static void MotorBoard_ResetRx(MotorBoard *board)
{
    board->rx_length = 0U;
    board->expected_length = 0U;
}

static void MotorBoard_ParseEncoderFrame(MotorBoard *board)
{
    uint8_t byte_count = board->rx[2];
    uint8_t frame_size = (uint8_t)(byte_count + 5U);
    uint16_t received_crc;
    uint16_t calculated_crc;

    calculated_crc = MotorBoard_Crc16(board->rx, (uint16_t)(frame_size - 2U));
    received_crc = (uint16_t)board->rx[frame_size - 2U] |
                   ((uint16_t)board->rx[frame_size - 1U] << 8);
    if (calculated_crc != received_crc) {
        board->crc_errors++;
        return;
    }

    for (uint8_t channel = 0U; channel < MOTOR_BOARD_CHANNEL_COUNT; channel++) {
        board->channel_counts[channel] =
            MotorBoard_ReadS16(&board->rx[3U + channel * 2U]);
    }
    if (byte_count == MOTOR_BOARD_ENCODER_SPEED_BYTES) {
        for (uint8_t channel = 0U; channel < MOTOR_BOARD_CHANNEL_COUNT; channel++) {
            board->channel_speeds[channel] =
                MotorBoard_ReadS16(&board->rx[11U + channel * 2U]);
        }
        board->speed_valid = true;
    }
    board->encoder_timestamp_ms = board->last_rx_byte_ms;
    board->encoder_valid = true;
    board->rx_frames++;
}

void MotorBoard_Init(MotorBoard *board, const MotorBoardConfig *config)
{
    if ((board == 0) || (config == 0)) {
        return;
    }
    *board = (MotorBoard){0};
    board->config = *config;
}

bool MotorBoard_SetClosedLoop(MotorBoard *board, bool enable)
{
    bool sent;

    if (board == 0) {
        return false;
    }
    if (board->config.direct_set_wheel_speeds != 0) {
        board->closed_loop_enabled = enable;
        return true;
    }
    sent = MotorBoard_WriteSingle(board, MOTOR_BOARD_REG_CLOSED_LOOP,
                                  enable ? 1U : 0U);
    if (sent) {
        board->closed_loop_enabled = enable;
    }
    return sent;
}

bool MotorBoard_SetWheelSpeeds(MotorBoard *board,
                               int16_t left,
                               int16_t right)
{
    int16_t speeds[MOTOR_BOARD_CHANNEL_COUNT] = {0, 0, 0, 0};
    int16_t direct_left;
    int16_t direct_right;

    if ((board == 0) ||
        ((uint8_t)board->config.left_channel >= MOTOR_BOARD_CHANNEL_COUNT) ||
        ((uint8_t)board->config.right_channel >= MOTOR_BOARD_CHANNEL_COUNT)) {
        return false;
    }

    direct_left = MotorBoard_ApplyInvert(left, board->config.left_inverted);
    direct_right = MotorBoard_ApplyInvert(right, board->config.right_inverted);
    if (board->config.direct_set_wheel_speeds != 0) {
        return board->config.direct_set_wheel_speeds(
            direct_left, direct_right, board->config.direct_context);
    }

    speeds[board->config.left_channel] = direct_left;
    speeds[board->config.right_channel] = direct_right;

    return MotorBoard_WriteFour(board, MOTOR_BOARD_REG_SPEED, speeds);
}//设置左右轮子速度

bool MotorBoard_Stop(MotorBoard *board)
{
    return MotorBoard_SetWheelSpeeds(board, 0, 0);
}//停止轮子

bool MotorBoard_EmergencyStop(MotorBoard *board)
{
    /* The vendor protocol does not define 0x0008=0 as a reliable stop. */
    return MotorBoard_Stop(board);
}

bool MotorBoard_ClearEncoders(MotorBoard *board)
{
    const int16_t zeros[MOTOR_BOARD_CHANNEL_COUNT] = {0, 0, 0, 0};

    if (board == 0) {
        return false;
    }
    return MotorBoard_WriteFour(board, MOTOR_BOARD_REG_ENCODER_CLEAR, zeros);
}

bool MotorBoard_SetEncoderPolarity(MotorBoard *board,
                                   MotorBoardChannel channel,
                                   bool inverted)
{
    if ((board == 0) || ((uint8_t)channel >= MOTOR_BOARD_CHANNEL_COUNT)) {
        return false;
    }
    if (board->config.direct_set_wheel_speeds != 0) {
        (void)inverted;
        return true;
    }
    return MotorBoard_WriteSingle(board,
                                  (uint16_t)(MOTOR_BOARD_REG_ENCODER_POLARITY + channel),
                                  inverted ? 1U : 0U);
}

bool MotorBoard_SetAllPid(
    MotorBoard *board,
    const MotorBoardPid pid[MOTOR_BOARD_CHANNEL_COUNT])
{
    uint16_t raw[MOTOR_BOARD_CHANNEL_COUNT * 3U];
    uint8_t index = 0U;

    if ((board == 0) || (pid == 0)) {
        return false;
    }
    if (board->config.direct_set_wheel_speeds != 0) {
        return true;
    }
    for (uint8_t channel = 0U; channel < MOTOR_BOARD_CHANNEL_COUNT; channel++) {
        raw[index++] = MotorBoard_PidToRaw(pid[channel].kp);
        raw[index++] = MotorBoard_PidToRaw(pid[channel].ki);
        raw[index++] = MotorBoard_PidToRaw(pid[channel].kd);
    }
    return MotorBoard_WriteRegisters(board, MOTOR_BOARD_REG_PID, raw, index);
}

void MotorBoard_OnRxByte(MotorBoard *board, uint8_t byte, uint32_t now_ms)
{
    if (board == 0) {
        return;
    }
    if ((board->rx_length > 0U) &&
        ((uint32_t)(now_ms - board->last_rx_byte_ms) >
         board->config.rx_inter_byte_timeout_ms)) {
        board->format_errors++;
        MotorBoard_ResetRx(board);
    }
    board->last_rx_byte_ms = now_ms;

    if (board->rx_length == 0U) {
        if (byte == MOTOR_BOARD_ADDRESS) {
            board->rx[board->rx_length++] = byte;
        }
        return;
    }

    if (board->rx_length == 1U) {
        if (byte != MOTOR_BOARD_FUNC_READ) {
            board->format_errors++;
            MotorBoard_ResetRx(board);
            if (byte == MOTOR_BOARD_ADDRESS) {
                board->rx[board->rx_length++] = byte;
            }
            return;
        }
        board->rx[board->rx_length++] = byte;
        return;
    }

    if (board->rx_length == 2U) {
        if ((byte != MOTOR_BOARD_ENCODER_ONLY_BYTES) &&
            (byte != MOTOR_BOARD_ENCODER_SPEED_BYTES)) {
            board->format_errors++;
            MotorBoard_ResetRx(board);
            return;
        }
        board->expected_length = (uint8_t)(byte + 5U);
    }

    if (board->rx_length >= MOTOR_BOARD_MAX_FRAME_SIZE) {
        board->format_errors++;
        MotorBoard_ResetRx(board);
        return;
    }
    board->rx[board->rx_length++] = byte;

    if ((board->expected_length > 0U) &&
        (board->rx_length == board->expected_length)) {
        MotorBoard_ParseEncoderFrame(board);
        MotorBoard_ResetRx(board);
    }
}

bool MotorBoard_GetEncoderSample(const MotorBoard *board,
                                 CarEncoderSample *sample)
{
    int16_t left;
    int16_t right;
    uint32_t timestamp_ms;

    if ((board == 0) || (sample == 0)) {
        return false;
    }
    if (board->config.direct_get_encoder != 0) {
        if (!board->config.direct_get_encoder(
                &left, &right, &timestamp_ms, board->config.direct_context)) {
            return false;
        }
        sample->left_count = left;
        sample->right_count = right;
        sample->timestamp_ms = timestamp_ms;
        sample->valid = true;
        return true;
    }
    if (!board->encoder_valid) {
        return false;
    }
    left = board->channel_counts[board->config.left_channel];
    right = board->channel_counts[board->config.right_channel];
    sample->left_count = MotorBoard_ApplyInvert(left, board->config.left_inverted);
    sample->right_count = MotorBoard_ApplyInvert(right, board->config.right_inverted);
    sample->timestamp_ms = board->encoder_timestamp_ms;
    sample->valid = true;
    return true;
}

bool MotorBoard_GetWheelSpeeds(const MotorBoard *board,
                               int16_t *left,
                               int16_t *right,
                               uint32_t *timestamp_ms)
{
    if ((board == 0) || (left == 0) || (right == 0) ||
        (timestamp_ms == 0) || !board->speed_valid) {
        return false;
    }
    *left = MotorBoard_ApplyInvert(
        board->channel_speeds[board->config.left_channel],
        board->config.left_inverted);
    *right = MotorBoard_ApplyInvert(
        board->channel_speeds[board->config.right_channel],
        board->config.right_inverted);
    *timestamp_ms = board->encoder_timestamp_ms;
    return true;
}

uint16_t MotorBoard_Crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;

    if (data == 0) {
        return 0U;
    }
    for (uint16_t i = 0U; i < length; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0U; bit < 8U; bit++) {
            if ((crc & 1U) != 0U) {
                crc = (uint16_t)((crc >> 1U) ^ 0xA001U);
            } else {
                crc >>= 1U;
            }
        }
    }
    return crc;
}
