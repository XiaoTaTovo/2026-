#ifndef AS5600_H
#define AS5600_H

#include <stdbool.h>
#include <stdint.h>

#include "bsp_i2c.h"

#define AS5600_I2C_ADDRESS_7BIT 0x36U
#define AS5600_COUNTS_PER_REVOLUTION 4096U
#define AS5600_I2C_TIMEOUT_MS 5U

typedef enum
{
    AS5600_OK = 0,
    AS5600_INVALID_ARGUMENT,
    AS5600_NOT_INITIALIZED,
    AS5600_NO_VALID_SAMPLE,
    AS5600_TRANSPORT_ERROR,
    AS5600_MAGNET_NOT_DETECTED,
    AS5600_MAGNET_TOO_WEAK,
    AS5600_MAGNET_TOO_STRONG
} As5600Result;

typedef enum
{
    AS5600_DIRECTION_CLOCKWISE = 1,
    AS5600_DIRECTION_COUNTERCLOCKWISE = -1
} As5600Direction;

typedef struct
{
    bool valid;
    bool magnet_detected;
    bool magnet_too_weak;
    bool magnet_too_strong;
    bool zeroed;
    uint8_t status;
    uint16_t raw_count;
    int32_t continuous_count;
    float absolute_degrees;
    float relative_degrees;
    uint32_t timestamp_ms;
} As5600Sample;

typedef struct
{
    uint8_t automatic_gain_control;
    uint16_t magnitude;
} As5600Diagnostics;

typedef struct
{
    BspI2c bus;
    As5600Sample latest;
    int32_t continuous_count;
    int32_t zero_count;
    uint16_t previous_raw_count;
    As5600Direction direction;
    As5600Result last_result;
    BspI2cResult last_bus_result;
    uint32_t sample_count;
    uint32_t transport_error_count;
    bool initialized;
    bool has_valid_sample;
    bool zeroed;
} As5600;

As5600Result As5600_Init(
    As5600 *encoder,
    I2C_HandleTypeDef *i2c);

As5600Result As5600_Probe(As5600 *encoder);

As5600Result As5600_ReadSample(
    As5600 *encoder,
    uint32_t timestamp_ms);

As5600Result As5600_ReadDiagnostics(
    As5600 *encoder,
    As5600Diagnostics *diagnostics);

As5600Result As5600_SetZeroHere(As5600 *encoder);

void As5600_ClearZero(As5600 *encoder);

As5600Result As5600_SetDirection(
    As5600 *encoder,
    As5600Direction direction);

bool As5600_GetLatestSample(
    const As5600 *encoder,
    As5600Sample *sample);

#endif
