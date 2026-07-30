#include "as5600.h"

#include <string.h>

/*
 * Source: ams OSRAM AS5600 datasheet, register map and status register.
 * The sensor is fixed at I2C address 0x36 and reports a 12-bit raw angle.
 */
#define AS5600_STATUS_REGISTER 0x0BU
#define AS5600_RAW_ANGLE_MSB_REGISTER 0x0CU
#define AS5600_AGC_REGISTER 0x1AU
#define AS5600_MAGNITUDE_MSB_REGISTER 0x1BU
#define AS5600_STATUS_MAGNET_DETECTED 0x20U
#define AS5600_STATUS_MAGNET_TOO_WEAK 0x10U
#define AS5600_STATUS_MAGNET_TOO_STRONG 0x08U
#define AS5600_ADDRESS_PROBE_TRIALS 2U

static As5600Result map_bus_result(BspI2cResult result)
{
    return (result == BSP_I2C_OK) ? AS5600_OK : AS5600_TRANSPORT_ERROR;
}

static float counts_to_degrees(int32_t counts)
{
    return ((float)counts * 360.0F) / (float)AS5600_COUNTS_PER_REVOLUTION;
}

static void update_relative_angle(As5600 *encoder, As5600Sample *sample)
{
    int32_t reference_count;

    reference_count = encoder->zeroed ? encoder->zero_count : 0;
    sample->continuous_count = encoder->continuous_count;
    sample->relative_degrees = counts_to_degrees(
        ((int32_t)encoder->direction) *
        (encoder->continuous_count - reference_count));
    sample->zeroed = encoder->zeroed;
}

static As5600Result magnet_result(const As5600Sample *sample)
{
    if (!sample->magnet_detected)
    {
        return AS5600_MAGNET_NOT_DETECTED;
    }
    if (sample->magnet_too_weak)
    {
        return AS5600_MAGNET_TOO_WEAK;
    }
    if (sample->magnet_too_strong)
    {
        return AS5600_MAGNET_TOO_STRONG;
    }

    return AS5600_OK;
}

static void update_tracking(As5600 *encoder, uint16_t raw_count)
{
    int32_t delta;

    if (!encoder->has_valid_sample)
    {
        encoder->continuous_count = (int32_t)raw_count;
        encoder->previous_raw_count = raw_count;
        encoder->has_valid_sample = true;
        return;
    }

    delta = (int32_t)raw_count - (int32_t)encoder->previous_raw_count;
    if (delta > ((int32_t)AS5600_COUNTS_PER_REVOLUTION / 2))
    {
        delta -= (int32_t)AS5600_COUNTS_PER_REVOLUTION;
    }
    else if (delta < -((int32_t)AS5600_COUNTS_PER_REVOLUTION / 2))
    {
        delta += (int32_t)AS5600_COUNTS_PER_REVOLUTION;
    }

    encoder->continuous_count += delta;
    encoder->previous_raw_count = raw_count;
}

As5600Result As5600_Init(
    As5600 *encoder,
    I2C_HandleTypeDef *i2c)
{
    BspI2cResult bus_result;

    if ((encoder == NULL) || (i2c == NULL))
    {
        return AS5600_INVALID_ARGUMENT;
    }

    memset(encoder, 0, sizeof(*encoder));
    bus_result = BspI2c_Init(&encoder->bus, i2c, AS5600_I2C_TIMEOUT_MS);
    encoder->last_bus_result = bus_result;
    encoder->last_result = map_bus_result(bus_result);
    if (bus_result != BSP_I2C_OK)
    {
        return encoder->last_result;
    }

    encoder->direction = AS5600_DIRECTION_CLOCKWISE;
    encoder->initialized = true;
    encoder->last_result = AS5600_OK;
    return AS5600_OK;
}

As5600Result As5600_Probe(As5600 *encoder)
{
    BspI2cResult bus_result;

    if (encoder == NULL)
    {
        return AS5600_INVALID_ARGUMENT;
    }
    if (!encoder->initialized)
    {
        return AS5600_NOT_INITIALIZED;
    }

    bus_result = BspI2c_Probe(
        &encoder->bus,
        AS5600_I2C_ADDRESS_7BIT,
        AS5600_ADDRESS_PROBE_TRIALS);
    encoder->last_bus_result = bus_result;
    encoder->last_result = map_bus_result(bus_result);
    if (bus_result != BSP_I2C_OK)
    {
        encoder->transport_error_count++;
    }

    return encoder->last_result;
}

As5600Result As5600_ReadSample(
    As5600 *encoder,
    uint32_t timestamp_ms)
{
    uint8_t registers[3];
    BspI2cResult bus_result;
    As5600Sample sample;

    if (encoder == NULL)
    {
        return AS5600_INVALID_ARGUMENT;
    }
    if (!encoder->initialized)
    {
        return AS5600_NOT_INITIALIZED;
    }

    bus_result = BspI2c_ReadRegisters(
        &encoder->bus,
        AS5600_I2C_ADDRESS_7BIT,
        AS5600_STATUS_REGISTER,
        registers,
        sizeof(registers));
    encoder->last_bus_result = bus_result;
    if (bus_result != BSP_I2C_OK)
    {
        encoder->transport_error_count++;
        encoder->last_result = map_bus_result(bus_result);
        return encoder->last_result;
    }

    memset(&sample, 0, sizeof(sample));
    sample.status = registers[0];
    sample.magnet_detected =
        (sample.status & AS5600_STATUS_MAGNET_DETECTED) != 0U;
    sample.magnet_too_weak =
        (sample.status & AS5600_STATUS_MAGNET_TOO_WEAK) != 0U;
    sample.magnet_too_strong =
        (sample.status & AS5600_STATUS_MAGNET_TOO_STRONG) != 0U;
    sample.raw_count =
        (((uint16_t)registers[1] << 8U) | registers[2]) &
        (AS5600_COUNTS_PER_REVOLUTION - 1U);
    sample.absolute_degrees = counts_to_degrees((int32_t)sample.raw_count);
    sample.timestamp_ms = timestamp_ms;
    sample.valid = (magnet_result(&sample) == AS5600_OK);

    if (sample.valid)
    {
        update_tracking(encoder, sample.raw_count);
    }
    update_relative_angle(encoder, &sample);

    encoder->latest = sample;
    encoder->sample_count++;
    encoder->last_result = magnet_result(&sample);
    return encoder->last_result;
}

As5600Result As5600_ReadDiagnostics(
    As5600 *encoder,
    As5600Diagnostics *diagnostics)
{
    uint8_t registers[3];
    BspI2cResult bus_result;

    if ((encoder == NULL) || (diagnostics == NULL))
    {
        return AS5600_INVALID_ARGUMENT;
    }
    if (!encoder->initialized)
    {
        return AS5600_NOT_INITIALIZED;
    }

    bus_result = BspI2c_ReadRegisters(
        &encoder->bus,
        AS5600_I2C_ADDRESS_7BIT,
        AS5600_AGC_REGISTER,
        registers,
        sizeof(registers));
    encoder->last_bus_result = bus_result;
    encoder->last_result = map_bus_result(bus_result);
    if (bus_result != BSP_I2C_OK)
    {
        encoder->transport_error_count++;
        return encoder->last_result;
    }

    diagnostics->automatic_gain_control = registers[0];
    diagnostics->magnitude =
        (((uint16_t)registers[1] << 8U) | registers[2]) & 0x0FFFU;
    return AS5600_OK;
}

As5600Result As5600_SetZeroHere(As5600 *encoder)
{
    if (encoder == NULL)
    {
        return AS5600_INVALID_ARGUMENT;
    }
    if (!encoder->initialized)
    {
        return AS5600_NOT_INITIALIZED;
    }
    if (!encoder->has_valid_sample || !encoder->latest.valid)
    {
        return AS5600_NO_VALID_SAMPLE;
    }

    encoder->zero_count = encoder->continuous_count;
    encoder->zeroed = true;
    update_relative_angle(encoder, &encoder->latest);
    encoder->last_result = AS5600_OK;
    return AS5600_OK;
}

void As5600_ClearZero(As5600 *encoder)
{
    if (encoder == NULL)
    {
        return;
    }

    encoder->zeroed = false;
    update_relative_angle(encoder, &encoder->latest);
}

As5600Result As5600_SetDirection(
    As5600 *encoder,
    As5600Direction direction)
{
    if (encoder == NULL)
    {
        return AS5600_INVALID_ARGUMENT;
    }
    if (!encoder->initialized)
    {
        return AS5600_NOT_INITIALIZED;
    }
    if ((direction != AS5600_DIRECTION_CLOCKWISE) &&
        (direction != AS5600_DIRECTION_COUNTERCLOCKWISE))
    {
        return AS5600_INVALID_ARGUMENT;
    }

    encoder->direction = direction;
    update_relative_angle(encoder, &encoder->latest);
    encoder->last_result = AS5600_OK;
    return AS5600_OK;
}

bool As5600_GetLatestSample(
    const As5600 *encoder,
    As5600Sample *sample)
{
    if ((encoder == NULL) || (sample == NULL) ||
        (encoder->sample_count == 0U))
    {
        return false;
    }

    *sample = encoder->latest;
    return true;
}
