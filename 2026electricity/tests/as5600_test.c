#include <assert.h>
#include <stdint.h>
#include <string.h>

#include "as5600.h"

static I2C_HandleTypeDef g_i2c;
static BspI2cResult g_probe_result = BSP_I2C_OK;
static BspI2cResult g_read_result = BSP_I2C_OK;
static uint8_t g_status;
static uint16_t g_raw_count;
static uint8_t g_agc;
static uint16_t g_magnitude;

BspI2cResult BspI2c_Init(
    BspI2c *bus,
    I2C_HandleTypeDef *handle,
    uint32_t timeout_ms)
{
    if ((bus == NULL) || (handle == NULL) || (timeout_ms == 0U))
    {
        return BSP_I2C_INVALID_ARGUMENT;
    }

    memset(bus, 0, sizeof(*bus));
    bus->handle = handle;
    bus->timeout_ms = timeout_ms;
    return BSP_I2C_OK;
}

BspI2cResult BspI2c_Probe(
    BspI2c *bus,
    uint8_t address_7bit,
    uint32_t trials)
{
    assert(bus != NULL);
    assert(address_7bit == AS5600_I2C_ADDRESS_7BIT);
    assert(trials == 2U);
    return g_probe_result;
}

BspI2cResult BspI2c_ReadRegisters(
    BspI2c *bus,
    uint8_t address_7bit,
    uint8_t start_register,
    uint8_t *data,
    uint16_t length)
{
    assert(bus != NULL);
    assert(address_7bit == AS5600_I2C_ADDRESS_7BIT);

    if (g_read_result != BSP_I2C_OK)
    {
        return g_read_result;
    }

    if ((start_register == 0x0BU) && (length == 3U))
    {
        data[0] = g_status;
        data[1] = (uint8_t)(g_raw_count >> 8U);
        data[2] = (uint8_t)g_raw_count;
        return BSP_I2C_OK;
    }
    if ((start_register == 0x1AU) && (length == 3U))
    {
        data[0] = g_agc;
        data[1] = (uint8_t)(g_magnitude >> 8U);
        data[2] = (uint8_t)g_magnitude;
        return BSP_I2C_OK;
    }

    return BSP_I2C_INVALID_ARGUMENT;
}

static void set_healthy_raw(uint16_t raw_count)
{
    g_status = 0x20U;
    g_raw_count = raw_count;
    g_read_result = BSP_I2C_OK;
}

static void test_angle_tracking_and_zero(void)
{
    As5600 encoder;
    As5600Sample sample;

    assert(As5600_Init(&encoder, &g_i2c) == AS5600_OK);
    assert(As5600_Probe(&encoder) == AS5600_OK);

    set_healthy_raw(4090U);
    assert(As5600_ReadSample(&encoder, 10U) == AS5600_OK);
    assert(As5600_SetZeroHere(&encoder) == AS5600_OK);

    set_healthy_raw(5U);
    assert(As5600_ReadSample(&encoder, 20U) == AS5600_OK);
    assert(As5600_GetLatestSample(&encoder, &sample));
    assert(sample.valid);
    assert(sample.continuous_count == 4101);
    assert(sample.relative_degrees > 0.9F);
    assert(sample.relative_degrees < 1.0F);

    assert(As5600_SetDirection(
               &encoder,
               AS5600_DIRECTION_COUNTERCLOCKWISE) == AS5600_OK);
    assert(As5600_GetLatestSample(&encoder, &sample));
    assert(sample.relative_degrees < -0.9F);
    assert(sample.relative_degrees > -1.0F);
}

static void test_invalid_magnet_does_not_advance_tracking(void)
{
    As5600 encoder;
    As5600Sample sample;

    assert(As5600_Init(&encoder, &g_i2c) == AS5600_OK);
    set_healthy_raw(100U);
    assert(As5600_ReadSample(&encoder, 1U) == AS5600_OK);

    g_status = 0U;
    g_raw_count = 2000U;
    assert(As5600_ReadSample(&encoder, 2U) == AS5600_MAGNET_NOT_DETECTED);
    assert(As5600_GetLatestSample(&encoder, &sample));
    assert(!sample.valid);
    assert(sample.continuous_count == 100);
    assert(encoder.continuous_count == 100);
}

static void test_diagnostics_and_transport_error(void)
{
    As5600 encoder;
    As5600Diagnostics diagnostics;

    assert(As5600_Init(&encoder, &g_i2c) == AS5600_OK);
    g_agc = 0x42U;
    g_magnitude = 0x0ABCU;
    assert(As5600_ReadDiagnostics(&encoder, &diagnostics) == AS5600_OK);
    assert(diagnostics.automatic_gain_control == 0x42U);
    assert(diagnostics.magnitude == 0x0ABCU);

    g_read_result = BSP_I2C_TIMEOUT;
    assert(As5600_ReadSample(&encoder, 3U) == AS5600_TRANSPORT_ERROR);
    assert(encoder.transport_error_count == 1U);
}

int main(void)
{
    test_angle_tracking_and_zero();
    test_invalid_magnet_does_not_advance_tracking();
    test_diagnostics_and_transport_error();
    return 0;
}
