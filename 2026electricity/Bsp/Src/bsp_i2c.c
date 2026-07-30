#include "bsp_i2c.h"

#include <string.h>

static BspI2cResult map_hal_status(BspI2c *bus, HAL_StatusTypeDef status)
{
    bus->last_hal_status = status;
    bus->last_hal_error = HAL_I2C_GetError(bus->handle);

    if (status == HAL_OK)
    {
        return BSP_I2C_OK;
    }

    bus->error_count++;

    if (status == HAL_BUSY)
    {
        return BSP_I2C_BUSY;
    }
    if (status == HAL_TIMEOUT)
    {
        return BSP_I2C_TIMEOUT;
    }
    if ((bus->last_hal_error & HAL_I2C_ERROR_AF) != 0U)
    {
        return BSP_I2C_NACK;
    }

    return BSP_I2C_HAL_ERROR;
}

static bool valid_address(uint8_t address_7bit)
{
    return (address_7bit > 0U) && (address_7bit <= 0x7FU);
}

BspI2cResult BspI2c_Init(
    BspI2c *bus,
    I2C_HandleTypeDef *handle,
    uint32_t timeout_ms)
{
    if ((bus == NULL) || (handle == NULL))
    {
        return BSP_I2C_INVALID_ARGUMENT;
    }
    if (timeout_ms == 0U)
    {
        return BSP_I2C_INVALID_CONFIG;
    }

    memset(bus, 0, sizeof(*bus));
    bus->handle = handle;
    bus->timeout_ms = timeout_ms;
    bus->last_hal_status = HAL_OK;
    return BSP_I2C_OK;
}

BspI2cResult BspI2c_Probe(
    BspI2c *bus,
    uint8_t address_7bit,
    uint32_t trials)
{
    HAL_StatusTypeDef status;

    if (bus == NULL)
    {
        return BSP_I2C_INVALID_ARGUMENT;
    }
    if ((bus->handle == NULL) || (bus->timeout_ms == 0U) ||
        !valid_address(address_7bit) || (trials == 0U))
    {
        return BSP_I2C_INVALID_CONFIG;
    }

    bus->probe_count++;
    status = HAL_I2C_IsDeviceReady(
        bus->handle,
        (uint16_t)address_7bit << 1U,
        trials,
        bus->timeout_ms);
    return map_hal_status(bus, status);
}

BspI2cResult BspI2c_ReadRegisters(
    BspI2c *bus,
    uint8_t address_7bit,
    uint8_t start_register,
    uint8_t *data,
    uint16_t length)
{
    HAL_StatusTypeDef status;

    if ((bus == NULL) || (data == NULL) || (length == 0U))
    {
        return BSP_I2C_INVALID_ARGUMENT;
    }
    if ((bus->handle == NULL) || (bus->timeout_ms == 0U) ||
        !valid_address(address_7bit))
    {
        return BSP_I2C_INVALID_CONFIG;
    }

    bus->read_count++;
    status = HAL_I2C_Mem_Read(
        bus->handle,
        (uint16_t)address_7bit << 1U,
        start_register,
        I2C_MEMADD_SIZE_8BIT,
        data,
        length,
        bus->timeout_ms);
    return map_hal_status(bus, status);
}
