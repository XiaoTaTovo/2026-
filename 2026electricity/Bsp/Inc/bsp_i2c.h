#ifndef BSP_I2C_H
#define BSP_I2C_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

typedef enum
{
    BSP_I2C_OK = 0,
    BSP_I2C_INVALID_ARGUMENT,
    BSP_I2C_INVALID_CONFIG,
    BSP_I2C_BUSY,
    BSP_I2C_TIMEOUT,
    BSP_I2C_NACK,
    BSP_I2C_HAL_ERROR
} BspI2cResult;

typedef struct
{
    I2C_HandleTypeDef *handle;
    uint32_t timeout_ms;
    HAL_StatusTypeDef last_hal_status;
    uint32_t last_hal_error;
    uint32_t probe_count;
    uint32_t read_count;
    uint32_t error_count;
} BspI2c;

BspI2cResult BspI2c_Init(
    BspI2c *bus,
    I2C_HandleTypeDef *handle,
    uint32_t timeout_ms);

BspI2cResult BspI2c_Probe(
    BspI2c *bus,
    uint8_t address_7bit,
    uint32_t trials);

BspI2cResult BspI2c_ReadRegisters(
    BspI2c *bus,
    uint8_t address_7bit,
    uint8_t start_register,
    uint8_t *data,
    uint16_t length);

#endif
