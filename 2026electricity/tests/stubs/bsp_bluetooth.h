#ifndef BSP_BLUETOOTH_H
#define BSP_BLUETOOTH_H

#include <stddef.h>
#include <stdint.h>

typedef enum
{
    BSP_BLUETOOTH_OK = 0,
    BSP_BLUETOOTH_EMPTY,
    BSP_BLUETOOTH_FULL,
    BSP_BLUETOOTH_INVALID_ARGUMENT,
    BSP_BLUETOOTH_INVALID_CONFIG,
    BSP_BLUETOOTH_HAL_ERROR
} BspBluetoothResult;

typedef struct
{
    uint32_t unused;
} BspBluetooth;

size_t BspBluetooth_Available(const BspBluetooth *port);
size_t BspBluetooth_TxFree(const BspBluetooth *port);

BspBluetoothResult BspBluetooth_Write(
    BspBluetooth *port,
    const uint8_t *data,
    size_t length);

BspBluetoothResult BspBluetooth_Read(
    BspBluetooth *port,
    uint8_t *data,
    size_t capacity,
    size_t *read_length);

#endif
