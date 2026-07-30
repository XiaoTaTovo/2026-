#ifndef BSP_BLUETOOTH_H
#define BSP_BLUETOOTH_H

#include <stddef.h>
#include <stdint.h>

#include "bsp_uart_dma.h"

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
    BspUartDmaPort transport;
} BspBluetooth;

BspBluetoothResult BspBluetooth_Init(
    BspBluetooth *port,
    UART_HandleTypeDef *uart);

BspBluetoothResult BspBluetooth_Start(BspBluetooth *port);
void BspBluetooth_Service(BspBluetooth *port);

size_t BspBluetooth_Available(const BspBluetooth *port);
size_t BspBluetooth_TxFree(const BspBluetooth *port);

BspBluetoothResult BspBluetooth_Write(
    BspBluetooth *port,
    const uint8_t *data,
    size_t length);

BspBluetoothResult BspBluetooth_WriteString(
    BspBluetooth *port,
    const char *text);

BspBluetoothResult BspBluetooth_Read(
    BspBluetooth *port,
    uint8_t *data,
    size_t capacity,
    size_t *read_length);

#endif
