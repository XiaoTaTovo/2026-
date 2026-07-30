#include "bsp_bluetooth.h"

#include <string.h>

/*
 * Source:
 * C:/Users/taowz/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.3/
 * Projects/STM32F412G-Discovery/Examples/UART/
 * UART_TwoBoards_ComDMA/Src/main.c
 *
 * Adaptation:
 * Bluetooth owns no HAL callback. It delegates byte transport to the shared
 * UART DMA router so USART2/3 X42S and USART6 MaixCAM can reuse the same rules.
 */

static BspBluetoothResult map_result(BspUartDmaResult result)
{
    switch (result)
    {
        case BSP_UART_DMA_OK:
            return BSP_BLUETOOTH_OK;
        case BSP_UART_DMA_EMPTY:
            return BSP_BLUETOOTH_EMPTY;
        case BSP_UART_DMA_FULL:
            return BSP_BLUETOOTH_FULL;
        case BSP_UART_DMA_INVALID_ARGUMENT:
            return BSP_BLUETOOTH_INVALID_ARGUMENT;
        case BSP_UART_DMA_INVALID_CONFIG:
            return BSP_BLUETOOTH_INVALID_CONFIG;
        default:
            return BSP_BLUETOOTH_HAL_ERROR;
    }
}

BspBluetoothResult BspBluetooth_Init(
    BspBluetooth *port,
    UART_HandleTypeDef *uart)
{
    if (port == NULL)
    {
        return BSP_BLUETOOTH_INVALID_ARGUMENT;
    }

    return map_result(BspUartDma_Init(&port->transport, uart));
}

BspBluetoothResult BspBluetooth_Start(BspBluetooth *port)
{
    if (port == NULL)
    {
        return BSP_BLUETOOTH_INVALID_ARGUMENT;
    }

    return map_result(BspUartDma_Start(&port->transport));
}

void BspBluetooth_Service(BspBluetooth *port)
{
    if (port != NULL)
    {
        BspUartDma_Service(&port->transport);
    }
}

size_t BspBluetooth_Available(const BspBluetooth *port)
{
    return (port == NULL) ? 0U : BspUartDma_RxAvailable(&port->transport);
}

size_t BspBluetooth_TxFree(const BspBluetooth *port)
{
    return (port == NULL) ? 0U : BspUartDma_TxFree(&port->transport);
}

BspBluetoothResult BspBluetooth_Write(
    BspBluetooth *port,
    const uint8_t *data,
    size_t length)
{
    if (port == NULL)
    {
        return BSP_BLUETOOTH_INVALID_ARGUMENT;
    }

    return map_result(BspUartDma_Write(&port->transport, data, length));
}

BspBluetoothResult BspBluetooth_WriteString(
    BspBluetooth *port,
    const char *text)
{
    if (text == NULL)
    {
        return BSP_BLUETOOTH_INVALID_ARGUMENT;
    }

    return BspBluetooth_Write(port, (const uint8_t *)text, strlen(text));
}

BspBluetoothResult BspBluetooth_Read(
    BspBluetooth *port,
    uint8_t *data,
    size_t capacity,
    size_t *read_length)
{
    size_t count;

    if ((port == NULL) || (read_length == NULL) ||
        ((data == NULL) && (capacity != 0U)))
    {
        return BSP_BLUETOOTH_INVALID_ARGUMENT;
    }

    count = BspUartDma_Read(&port->transport, data, capacity);
    *read_length = count;
    return (count == 0U) ? BSP_BLUETOOTH_EMPTY : BSP_BLUETOOTH_OK;
}
