#ifndef BSP_UART_DMA_H
#define BSP_UART_DMA_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct
{
    uint32_t instance;
} UART_HandleTypeDef;

typedef enum
{
    BSP_UART_DMA_OK = 0,
    BSP_UART_DMA_EMPTY,
    BSP_UART_DMA_FULL,
    BSP_UART_DMA_INVALID_ARGUMENT,
    BSP_UART_DMA_INVALID_CONFIG,
    BSP_UART_DMA_HAL_ERROR
} BspUartDmaResult;

typedef struct
{
    UART_HandleTypeDef *uart;
    uint8_t rx_data[32];
    size_t rx_length;
    size_t rx_offset;
    uint32_t tx_bytes;
    uint32_t rx_bytes;
    uint32_t uart_error_count;
    uint32_t dma_start_error_count;
    uint32_t last_hal_error;
    uint32_t rx_overflow_count;
    bool started;
} BspUartDmaPort;

BspUartDmaResult BspUartDma_Init(
    BspUartDmaPort *port,
    UART_HandleTypeDef *uart);

BspUartDmaResult BspUartDma_Start(BspUartDmaPort *port);
void BspUartDma_Service(BspUartDmaPort *port);

size_t BspUartDma_Read(
    BspUartDmaPort *port,
    uint8_t *data,
    size_t capacity);

BspUartDmaResult BspUartDma_Write(
    BspUartDmaPort *port,
    const uint8_t *data,
    size_t length);

#endif
